#include "transform_manager/transform_manager.hpp"

#include <chrono>

using namespace transform_manager;

TransformManager::TransformManager(const rclcpp::NodeOptions &options)
    : Node("transform_manager", options), tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_) {
  dynamic_tf_broadcaster_ =
      std::make_shared<tf2_ros::TransformBroadcaster>(this);
  static_tf_broadcaster_ =
      std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

  // Parameter
  getParameters();

  // start publisher for the dynamic frames
  const auto period = std::chrono::duration<double>(
      1.0 / params_.dynamic_frames_publish_frequency);

  dynamic_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&TransformManager::publishDynamicFrames, this));

  if (params_.enable_services) {
    setup_service_servers();
  }
}

void TransformManager::getParameters() {
  param_listener_ = std::make_unique<transform_manager::ParamListener>(
      get_node_parameters_interface());
  params_ = param_listener_->get_params();

  RCLCPP_INFO(get_logger(),
              "Publishing frequency of dynamic frames set to %fHz.",
              params_.dynamic_frames_publish_frequency);

  RCLCPP_INFO(get_logger(), "Publishing services are %s.",
              params_.enable_services ? "enabled" : "disabled");
}

void TransformManager::setup_service_servers() {
  add_dynamic_srv_ =
      this->create_service<rosiris_transform_interfaces::srv::AddDynamicFrames>(
          "~/add_dynamic_frames",
          std::bind(&TransformManager::addDynamicFramesCb, this,
                    std::placeholders::_1, std::placeholders::_2));

  update_dynamic_srv_ = this->create_service<
      rosiris_transform_interfaces::srv::UpdateDynamicFrames>(
      "~/update_dynamic_frames",
      std::bind(&TransformManager::updateDynamicFramesCb, this,
                std::placeholders::_1, std::placeholders::_2));

  remove_dynamic_srv_ = this->create_service<
      rosiris_transform_interfaces::srv::RemoveDynamicFrames>(
      "~/remove_dynamic_frames",
      std::bind(&TransformManager::removeDynamicFramesCb, this,
                std::placeholders::_1, std::placeholders::_2));

  add_static_srv_ =
      this->create_service<rosiris_transform_interfaces::srv::AddStaticFrames>(
          "~/add_static_frames",
          std::bind(&TransformManager::addStaticFramesCb, this,
                    std::placeholders::_1, std::placeholders::_2));

  get_pose_srv_ =
      this->create_service<rosiris_transform_interfaces::srv::GetPosesInFrames>(
          "~/get_pose_in_frame",
          std::bind(&TransformManager::getPosesInFramesCb, this,
                    std::placeholders::_1, std::placeholders::_2));
}

rosiris_transform_interfaces::msg::ServiceResult
TransformManager::addDynamicFrames(
    const std::vector<geometry_msgs::msg::TransformStamped> &frames,
    bool allow_override,
    std::vector<geometry_msgs::msg::TransformStamped> &not_added_frames) {
  std::lock_guard<std::mutex> lock(dynamic_frames_mutex_);
  {
    for (const auto &transform : frames) {
      const std::string parent = transform.header.frame_id;
      const std::string frame_name = transform.child_frame_id;
      const bool empty_parent = parent.empty();
      if (empty_parent || !frame_exists_in_tree(parent)) {
        not_added_frames.push_back(transform);
        RCLCPP_WARN(get_logger(), "Do not add frame: Parent frame for %s %s",
                    frame_name.c_str(),
                    empty_parent ? "is empty." : "does not exist in tree.");
        continue;
      }
      if (allow_override || !dynamic_frame_exists(transform)) {
        dynamic_frames_[frame_name] = transform;
      } else {
        not_added_frames.push_back(transform);
        RCLCPP_WARN(
            get_logger(),
            "Do not add frame: Transform %s exists and override parameter "
            "is not set.",
            frame_name.c_str());
      }
    }
  }

  rosiris_transform_interfaces::msg::ServiceResult result{};
  if (not_added_frames.empty()) {
    setSuccess(result, "All frames successfully added.");
  } else {
    setError(result, "Not all frames added.");
  }
  return result;
}

rosiris_transform_interfaces::msg::ServiceResult
TransformManager::updateDynamicFrames(
    const std::vector<geometry_msgs::msg::TransformStamped> &frames,
    bool add_if_not_existent,
    std::vector<geometry_msgs::msg::TransformStamped> &not_updated_frames) {
  if (add_if_not_existent) {
    return addDynamicFrames(frames, true, not_updated_frames);
  }
  {
    std::lock_guard<std::mutex> lock(dynamic_frames_mutex_);
    for (const auto &transform : frames) {
      const std::string frame_name = transform.child_frame_id;
      if (dynamic_frame_exists(frame_name)) {
        dynamic_frames_.at(frame_name) = transform;
      } else {
        not_updated_frames.push_back(transform);
        RCLCPP_WARN(get_logger(), "Frame %s not updated. The frame is unknow.",
                    frame_name.c_str());
      }
    }
  }

  rosiris_transform_interfaces::msg::ServiceResult result{};
  if (not_updated_frames.empty()) {
    setSuccess(result, "All frames successfully updated.");
  } else {
    setError(result, "Not all frames updated.");
  }
  return result;
}

rosiris_transform_interfaces::msg::ServiceResult
TransformManager::removeDynamicFrames(
    const std::vector<std::string> &frame_names,
    std::vector<std::string> &not_found_frames) {
  {
    std::lock_guard<std::mutex> lock(dynamic_frames_mutex_);
    for (const auto &frame_name : frame_names) {
      if (!dynamic_frames_.erase(frame_name)) {
        RCLCPP_WARN(get_logger(), "Frame %s not updated. The frame is unknow.",
                    frame_name.c_str());
        not_found_frames.push_back(frame_name);
      }
    }
  }

  rosiris_transform_interfaces::msg::ServiceResult result{};
  if (not_found_frames.empty()) {
    setSuccess(result, "All frames successfully removed.");
  } else {
    setError(result, "Not all frames found which should be removed.");
  }

  return result;
}

rosiris_transform_interfaces::msg::ServiceResult
TransformManager::addStaticFrames(
    const std::vector<geometry_msgs::msg::TransformStamped> &frames,
    bool allow_override,
    std::vector<geometry_msgs::msg::TransformStamped> &not_added_frames) {

  for (const auto &transform : frames) {
    const std::string parent = transform.header.frame_id;
    const std::string frame_name = transform.child_frame_id;
    const bool empty_parent = parent.empty();
    if (empty_parent || !frame_exists_in_tree(parent)) {
      not_added_frames.push_back(transform);
      RCLCPP_WARN(get_logger(), "Do not add frame: Parent frame for %s %s",
                  frame_name.c_str(),
                  empty_parent ? "is empty." : "does not exist in tree.");
      continue;
    }
    if (allow_override || !frame_exists_in_tree(parent)) {
      static_tf_broadcaster_->sendTransform(transform);
    } else {
      RCLCPP_WARN(
          get_logger(),
          "Frame %s not added. Frame exists and allow_override not set.",
          frame_name.c_str());
      not_added_frames.push_back(transform);
    }
  }

  rosiris_transform_interfaces::msg::ServiceResult result{};
  if (not_added_frames.empty()) {
    setSuccess(result, "All frames successfully added.");
  } else {
    setError(result, "Not all frames added.");
  }
  return result;
}

rosiris_transform_interfaces::msg::ServiceResult
TransformManager::getPosesInFrames(
    const std::vector<rosiris_transform_interfaces::msg::PoseStampedWithTarget>
        &poses_to_transform,
    std::vector<geometry_msgs::msg::PoseStamped> &transformed_poses,
    std::optional<
        std::reference_wrapper<std::vector<geometry_msgs::msg::PoseStamped>>>
        not_transformed_poses) {
  for (const auto &pose_with_target : poses_to_transform) {
    try {
      auto transformed =
          getPoseInFrame(pose_with_target.pose, pose_with_target.target_frame);

      transformed_poses.push_back(transformed);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "%s", ex.what());
      if (not_transformed_poses) {
        not_transformed_poses->get().push_back(pose_with_target.pose);
      }
    }
  }

  rosiris_transform_interfaces::msg::ServiceResult result{};
  if (!not_transformed_poses || not_transformed_poses->get().empty()) {
    setSuccess(result, "All poses successfully transformed.");
  } else {
    setError(result, "Not all poses transformed.");
  }
  return result;
}

void TransformManager::publishDynamicFrames() {
  std::vector<geometry_msgs::msg::TransformStamped> transforms;
  {
    std::lock_guard<std::mutex> lock(dynamic_frames_mutex_);
    transforms.reserve(dynamic_frames_.size());

    for (auto &[_, transform] : dynamic_frames_) {
      transform.header.stamp = this->now();
      transforms.push_back(transform);
    }
  }

  if (!transforms.empty()) {
    dynamic_tf_broadcaster_->sendTransform(transforms);
  }
}

void TransformManager::addDynamicFramesCb(
    const std::shared_ptr<
        rosiris_transform_interfaces::srv::AddDynamicFrames::Request>
        request,
    const std::shared_ptr<
        rosiris_transform_interfaces::srv::AddDynamicFrames::Response>
        response) {
  response->result = addDynamicFrames(request->frames, request->allow_override,
                                      response->not_added_frames);
}

void TransformManager::updateDynamicFramesCb(
    const std::shared_ptr<
        rosiris_transform_interfaces::srv::UpdateDynamicFrames::Request>
        request,
    const std::shared_ptr<
        rosiris_transform_interfaces::srv::UpdateDynamicFrames::Response>
        response) {
  response->result =
      updateDynamicFrames(request->frames_to_update, request->if_not_exists_add,
                          response->not_updated_frames);
}

void TransformManager::removeDynamicFramesCb(
    const std::shared_ptr<
        rosiris_transform_interfaces::srv::RemoveDynamicFrames::Request>
        request,
    const std::shared_ptr<
        rosiris_transform_interfaces::srv::RemoveDynamicFrames::Response>
        response) {
  response->result =
      removeDynamicFrames(request->frame_names, response->not_removed_frames);
}

void TransformManager::addStaticFramesCb(
    const std::shared_ptr<
        rosiris_transform_interfaces::srv::AddStaticFrames::Request>
        request,
    const std::shared_ptr<
        rosiris_transform_interfaces::srv::AddStaticFrames::Response>
        response) {
  response->result = addStaticFrames(request->frames, request->allow_override,
                                     response->not_added_frames);
}

void TransformManager::getPosesInFramesCb(
    const std::shared_ptr<
        rosiris_transform_interfaces::srv::GetPosesInFrames::Request>
        request,
    const std::shared_ptr<
        rosiris_transform_interfaces::srv::GetPosesInFrames::Response>
        response) {
  response->result = getPosesInFrames(request->poses_to_transform,
                                      response->transformed_poses);
}

void TransformManager::setSuccess(
    rosiris_transform_interfaces::msg::ServiceResult &result,
    const std::string &message) {
  result.result = rosiris_transform_interfaces::msg::ServiceResult::SUCCESS;
  result.message = message;
}

void TransformManager::setError(
    rosiris_transform_interfaces::msg::ServiceResult &result,
    const std::string &message) {
  result.result = rosiris_transform_interfaces::msg::ServiceResult::ERROR;
  result.message = message;
}