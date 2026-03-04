// Copyright 2026 Manuel Muth
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef TRANSFORM_MANAGER__TRANSFORM_MANAGER_HPP_
#define TRANSFORM_MANAGER__TRANSFORM_MANAGER_HPP_

#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

// msgs
#include <rosiris_transform_interfaces/msg/pose_stamped_with_target.hpp>
#include <rosiris_transform_interfaces/msg/service_result.hpp>
// srvs
#include <rosiris_transform_interfaces/srv/add_dynamic_frames.hpp>
#include <rosiris_transform_interfaces/srv/add_static_frames.hpp>
#include <rosiris_transform_interfaces/srv/get_poses_in_frames.hpp>
#include <rosiris_transform_interfaces/srv/remove_dynamic_frames.hpp>
#include <rosiris_transform_interfaces/srv/update_dynamic_frames.hpp>

#include <transform_manager/transform_manager_parameters.hpp>

namespace transform_manager {

class TransformManager : public rclcpp::Node {
public:
  explicit TransformManager(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  rosiris_transform_interfaces::msg::ServiceResult addDynamicFrames(
      const std::vector<geometry_msgs::msg::TransformStamped> &frames,
      bool allow_override,
      std::vector<geometry_msgs::msg::TransformStamped> &not_added_frames);

  rosiris_transform_interfaces::msg::ServiceResult updateDynamicFrames(
      const std::vector<geometry_msgs::msg::TransformStamped> &frames,
      bool add_if_not_existent,
      std::vector<geometry_msgs::msg::TransformStamped> &not_updated_frames);

  rosiris_transform_interfaces::msg::ServiceResult
  removeDynamicFrames(const std::vector<std::string> &frame_names,
                      std::vector<std::string> &not_found_frames);

  rosiris_transform_interfaces::msg::ServiceResult addStaticFrames(
      const std::vector<geometry_msgs::msg::TransformStamped> &frames,
      bool allow_override,
      std::vector<geometry_msgs::msg::TransformStamped> &not_added_frames);

  rosiris_transform_interfaces::msg::ServiceResult getPosesInFrames(
      const std::vector<
          rosiris_transform_interfaces::msg::PoseStampedWithTarget>
          &poses_to_transform,
      std::vector<geometry_msgs::msg::PoseStamped> &transformed_poses,
      std::optional<
          std::reference_wrapper<std::vector<geometry_msgs::msg::PoseStamped>>>
          not_transformed_poses = std::nullopt);

  geometry_msgs::msg::PoseStamped
  getPoseInFrame(const geometry_msgs::msg::PoseStamped &pose,
                 const std::string &target_frame) {
    return tf_buffer_.transform(pose, target_frame, tf2::durationFromSec(0.0));
  }

  bool frame_exists_in_tree(const std::string &frame) {
    const auto frames_in_buffer = tf_buffer_.getAllFrameNames();
    return std::find(frames_in_buffer.begin(), frames_in_buffer.end(), frame) !=
           frames_in_buffer.end();
  }

private:
  void getParameters();

  void setup_service_servers();

  void publishDynamicFrames();

  void addDynamicFramesCb(
      const std::shared_ptr<
          rosiris_transform_interfaces::srv::AddDynamicFrames::Request>
          request,
      const std::shared_ptr<
          rosiris_transform_interfaces::srv::AddDynamicFrames::Response>
          response);

  void updateDynamicFramesCb(
      const std::shared_ptr<
          rosiris_transform_interfaces::srv::UpdateDynamicFrames::Request>
          request,
      const std::shared_ptr<
          rosiris_transform_interfaces::srv::UpdateDynamicFrames::Response>
          response);

  void removeDynamicFramesCb(
      const std::shared_ptr<
          rosiris_transform_interfaces::srv::RemoveDynamicFrames::Request>
          request,
      const std::shared_ptr<
          rosiris_transform_interfaces::srv::RemoveDynamicFrames::Response>
          response);

  void addStaticFramesCb(
      const std::shared_ptr<
          rosiris_transform_interfaces::srv::AddStaticFrames::Request>
          request,
      const std::shared_ptr<
          rosiris_transform_interfaces::srv::AddStaticFrames::Response>
          response);

  void getPosesInFramesCb(
      const std::shared_ptr<
          rosiris_transform_interfaces::srv::GetPosesInFrames::Request>
          request,
      const std::shared_ptr<
          rosiris_transform_interfaces::srv::GetPosesInFrames::Response>
          response);

  bool dynamic_frame_exists(
      const geometry_msgs::msg::TransformStamped &transform) const {
    return dynamic_frames_.find(transform.child_frame_id) !=
           dynamic_frames_.end();
  }

  bool dynamic_frame_exists(const std::string &frame_name) const {
    return dynamic_frames_.find(frame_name) != dynamic_frames_.end();
  }

  void setSuccess(rosiris_transform_interfaces::msg::ServiceResult &result,
                  const std::string &message);
  void setError(rosiris_transform_interfaces::msg::ServiceResult &result,
                const std::string &message);

  // parameters
  std::unique_ptr<transform_manager::ParamListener> param_listener_;
  transform_manager::Params params_;

  // needed for lookup and calculation of transforms
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // ### Dynamic Frames ###
  // store dynamic transforms
  std::unordered_map<std::string, geometry_msgs::msg::TransformStamped>
      dynamic_frames_;
  std::mutex dynamic_frames_mutex_;
  // publishing of dynamic transforms
  std::shared_ptr<tf2_ros::TransformBroadcaster> dynamic_tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr dynamic_timer_;
  double publish_frequency_;

  // ### Static Frames ###
  // store static transforms
  std::unordered_map<std::string, geometry_msgs::msg::TransformStamped>
      static_frames_;
  std::mutex static_frames_mutex_;
  // publish static frames
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

  // provided services
  rclcpp::Service<rosiris_transform_interfaces::srv::AddDynamicFrames>::
      SharedPtr add_dynamic_srv_;
  rclcpp::Service<rosiris_transform_interfaces::srv::RemoveDynamicFrames>::
      SharedPtr remove_dynamic_srv_;
  rclcpp::Service<rosiris_transform_interfaces::srv::UpdateDynamicFrames>::
      SharedPtr update_dynamic_srv_;

  rclcpp::Service<rosiris_transform_interfaces::srv::AddStaticFrames>::SharedPtr
      add_static_srv_;

  rclcpp::Service<rosiris_transform_interfaces::srv::GetPosesInFrames>::
      SharedPtr get_pose_srv_;
};

} // namespace transform_manager

#endif // TRANSFORM_MANAGER__TRANSFORM_MANAGER_HPP_