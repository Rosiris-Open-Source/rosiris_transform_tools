#ifndef TRANSFORMS_RVIZ_PLUGIN_HPP_
#define TRANSFORMS_RVIZ_PLUGIN_HPP_

#include <memory>

#include <QComboBox>
#include <QLabel>
#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>

#include <rviz_common/panel.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <interactive_markers/interactive_marker_server.hpp>

namespace transform_rviz_plugin {

class TFManagerPanel : public rviz_common::Panel {
  Q_OBJECT

public:
  explicit TFManagerPanel(QWidget *parent = nullptr);
  void onInitialize() override;

  // Marker state
  enum class OrientationMode { quaternion, rpy_deg, rpy_rad };

private Q_SLOTS:
  void onToggleInteractive();
  void onFrameChanged();
  void onOrientationModeChanged(int index);
  void refreshFrameList();

private:
  void createInteractiveMarker();
  void clearInteractiveMarker();
  void processFeedback(
      const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr
          &feedback);
  void computeAndDisplayTransform();
  void updateDisplay(const tf2::Transform &tf);

  // ROS
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> im_server_;

  // UI
  QComboBox *parent_frame_combo_;
  QComboBox *reference_frame_combo_;
  QComboBox *orientation_mode_combo_;
  QPushButton *interactive_toggle_;

  QLabel *translation_label_;
  QLabel *orientation_label_;

  QTimer *frame_refresh_timer_;

  geometry_msgs::msg::Pose marker_pose_;
};
} // namespace transform_rviz_plugin

Q_DECLARE_METATYPE(transform_rviz_plugin::TFManagerPanel::OrientationMode)

#endif
