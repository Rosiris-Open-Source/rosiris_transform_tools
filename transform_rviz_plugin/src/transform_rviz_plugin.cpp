#include "transform_rviz_plugin/transform_rviz_plugin.hpp"

#include <rviz_common/display_context.hpp>

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>

namespace transform_rviz_plugin {

TFManagerPanel::TFManagerPanel(QWidget *parent) : rviz_common::Panel(parent) {

  QVBoxLayout *layout = new QVBoxLayout;

  parent_frame_combo_ = new QComboBox;
  reference_frame_combo_ = new QComboBox;

  parent_frame_combo_->setEditable(true);
  reference_frame_combo_->setEditable(true);

  qRegisterMetaType<OrientationMode>();
  orientation_mode_combo_ = new QComboBox;
  orientation_mode_combo_->addItem(
      "Quaternion", QVariant::fromValue(OrientationMode::quaternion));
  orientation_mode_combo_->addItem(
      "RPY (deg)", QVariant::fromValue(OrientationMode::rpy_deg));
  orientation_mode_combo_->addItem(
      "RPY (rad)", QVariant::fromValue(OrientationMode::rpy_rad));

  interactive_toggle_ = new QPushButton("Enable Interactive Marker");
  interactive_toggle_->setCheckable(true);

  translation_label_ = new QLabel("Translation: ");
  orientation_label_ = new QLabel("Orientation: ");

  layout->addWidget(new QLabel("Parent Frame"));
  layout->addWidget(parent_frame_combo_);
  layout->addWidget(new QLabel("Reference Frame"));
  layout->addWidget(reference_frame_combo_);
  layout->addWidget(new QLabel("Orientation Format"));
  layout->addWidget(orientation_mode_combo_);
  layout->addWidget(interactive_toggle_);
  layout->addWidget(translation_label_);
  layout->addWidget(orientation_label_);

  setLayout(layout);

  connect(interactive_toggle_, &QPushButton::clicked, this,
          &TFManagerPanel::onToggleInteractive);

  connect(parent_frame_combo_, &QComboBox::currentTextChanged, this,
          &TFManagerPanel::onFrameChanged);

  connect(reference_frame_combo_, &QComboBox::currentTextChanged, this,
          &TFManagerPanel::onFrameChanged);

  connect(orientation_mode_combo_,
          QOverload<int>::of(&QComboBox::currentIndexChanged), this,
          &TFManagerPanel::onOrientationModeChanged);
}

void TFManagerPanel::onInitialize() {

  node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  im_server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>(
      "transform_marker_server", node_);

  frame_refresh_timer_ = new QTimer(this);
  connect(frame_refresh_timer_, &QTimer::timeout, this,
          &TFManagerPanel::refreshFrameList);
  frame_refresh_timer_->start(2000);
}

void TFManagerPanel::refreshFrameList() {
  parent_frame_combo_->clear();
  reference_frame_combo_->clear();

  auto frames = tf_buffer_->getAllFrameNames();
  for (const auto &f : frames) {
    parent_frame_combo_->addItem(QString::fromStdString(f));
    reference_frame_combo_->addItem(QString::fromStdString(f));
  }
}

void TFManagerPanel::onToggleInteractive() {
  if (interactive_toggle_->isChecked()) {
    createInteractiveMarker();
  } else {
    clearInteractiveMarker();
  }
}

void TFManagerPanel::createInteractiveMarker() {

  clearInteractiveMarker();

  visualization_msgs::msg::InteractiveMarker int_marker;
  int_marker.header.frame_id = parent_frame_combo_->currentText().toStdString();
  int_marker.name = "transform_marker";
  int_marker.scale = 1.0;
  int_marker.pose = marker_pose_;

  visualization_msgs::msg::InteractiveMarkerControl control;

  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.name = "rotate_x";
  control.interaction_mode =
      visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);

  control.name = "move_x";
  control.interaction_mode =
      visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  control.orientation.x = 0;
  control.orientation.y = 1;
  control.name = "rotate_y";
  control.interaction_mode =
      visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);

  control.name = "move_y";
  control.interaction_mode =
      visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  control.orientation.y = 0;
  control.orientation.z = 1;
  control.name = "rotate_z";
  control.interaction_mode =
      visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);

  control.name = "move_z";
  control.interaction_mode =
      visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  im_server_->insert(int_marker, std::bind(&TFManagerPanel::processFeedback,
                                           this, std::placeholders::_1));

  im_server_->applyChanges();
}

void TFManagerPanel::clearInteractiveMarker() {
  im_server_->clear();
  im_server_->applyChanges();
}

void TFManagerPanel::processFeedback(
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr
        &feedback) {

  marker_pose_ = feedback->pose;
  computeAndDisplayTransform();
}

void TFManagerPanel::onFrameChanged() {
  if (interactive_toggle_->isChecked())
    createInteractiveMarker();
}

void TFManagerPanel::onOrientationModeChanged(int /*index*/) {
  computeAndDisplayTransform();
}

void TFManagerPanel::computeAndDisplayTransform() {

  if (parent_frame_combo_->currentText().isEmpty() ||
      reference_frame_combo_->currentText().isEmpty())
    return;

  try {
    auto tf_msg = tf_buffer_->lookupTransform(
        reference_frame_combo_->currentText().toStdString(),
        parent_frame_combo_->currentText().toStdString(), tf2::TimePointZero);

    tf2::Transform ref_to_parent;
    tf2::fromMsg(tf_msg.transform, ref_to_parent);

    tf2::Transform parent_to_marker;
    tf2::fromMsg(marker_pose_, parent_to_marker);

    tf2::Transform ref_to_marker = ref_to_parent * parent_to_marker;

    updateDisplay(ref_to_marker);

  } catch (tf2::TransformException &) {
    return;
  }
}

void TFManagerPanel::updateDisplay(const tf2::Transform &tf) {

  auto t = tf.getOrigin();
  auto q = tf.getRotation();

  translation_label_->setText(QString("Translation: x=%1  y=%2  z=%3")
                                  .arg(t.x())
                                  .arg(t.y())
                                  .arg(t.z()));

  OrientationMode mode =
      orientation_mode_combo_->currentData().value<OrientationMode>();

  if (mode == OrientationMode::quaternion) {

    orientation_label_->setText(QString("Quaternion: x=%1  y=%2  z=%3  w=%4")
                                    .arg(q.x())
                                    .arg(q.y())
                                    .arg(q.z())
                                    .arg(q.w()));

  } else {

    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    if (mode == OrientationMode::rpy_deg) {
      roll *= 180.0 / M_PI;
      pitch *= 180.0 / M_PI;
      yaw *= 180.0 / M_PI;
    }

    orientation_label_->setText(
        QString("RPY: r=%1  p=%2  y=%3").arg(roll).arg(pitch).arg(yaw));
  }
}

} // namespace transform_rviz_plugin

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(transform_rviz_plugin::TFManagerPanel,
                       rviz_common::Panel)
