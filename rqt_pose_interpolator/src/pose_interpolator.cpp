/** @file
 *
 * Simple rqt plugin to generate tracking shots.
 *
 * @author Jan Razlaw
 */

#include <rqt_pose_interpolator/pose_interpolator.h>

namespace pose_interpolator {

PoseInterpolator::PoseInterpolator()
: rqt_gui_cpp::Plugin()
  , widget_(0)
{
  cam_pose_.orientation.w = 1.0;

  // give QObjects reasonable names
  setObjectName("PoseInterpolator");
}

void PoseInterpolator::initPlugin(qt_gui_cpp::PluginContext& context)
{
  ros::NodeHandle ph("~");
  camera_pose_sub_ = ph.subscribe("/rviz/current_camera_pose", 1, &PoseInterpolator::camPoseCallback, this);
  camera_placement_pub_ = ph.advertise<rviz_animated_view_controller::CameraTrajectory>("/rviz/camera_trajectory", 1);

  // access standalone command line arguments
  QStringList argv = context.argv();
  // create QWidget
  widget_ = new QWidget();
  // extend the widget with all attributes and children from UI file
  ui_.setupUi(widget_);

  connect(ui_.move_to_start_button, SIGNAL(clicked(bool)), this, SLOT(moveCamToStart()));
  connect(ui_.move_to_end_button, SIGNAL(clicked(bool)), this, SLOT(moveCamToEnd()));
  connect(ui_.set_start_to_cam_button, SIGNAL(clicked(bool)), this, SLOT(setStartToCurrentCam()));
  connect(ui_.set_end_to_cam_button, SIGNAL(clicked(bool)), this, SLOT(setEndToCurrentCam()));
  connect(ui_.frame_text_edit, SIGNAL(textChanged()), this, SLOT(setMarkerFrames()));

  // add widget to the user interface
  context.addWidget(widget_);

  // set up markers
  start_marker_ = makeMarker();
  start_marker_.name = "start_marker";
  start_marker_.description = "Start Marker";
  start_marker_.pose.orientation.w = 1.0;
  start_marker_.controls[0].markers[0].color.g = 1.f;
  tf::Vector3 rotated_vector = rotateVector(tf::Vector3(0, 0, -1), start_marker_.pose.orientation);
  start_look_at_.x = start_marker_.pose.position.x + ui_.smoothness_spin_box->value() * rotated_vector.x();
  start_look_at_.y = start_marker_.pose.position.y + ui_.smoothness_spin_box->value() * rotated_vector.y();
  start_look_at_.z = start_marker_.pose.position.z + ui_.smoothness_spin_box->value() * rotated_vector.z();

  end_marker_ = makeMarker(5.0, 0.0, 0.0);
  end_marker_.name = "end_marker";
  end_marker_.description = "End Marker";
  end_marker_.pose.orientation.w = 1.0;
  end_marker_.controls[0].markers[0].color.r = 1.f;
  rotated_vector = rotateVector(tf::Vector3(0, 0, -1), end_marker_.pose.orientation);
  end_look_at_.x = end_marker_.pose.position.x + ui_.smoothness_spin_box->value() * rotated_vector.x();
  end_look_at_.y = end_marker_.pose.position.y + ui_.smoothness_spin_box->value() * rotated_vector.y();
  end_look_at_.z = end_marker_.pose.position.z + ui_.smoothness_spin_box->value() * rotated_vector.z();

  // connect markers to callback functions
  server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>("trajectory");
  server_->insert(start_marker_, boost::bind( &PoseInterpolator::processFeedback, this, _1));
  menu_handler_.apply(*server_, start_marker_.name);
  server_->insert(end_marker_, boost::bind( &PoseInterpolator::processFeedback, this, _1));
  menu_handler_.apply(*server_, end_marker_.name);

  server_->applyChanges();
}

void PoseInterpolator::shutdownPlugin()
{
  camera_pose_sub_.shutdown();
  camera_placement_pub_.shutdown();
}

void PoseInterpolator::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
{
  // TODO save intrinsic configuration, usually using:
  // instance_settings.setValue(k, v)
}

void PoseInterpolator::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
{
  // TODO restore intrinsic configuration, usually using:
  // v = instance_settings.value(k)
}

void PoseInterpolator::camPoseCallback(const geometry_msgs::Pose::ConstPtr& cam_pose)
{
  cam_pose_ = geometry_msgs::Pose(*cam_pose);
}

void PoseInterpolator::setStartToCurrentCam()
{
  // rviz camera looks into negative z direction
  tf::Vector3 rotated_vector = rotateVector(tf::Vector3(0, 0, -1), cam_pose_.orientation);
  start_look_at_.x = cam_pose_.position.x + ui_.smoothness_spin_box->value() * rotated_vector.x();
  start_look_at_.y = cam_pose_.position.y + ui_.smoothness_spin_box->value() * rotated_vector.y();
  start_look_at_.z = cam_pose_.position.z + ui_.smoothness_spin_box->value() * rotated_vector.z();

  start_marker_.pose = cam_pose_;

  // rotate cam pose around z axis for -90 degrees
  tf::Quaternion cam_orientation;
  tf::quaternionMsgToTF(cam_pose_.orientation, cam_orientation);
  tf::Quaternion rot_around_z_neg_90_deg(0.0, 0.0, -0.707, 0.707);
  tf::quaternionTFToMsg(cam_orientation * rot_around_z_neg_90_deg, start_marker_.pose.orientation);

  server_->setPose(start_marker_.name, start_marker_.pose, start_marker_.header);
  server_->applyChanges();
}

void PoseInterpolator::setEndToCurrentCam()
{
  // rviz camera looks into negative z direction
  tf::Vector3 rotated_vector = rotateVector(tf::Vector3(0, 0, -1), cam_pose_.orientation);
  end_look_at_.x = cam_pose_.position.x + ui_.smoothness_spin_box->value() * rotated_vector.x();
  end_look_at_.y = cam_pose_.position.y + ui_.smoothness_spin_box->value() * rotated_vector.y();
  end_look_at_.z = cam_pose_.position.z + ui_.smoothness_spin_box->value() * rotated_vector.z();

  end_marker_.pose = cam_pose_;

  // rotate cam pose around z axis for -90 degrees
  tf::Quaternion cam_orientation;
  tf::quaternionMsgToTF(cam_pose_.orientation, cam_orientation);
  tf::Quaternion rot_around_z_neg_90_deg(0.0, 0.0, -0.707, 0.707);
  tf::quaternionTFToMsg(cam_orientation * rot_around_z_neg_90_deg, end_marker_.pose.orientation);

  server_->setPose(end_marker_.name, end_marker_.pose, end_marker_.header);
  server_->applyChanges();
}

void PoseInterpolator::setMarkerFrames()
{
  start_marker_.header.frame_id = ui_.frame_text_edit->toPlainText().toStdString();
  end_marker_.header.frame_id = ui_.frame_text_edit->toPlainText().toStdString();
  server_->erase("start_marker");
  server_->erase("end_marker");
  server_->insert(start_marker_);
  server_->insert(end_marker_);
  server_->applyChanges();
}

rviz_animated_view_controller::CameraMovement PoseInterpolator::makeCameraMovement()
{
  rviz_animated_view_controller::CameraMovement cp;
  cp.eye.header.stamp = ros::Time::now();
  cp.eye.header.frame_id = ui_.frame_text_edit->toPlainText().toStdString();

  cp.up.header = cp.focus.header = cp.eye.header;

  cp.up.vector.x = 0.0;
  cp.up.vector.y = 0.0;
  cp.up.vector.z = 1.0;

  return cp;
}

visualization_msgs::InteractiveMarker PoseInterpolator::makeMarker(double x, double y, double z)
{
  visualization_msgs::InteractiveMarker marker;
  marker.header.frame_id = ui_.frame_text_edit->toPlainText().toStdString();
  marker.name = "marker";
  marker.description = "Marker";
  marker.scale = 2.22;
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = z;
  marker.pose.orientation.w = 1.0;
  marker.pose.orientation.y = 1.0;

  makeBoxControl(marker);

  visualization_msgs::InteractiveMarkerControl pose_control;
  pose_control.orientation.w = 1;
  pose_control.orientation.x = 1;
  pose_control.orientation.y = 0;
  pose_control.orientation.z = 0;
  pose_control.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;
  pose_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE;
  marker.controls.push_back(pose_control);

  pose_control.orientation.x = 0;
  pose_control.orientation.y = 1;
  pose_control.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;
  pose_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE;
  marker.controls.push_back(pose_control);

  pose_control.orientation.y = 0;
  pose_control.orientation.z = 1;
  pose_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE;
  marker.controls.push_back(pose_control);
  return marker;
}


void PoseInterpolator::moveCamToStart()
{
  moveCamToStart(ui_.transition_time_spin_box->value());
}

void PoseInterpolator::moveCamToStart(double transition_time)
{
  // fill Camera Trajectory msg with markers and times
  rviz_animated_view_controller::CameraTrajectoryPtr cam_trajectory(new rviz_animated_view_controller::CameraTrajectory());
  cam_trajectory->target_frame = ui_.frame_text_edit->toPlainText().toStdString();
  cam_trajectory->allow_free_yaw_axis = !ui_.use_up_of_world_radio_button->isChecked();

  rviz_animated_view_controller::CameraMovement cp = makeCameraMovement();
  cp.transition_time = ros::Duration(transition_time);

  if(!ui_.use_up_of_world_radio_button->isChecked())
  {
    // in the cam frame up is the negative x direction
    tf::Vector3 rotated_vector = rotateVector(tf::Vector3(-1, 0, 0), start_marker_.pose.orientation);
    cp.up.vector.x = rotated_vector.x();
    cp.up.vector.y = rotated_vector.y();
    cp.up.vector.z = rotated_vector.z();
  }

  cp.eye.point = start_marker_.pose.position;
  cp.focus.point = start_look_at_;

  cam_trajectory->trajectory.push_back(cp);
  camera_placement_pub_.publish(cam_trajectory);
}

void PoseInterpolator::moveCamToEnd()
{
  // fill Camera Trajectory msg with markers and times
  rviz_animated_view_controller::CameraTrajectoryPtr cam_trajectory(new rviz_animated_view_controller::CameraTrajectory());
  cam_trajectory->target_frame = ui_.frame_text_edit->toPlainText().toStdString();
  cam_trajectory->allow_free_yaw_axis = !ui_.use_up_of_world_radio_button->isChecked();

  rviz_animated_view_controller::CameraMovement cp = makeCameraMovement();
  cp.transition_time = ros::Duration(ui_.transition_time_spin_box->value());

  if(!ui_.use_up_of_world_radio_button->isChecked())
  {
    // in the cam frame up is the negative x direction 
    tf::Vector3 rotated_vector = rotateVector(tf::Vector3(-1, 0, 0), end_marker_.pose.orientation);
    cp.up.vector.x = rotated_vector.x();
    cp.up.vector.y = rotated_vector.y();
    cp.up.vector.z = rotated_vector.z();
  }

  cp.eye.point = end_marker_.pose.position;
  cp.focus.point = end_look_at_;

  cam_trajectory->trajectory.push_back(cp);
  camera_placement_pub_.publish(cam_trajectory);
}

void PoseInterpolator::processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
  if(feedback->marker_name == "start_marker")
  {
    start_marker_.pose = feedback->pose;

    tf::Vector3 rotated_vector = rotateVector(tf::Vector3(0, 0, -1), start_marker_.pose.orientation);
    start_look_at_.x = start_marker_.pose.position.x + ui_.smoothness_spin_box->value() * rotated_vector.x();
    start_look_at_.y = start_marker_.pose.position.y + ui_.smoothness_spin_box->value() * rotated_vector.y();
    start_look_at_.z = start_marker_.pose.position.z + ui_.smoothness_spin_box->value() * rotated_vector.z();
  }
  else
  {
    end_marker_.pose = feedback->pose;

    tf::Vector3 rotated_vector = rotateVector(tf::Vector3(0, 0, -1), end_marker_.pose.orientation);
    end_look_at_.x = end_marker_.pose.position.x + ui_.smoothness_spin_box->value() * rotated_vector.x();
    end_look_at_.y = end_marker_.pose.position.y + ui_.smoothness_spin_box->value() * rotated_vector.y();
    end_look_at_.z = end_marker_.pose.position.z + ui_.smoothness_spin_box->value() * rotated_vector.z();
  }
}

tf::Vector3 PoseInterpolator::rotateVector(const tf::Vector3 vector,
                                           const geometry_msgs::Quaternion& quat)
{
  tf::Quaternion rotation;
  tf::quaternionMsgToTF(quat, rotation);
  return tf::quatRotate(rotation, vector);
}


} // namespace