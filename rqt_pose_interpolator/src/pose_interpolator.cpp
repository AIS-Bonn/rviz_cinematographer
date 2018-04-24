/** @file
 *
 * Simple rqt plugin to generate tracking shots.
 *
 * @author Jan Razlaw
 */

#include <rqt_pose_interpolator/pose_interpolator.h>
#include <pluginlib/class_list_macros.h> //TODO ?

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
  camera_placement_pub_ = ph.advertise<view_controller_msgs::CameraPlacement>("/rviz/camera_placement", 1);

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
  end_marker_ = makeMarker(5.0, 0.0, 0.0);
  end_marker_.name = "end_marker";
  end_marker_.description = "End Marker";
  end_marker_.pose.orientation.w = 1.0;
  end_marker_.controls[0].markers[0].color.r = 1.f;

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
  ui_.messages_label->setText(QString("Message: "));

  if(ui_.start_x_spin_box->maximum() < cam_pose_.position.x ||
     ui_.start_x_spin_box->minimum() > cam_pose_.position.x ||
     ui_.start_y_spin_box->maximum() < cam_pose_.position.y ||
     ui_.start_y_spin_box->minimum() > cam_pose_.position.y ||
     ui_.start_z_spin_box->maximum() < cam_pose_.position.z ||
     ui_.start_z_spin_box->minimum() > cam_pose_.position.z )
  {
    ui_.messages_label->setText(QString("Message: Current position is out of scope.\n\tTry moving closer to the center of the frame."));
    return;
  }
  
  ui_.start_x_spin_box->setValue(cam_pose_.position.x);
  ui_.start_y_spin_box->setValue(cam_pose_.position.y);
  ui_.start_z_spin_box->setValue(cam_pose_.position.z);

  // rviz camera looks into negative z direction
  tf::Vector3 rotated_vector = rotateVector(tf::Vector3(0, 0, -1), cam_pose_.orientation);
  start_look_at_.x = cam_pose_.position.x + ui_.smoothness_spin_box->value() * rotated_vector.x();
  start_look_at_.y = cam_pose_.position.y + ui_.smoothness_spin_box->value() * rotated_vector.y();
  start_look_at_.z = cam_pose_.position.z + ui_.smoothness_spin_box->value() * rotated_vector.z();
}

void PoseInterpolator::setEndToCurrentCam()
{
  ui_.messages_label->setText(QString("Message: "));

  if(ui_.end_x_spin_box->maximum() < cam_pose_.position.x ||
     ui_.end_x_spin_box->minimum() > cam_pose_.position.x ||
     ui_.end_y_spin_box->maximum() < cam_pose_.position.y ||
     ui_.end_y_spin_box->minimum() > cam_pose_.position.y ||
     ui_.end_z_spin_box->maximum() < cam_pose_.position.z ||
     ui_.end_z_spin_box->minimum() > cam_pose_.position.z )
  {
    ui_.messages_label->setText(QString("Message: Current position is out of scope.\n\tTry moving closer to the center of the frame."));
    return;
  }

  ui_.end_x_spin_box->setValue(cam_pose_.position.x);
  ui_.end_y_spin_box->setValue(cam_pose_.position.y);
  ui_.end_z_spin_box->setValue(cam_pose_.position.z);

  // rviz camera looks into negative z direction
  tf::Vector3 rotated_vector = rotateVector(tf::Vector3(0, 0, -1), cam_pose_.orientation);
  end_look_at_.x = cam_pose_.position.x + ui_.smoothness_spin_box->value() * rotated_vector.x();
  end_look_at_.y = cam_pose_.position.y + ui_.smoothness_spin_box->value() * rotated_vector.y();
  end_look_at_.z = cam_pose_.position.z + ui_.smoothness_spin_box->value() * rotated_vector.z();
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

view_controller_msgs::CameraPlacement PoseInterpolator::makeCameraPlacement()
{
  view_controller_msgs::CameraPlacement cp;
  cp.eye.header.stamp = ros::Time::now();
  cp.eye.header.frame_id = ui_.frame_text_edit->toPlainText().toStdString();
  cp.target_frame = ui_.frame_text_edit->toPlainText().toStdString();
  cp.interpolation_mode = view_controller_msgs::CameraPlacement::FPS; // SPHERICAL
  cp.time_from_start = ros::Duration(0);

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
  view_controller_msgs::CameraPlacement cp = makeCameraPlacement();
  cp.time_from_start = ros::Duration(transition_time);

  if(!ui_.use_up_of_world_radio_button->isChecked())
  {
    // in the cam frame up is the negative x direction
    tf::Vector3 rotated_vector = rotateVector(tf::Vector3(-1, 0, 0), start_marker_.pose.orientation);
    cp.up.vector.x = rotated_vector.x();
    cp.up.vector.y = rotated_vector.y();
    cp.up.vector.z = rotated_vector.z();
  }

  geometry_msgs::Point look_from;
  look_from.x = ui_.start_x_spin_box->value();
  look_from.y = ui_.start_y_spin_box->value();
  look_from.z = ui_.start_z_spin_box->value();
  cp.eye.point = look_from;

  cp.focus.point = start_look_at_;

  camera_placement_pub_.publish(cp);
}

void PoseInterpolator::moveCamToEnd()
{
  view_controller_msgs::CameraPlacement cp = makeCameraPlacement();
  cp.time_from_start = ros::Duration(ui_.transition_time_spin_box->value());

  if(!ui_.use_up_of_world_radio_button->isChecked())
  {
    // in the cam frame up is the negative x direction 
    tf::Vector3 rotated_vector = rotateVector(tf::Vector3(-1, 0, 0), end_marker_.pose.orientation);
    cp.up.vector.x = rotated_vector.x();
    cp.up.vector.y = rotated_vector.y();
    cp.up.vector.z = rotated_vector.z();
  }

  geometry_msgs::Point look_from;
  look_from.x = ui_.end_x_spin_box->value();
  look_from.y = ui_.end_y_spin_box->value();
  look_from.z = ui_.end_z_spin_box->value();
  cp.eye.point = look_from;

  cp.focus.point = end_look_at_;

  camera_placement_pub_.publish(cp);
}

void PoseInterpolator::processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
  if(feedback->marker_name == "start_marker")
  {
    start_marker_.pose = feedback->pose;

    ui_.start_x_spin_box->setValue(start_marker_.pose.position.x);
    ui_.start_y_spin_box->setValue(start_marker_.pose.position.y);
    ui_.start_z_spin_box->setValue(start_marker_.pose.position.z);

    tf::Vector3 rotated_vector = rotateVector(tf::Vector3(0, 0, -1), start_marker_.pose.orientation);
    start_look_at_.x = start_marker_.pose.position.x + ui_.smoothness_spin_box->value() * rotated_vector.x();
    start_look_at_.y = start_marker_.pose.position.y + ui_.smoothness_spin_box->value() * rotated_vector.y();
    start_look_at_.z = start_marker_.pose.position.z + ui_.smoothness_spin_box->value() * rotated_vector.z();
  }
  else
  {
    end_marker_.pose = feedback->pose;

    ui_.end_x_spin_box->setValue(end_marker_.pose.position.x);
    ui_.end_y_spin_box->setValue(end_marker_.pose.position.y);
    ui_.end_z_spin_box->setValue(end_marker_.pose.position.z);

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