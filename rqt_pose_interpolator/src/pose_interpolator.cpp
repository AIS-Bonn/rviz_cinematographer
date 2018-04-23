//
// Created by razlaw on 19.04.18.
//

#include <rqt_pose_interpolator/pose_interpolator.h>
#include <pluginlib/class_list_macros.h>
#include <QStringList>

namespace pose_interpolator {

PoseInterpolator::PoseInterpolator()
: rqt_gui_cpp::Plugin()
  , widget_(0)
{
  // Constructor is called first before initPlugin function, needless to say.
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

  // add widget to the user interface
  context.addWidget(widget_);
}

void PoseInterpolator::shutdownPlugin()
{
  // TODO unregister all publishers here
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

  tf::Quaternion rotation;
  tf::quaternionMsgToTF(cam_pose_.orientation , rotation);
  tf::Vector3 vector(0, 0, 1);
  tf::Vector3 rotated_vector = tf::quatRotate(rotation, vector);
  
  ui_.start_x_spin_box->setValue(cam_pose_.position.x);
  ui_.start_y_spin_box->setValue(cam_pose_.position.y);
  ui_.start_z_spin_box->setValue(cam_pose_.position.z);

  start_look_at_.x = cam_pose_.position.x - ui_.smoothness_spin_box->value() * rotated_vector.x();
  start_look_at_.y = cam_pose_.position.y - ui_.smoothness_spin_box->value() * rotated_vector.y();
  start_look_at_.z = cam_pose_.position.z - ui_.smoothness_spin_box->value() * rotated_vector.z();

//  moveCamToStart(ui_.transition_time_spin_box->minimum());
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

  tf::Quaternion rotation;
  tf::quaternionMsgToTF(cam_pose_.orientation , rotation);
  tf::Vector3 vector(0, 0, 1);
  tf::Vector3 rotated_vector = tf::quatRotate(rotation, vector);

  ui_.end_x_spin_box->setValue(cam_pose_.position.x);
  ui_.end_y_spin_box->setValue(cam_pose_.position.y);
  ui_.end_z_spin_box->setValue(cam_pose_.position.z);

  end_look_at_.x = cam_pose_.position.x - ui_.smoothness_spin_box->value() * rotated_vector.x();
  end_look_at_.y = cam_pose_.position.y - ui_.smoothness_spin_box->value() * rotated_vector.y();
  end_look_at_.z = cam_pose_.position.z - ui_.smoothness_spin_box->value() * rotated_vector.z();

//  moveCamToEnd();
}

view_controller_msgs::CameraPlacement PoseInterpolator::makeCameraPlacement()
{
  view_controller_msgs::CameraPlacement cp;
  cp.eye.header.stamp = ros::Time::now();
  cp.eye.header.frame_id = "base_link"; // TODO: make parameter
  cp.target_frame = "base_link";
  cp.interpolation_mode = view_controller_msgs::CameraPlacement::FPS; // SPHERICAL
  cp.time_from_start = ros::Duration(0);

  cp.up.header = cp.focus.header = cp.eye.header;
  cp.up.vector.x = 0.0;
  cp.up.vector.y = 0.0;
  cp.up.vector.z = 1.0;

  return cp;
}

void PoseInterpolator::moveCamToStart()
{
  moveCamToStart(ui_.transition_time_spin_box->value());
}

void PoseInterpolator::moveCamToStart(double transition_time)
{
  view_controller_msgs::CameraPlacement cp = makeCameraPlacement();
  cp.time_from_start = ros::Duration(transition_time);

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

  geometry_msgs::Point look_from;
  look_from.x = ui_.end_x_spin_box->value();
  look_from.y = ui_.end_y_spin_box->value();
  look_from.z = ui_.end_z_spin_box->value();
  cp.eye.point = look_from;
  
  cp.focus.point = end_look_at_;

  camera_placement_pub_.publish(cp);
}

/*bool hasConfiguration() const
{
  return true;
}

void triggerConfiguration()
{
  // Usually used to open a dialog to offer the user a set of configuration
}*/

} // namespace
PLUGINLIB_EXPORT_CLASS(pose_interpolator::PoseInterpolator, rqt_gui_cpp::Plugin)