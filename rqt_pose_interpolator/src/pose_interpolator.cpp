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

  // give QObjects reasonable names
  setObjectName("PoseInterpolator");
}

void PoseInterpolator::initPlugin(qt_gui_cpp::PluginContext& context)
{
  ros::NodeHandle ph("~");

  camera_placement_pub_ = ph.advertise<view_controller_msgs::CameraPlacement>("/rviz/camera_placement", 1);

  // access standalone command line arguments
  QStringList argv = context.argv();
  // create QWidget
  widget_ = new QWidget();
  // extend the widget with all attributes and children from UI file
  ui_.setupUi(widget_);

  connect(ui_.set_camera_button, SIGNAL(clicked(bool)), this, SLOT(setCamera()));

  // add widget to the user interface
  context.addWidget(widget_);
}

void PoseInterpolator::shutdownPlugin()
{
  // TODO unregister all publishers here
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

void PoseInterpolator::setCamera()
{
  view_controller_msgs::CameraPlacement cp;
  cp.eye.header.stamp = ros::Time::now();
  cp.eye.header.frame_id = "base_link";
  cp.target_frame = "base_link";
  cp.interpolation_mode = view_controller_msgs::CameraPlacement::LINEAR; // SPHERICAL
  cp.time_from_start = ros::Duration(ui_.transition_time_spin_box->value());

  cp.up.header = cp.focus.header = cp.eye.header;
  cp.up.vector.x = 0.0;
  cp.up.vector.y = 0.0;
  cp.up.vector.z = 1.0;
  geometry_msgs::Point look_from;
  look_from.x = 100;
  look_from.y = 0;
  look_from.z = 0;
  cp.eye.point = look_from;

  geometry_msgs::Point look_at;
  look_at.x = 0;
  look_at.y = 0;
  look_at.z = 0;
  cp.focus.point = look_at;

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