//
// Created by razlaw on 19.04.18.
//

#ifndef RQT_POSE_INTERPOLATOR_POSE_INTERPOLATOR_H
#define RQT_POSE_INTERPOLATOR_POSE_INTERPOLATOR_H

#include <ros/ros.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>

#include <geometry_msgs/Pose.h>
#include <view_controller_msgs/CameraPlacement.h>

#include <rqt_gui_cpp/plugin.h>
#include <QWidget>
#include "ui_pose_interpolator.h"

namespace pose_interpolator {

class PoseInterpolator
        : public rqt_gui_cpp::Plugin
{
  Q_OBJECT
public:
  PoseInterpolator();
  virtual void initPlugin(qt_gui_cpp::PluginContext& context);
  virtual void shutdownPlugin();
  virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
  virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);

  // Comment in to signal that the plugin has a way to configure it
  //bool hasConfiguration() const;
  //void triggerConfiguration();

  void camPoseCallback(const geometry_msgs::Pose::ConstPtr& cam_pose);
Q_SIGNALS:
  void updateRequested();

public slots:
  void moveCamToStart();
  void moveCamToEnd();
  void setStartToCurrentCam();

private:
  Ui::pose_interpolator ui_;
  QWidget* widget_;

  ros::Publisher camera_placement_pub_;
  ros::Subscriber camera_pose_sub_;

  geometry_msgs::Pose cam_pose_;

  geometry_msgs::Point start_look_at_;
  geometry_msgs::Point end_look_at_;
};
} // namespace

#endif //RQT_POSE_INTERPOLATOR_POSE_INTERPOLATOR_H
