//
// Created by razlaw on 24.04.18.
//

#include "rqt_pose_interpolator/pose_interpolator.h"
#include "rqt_pose_interpolator/trajectory_editor.h"

#include "pluginlib/class_list_macros.h"

PLUGINLIB_EXPORT_CLASS(pose_interpolator::PoseInterpolator, rqt_gui_cpp::Plugin)
PLUGINLIB_EXPORT_CLASS(pose_interpolator::TrajectoryEditor, rqt_gui_cpp::Plugin)