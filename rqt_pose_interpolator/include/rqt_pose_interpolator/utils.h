/** @file
 *
 * Helper functions for plugins.
 *
 * @author Jan Razlaw
 */

#ifndef RQT_POSE_INTERPOLATOR_UTILS_H
#define RQT_POSE_INTERPOLATOR_UTILS_H

#include <ros/node_handle.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerControl.h>

namespace pose_interpolator
{

/**
 * @breif Creates box marker.
 *
 * @param[in] scale     scale.
 * @return Box marker.
 */
inline visualization_msgs::Marker makeBox(float scale)
{
  visualization_msgs::Marker marker;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.pose.orientation.w = 1.;
  marker.pose.orientation.y = 1.;
  marker.scale.x = scale * 0.15;
  marker.scale.y = scale * 0.45;
  marker.scale.z = scale * 0.25;
  marker.color.r = 0.f;
  marker.color.g = 0.f;
  marker.color.b = 0.f;
  marker.color.a = 1.0;
  return marker;
}

/**
 * @breif Creates arrow marker.
 *
 * @param[in] scale     scale.
 * @return Arrow marker.
 */
inline visualization_msgs::Marker makeArrow(float scale)
{
  visualization_msgs::Marker marker;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.pose.orientation.w = 1.;
  marker.pose.orientation.y = 1.;
  marker.scale.x = scale * 0.7;
  marker.scale.y = scale * 0.1;
  marker.scale.z = scale * 0.1;
  marker.color.r = 1.f;
  marker.color.g = 1.f;
  marker.color.b = 1.f;
  marker.color.a = 0.6;
  return marker;
}

/**
 * @breif Augments marker with control.
 *
 * @param[in,out] marker    marker that is augmented with control.
 */
inline void makeBoxControl(visualization_msgs::InteractiveMarker& marker)
{
  visualization_msgs::InteractiveMarkerControl control;
  control.always_visible = true;
  control.markers.push_back(makeBox(marker.scale));
  control.markers.push_back(makeArrow(marker.scale));
  marker.controls.push_back(control);
  marker.controls.back().interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
  marker.controls.back().name = "submit_button";
}

inline bool getFullParamName(const ros::NodeHandle& nh,
                             std::string& param_name)
{
  std::vector<std::string> keys;
  nh.getParamNames(keys);
  for(auto& key : keys)
  {
    if (key.find(param_name) != std::string::npos) {
      param_name = key;
      return true;
    }
  }
  return false;
}

template<typename T> inline bool getParam(const ros::NodeHandle& nh,
                     std::string& param_name,
                     T& param,
                     T default_param)
{
  getFullParamName(nh, param_name);
  return nh.param< T >(param_name, param, default_param);
}

}

#endif //RQT_POSE_INTERPOLATOR_UTILS_H