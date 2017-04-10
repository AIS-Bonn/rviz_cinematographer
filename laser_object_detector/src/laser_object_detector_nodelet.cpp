/** @file

    This ROS nodelet detects objects of a specific size in laser point clouds

*/

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include "detector.h"

namespace laser_object_detector
{
  class LaserObjectDetectorNodelet: public nodelet::Nodelet
  {
  public:

    LaserObjectDetectorNodelet() {}
    ~LaserObjectDetectorNodelet() {}

  private:

    virtual void onInit();
    boost::shared_ptr<Detector> detector_;
  };

  /** @brief Nodelet initialization. */
  void LaserObjectDetectorNodelet::onInit()
  {
    detector_.reset(new Detector(getNodeHandle(), getPrivateNodeHandle()));
  }

} // namespace laser_object_detector


// Register this plugin with pluginlib.  Names must match nodelet_plugin.xml.
//
// parameters: package, class name, class type, base class type
PLUGINLIB_DECLARE_CLASS(laser_object_detector, LaserObjectDetectorNodelet,
                        laser_object_detector::LaserObjectDetectorNodelet, nodelet::Nodelet);
