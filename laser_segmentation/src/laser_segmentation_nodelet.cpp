/** @file

    This ROS node segments objects of a specified width in laser point clouds

*/

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include "segmenter.h"

namespace laser_segmentation
{
  class LaserSegmenterNodelet: public nodelet::Nodelet
  {
  public:

    LaserSegmenterNodelet() {}
    ~LaserSegmenterNodelet() {}

  private:

    virtual void onInit();
    boost::shared_ptr<Segmenter> segmenter_;
  };

  /** @brief Nodelet initialization. */
  void LaserSegmenterNodelet::onInit()
  {
    segmenter_.reset(new Segmenter(getNodeHandle(), getPrivateNodeHandle()));
  }

} // namespace laser_segmentation


// Register this plugin with pluginlib.  Names must match nodelet_plugin.xml.
//
// parameters: package, class name, class type, base class type
PLUGINLIB_DECLARE_CLASS(laser_segmentation, LaserSegmenterNodelet,
                        laser_segmentation::LaserSegmenterNodelet, nodelet::Nodelet);
