/** @file

    This ROS nodelet detects objects of a specific size in velodyne laser point clouds

*/

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include "detector.h"

namespace velodyne_object_detector
{
  class VelodyneObjectDetectorNodelet: public nodelet::Nodelet
  {
  public:

     VelodyneObjectDetectorNodelet() {}
    ~VelodyneObjectDetectorNodelet() {}

  private:

    virtual void onInit();
    boost::shared_ptr<Detector> detector_;
  };

  /** @brief Nodelet initialization. */
  void VelodyneObjectDetectorNodelet::onInit()
  {
    detector_.reset(new Detector(getNodeHandle(), getPrivateNodeHandle()));
  }

} // namespace velodyne_object_detector


// Register this plugin with pluginlib.  Names must match nodelet_velodyne.xml.
//
// parameters: package, class name, class type, base class type
PLUGINLIB_DECLARE_CLASS(velodyne_object_detector, VelodyneObjectDetectorNodelet,
                        velodyne_object_detector::VelodyneObjectDetectorNodelet, nodelet::Nodelet);
