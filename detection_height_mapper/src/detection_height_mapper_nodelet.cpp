/** @file

    This ROS nodelet converts a 3D point cloud with height and detection information into a 2D height map

*/

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include "mapper.h"

namespace detection_height_mapper
{
  class DetectionHeightMapperNodelet: public nodelet::Nodelet
  {
  public:

    DetectionHeightMapperNodelet() {}
    ~DetectionHeightMapperNodelet() {}

  private:

    virtual void onInit();
    boost::shared_ptr<Mapper> mapper_;
  };

  /** @brief Nodelet initialization. */
  void DetectionHeightMapperNodelet::onInit()
  {
    mapper_.reset(new Mapper(getNodeHandle(), getPrivateNodeHandle()));
  }

} // namespace detection_height_mapper


// Register this plugin with pluginlib.  Names must match nodelet_velodyne.xml.
//
// parameters: package, class name, class type, base class type
PLUGINLIB_DECLARE_CLASS(detection_height_mapper, DetectionHeightMapperNodelet,
                        detection_height_mapper::DetectionHeightMapperNodelet, nodelet::Nodelet);
