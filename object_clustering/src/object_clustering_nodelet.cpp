/** @file

    This ROS nodelet clusters potential objects in point clouds

*/

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include "clusterer.h"

namespace object_clustering
{
  class ObjectClusteringNodelet: public nodelet::Nodelet
  {
  public:

     ObjectClusteringNodelet() {}
    ~ObjectClusteringNodelet() {}

  private:

    virtual void onInit();
    boost::shared_ptr<Clusterer> clusterer_;
  };

  /** @brief Nodelet initialization. */
  void ObjectClusteringNodelet::onInit()
  {
    clusterer_.reset(new Clusterer(getNodeHandle(), getPrivateNodeHandle()));
  }

} // namespace object_clustering


// Register this plugin with pluginlib.  Names must match nodelet_velodyne.xml.
//
// parameters: package, class name, class type, base class type
PLUGINLIB_DECLARE_CLASS(object_clustering, ObjectClusteringNodelet,
                        object_clustering::ObjectClusteringNodelet, nodelet::Nodelet);
