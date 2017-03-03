/** \file

    This ROS node clusters potential objects in point clouds

*/

#include <ros/ros.h>
#include "clusterer.h"

/** Main node entry point. */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "object_clustering_node");
  ros::NodeHandle node;
  ros::NodeHandle priv_nh("~");

  // create conversion class, which subscribes to raw data
  object_clustering::Clusterer clusterer(node, priv_nh);

  // handle callbacks until shut down
  ros::spin();

  return 0;
}
