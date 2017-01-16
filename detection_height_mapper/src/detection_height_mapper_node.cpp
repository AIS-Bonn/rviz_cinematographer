/** \file

    This ROS node converts a 3D point cloud with height and detection information into a 2D height map

*/

#include <ros/ros.h>
#include "mapper.h"

/** Main node entry point. */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "detection_height_mapper_node");
  ros::NodeHandle node;
  ros::NodeHandle priv_nh("~");

  // create conversion class, which subscribes to raw data
  detection_height_mapper::Mapper mapper(node, priv_nh);

  // handle callbacks until shut down
  ros::spin();

  return 0;
}
