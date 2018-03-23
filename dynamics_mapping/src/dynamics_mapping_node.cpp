/** @file
 *
 * This ROS node stores and maintains a representation of the dynamics in the environment
 *
 * @author Jan Razlaw
 */

#include <ros/ros.h>
#include <dynamics_mapping/dynamics_mapper.h>

/**
 * Main node entry point.
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "dynamics_mapping_node");
  ros::NodeHandle node;
  ros::NodeHandle priv_nh("~");

  dynamics_mapping::DynamicsMapper dynamics_mapper(node, priv_nh);

  // handle callbacks until shut down
  ros::spin();

  return 0;
}
