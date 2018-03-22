/** \file
 *
 * This ROS node segments objects of a specified width in laser point clouds
 *
 * @author Jan Razlaw
 */

#include <ros/ros.h>
#include <laser_segmentation/segmenter.h>

/**
 * Main node entry point.
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "laser_segmentation_node");
  ros::NodeHandle node;
  ros::NodeHandle priv_nh("~");

  // create conversion class, which subscribes to raw data
  laser_segmentation::Segmenter segmenter(node, priv_nh);

  // handle callbacks until shut down
  ros::spin();

  return 0;
}
