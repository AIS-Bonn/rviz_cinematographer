/** @file
 *
 * This ROS node detects potential objects in point clouds
 *
 * @author Jan Razlaw
 */

#include <ros/ros.h>
#include <object_detection/detector.h>

/**
 * Main node entry point.
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "object_detection_node");
  ros::NodeHandle node;
  ros::NodeHandle priv_nh("~");

  object_detection::Detector detector(node, priv_nh);

  // handle callbacks until shut down
  ros::spin();

  return 0;
}
