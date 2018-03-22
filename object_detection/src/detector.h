// Detect potential objects in point clouds
// Author: Jan Razlaw <s6jarazl@uni-bonn.de>

#ifndef PANEL_FILTER_H
#define PANEL_FILTER_H

#include <laser_segmentation/point_type.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <tf/transform_listener.h>

#include <config_server/parameter.h>

#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>

namespace object_detection
{

class Detector
{
public:
	typedef velodyne_pointcloud::PointXYZIdsSegment PointWithSegmentation;

	typedef PointWithSegmentation 				InputPoint;
	typedef pcl::PointCloud<InputPoint> 	InputPointCloud;

	Detector(ros::NodeHandle node, ros::NodeHandle private_nh);
	virtual ~Detector();

private:
	tf::TransformListener m_tf;

	ros::Subscriber m_sub_cloud;
  ros::Publisher m_pub_pose;
  ros::Publisher m_pub_vis_marker;

  config_server::Parameter<float> m_min_certainty_thresh;

  config_server::Parameter<float> m_cluster_tolerance;
  config_server::Parameter<int> m_min_cluster_size;
  config_server::Parameter<int> m_max_cluster_size;

	config_server::Parameter<float> m_max_object_height;
	config_server::Parameter<float> m_max_object_width;
	config_server::Parameter<float> m_max_object_altitude;

  std::string m_fixed_frame;
  std::string m_input_topic;

	bool transformCloud(InputPointCloud::Ptr& cloud, std::string target_frame);
	bool euclideanClustering(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
													 std::vector<pcl::PointIndices>& cluster_indices,
													 float cluster_tolerance,
													 int min_cluster_size,
													 int max_cluster_size);
	void getClusterProperties(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
														const pcl::PointIndices& point_indices,
														Eigen::Vector3f& mean,
														Eigen::Array3f& min,
														Eigen::Array3f& max);

	void handleCloud(const InputPointCloud::ConstPtr& segmentation);
};

}

#endif
