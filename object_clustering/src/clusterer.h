// Cluster potential objects in point clouds
// Author: Jan Razlaw <s6jarazl@uni-bonn.de>

#ifndef PANEL_FILTER_H
#define PANEL_FILTER_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <velodyne_object_detector/point_type.h>

#include <tf/transform_listener.h>

#include <config_server/parameter.h>

#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>

namespace object_clustering
{

class Clusterer
{
public:
	typedef velodyne_pointcloud::PointXYZDetection PointWithDetection;
	typedef PointWithDetection InputPoint;
	typedef pcl::PointCloud<InputPoint> InputPointCloud;

	Clusterer(ros::NodeHandle node, ros::NodeHandle private_nh);
	virtual ~Clusterer();

private:
	tf::TransformListener m_tf;

	ros::Subscriber m_sub_cloud;
	ros::Publisher m_pub_geofenced_cloud;
   ros::Publisher m_pub_pose;
   ros::Publisher m_pub_vis_marker;

	config_server::Parameter<float> m_max_object_height;
   config_server::Parameter<float> m_max_object_width;
   config_server::Parameter<float> m_max_object_altitude;
   config_server::Parameter<float> m_min_certainty_thresh;
   config_server::Parameter<float> m_cluster_tolerance;
   config_server::Parameter<int> m_min_cluster_size;
   config_server::Parameter<int> m_max_cluster_size;
   config_server::Parameter<float> m_geofencing_min_x;
   config_server::Parameter<float> m_geofencing_max_x;
   config_server::Parameter<float> m_geofencing_min_y;
   config_server::Parameter<float> m_geofencing_max_y;

   std::string m_fixed_frame;
   std::string m_input_topic;

	void handleCloud(const InputPointCloud::ConstPtr& detection);
};

}

#endif
