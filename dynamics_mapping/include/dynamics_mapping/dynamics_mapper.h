/** @file
 *
 * This class stores and maintains a representation of the dynamics in the environment
 *
 * @author Jan Razlaw
 */

#ifndef PANEL_FILTER_H
#define PANEL_FILTER_H

#include <laser_segmentation/point_type.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <tf/transform_listener.h>

#include <config_server/parameter.h>

#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>

namespace dynamics_mapping
{

/**
 *  Class to store and maintain a representation of the dynamics in the environment.
 */

class DynamicsMapper
{
public:
	typedef velodyne_pointcloud::PointXYZSegmentation InputPoint;
	typedef pcl::PointCloud<InputPoint> InputPointCloud;

	/**
	 * @brief Constructor.
	 *
	 * Parameters are needed to use this class within a ros node or nodelet.
	 */
	DynamicsMapper(ros::NodeHandle node, ros::NodeHandle private_nh);
	virtual ~DynamicsMapper();

private:

	/**
	 * @brief TODO.
	 *
	 * @param[in,out] cloud 				the cloud that is transformed.
	 * @param[in] 		target_frame 	the frame the cloud should be transformed to.
	 *
	 * @return true if transform was successful, false otherwise
	 */
	void handleCloud(const InputPointCloud::ConstPtr& segmentation);

	tf::TransformListener m_tf;

	ros::Subscriber m_sub_cloud;
	ros::Publisher m_pub_filtered_cloud;
  ros::Publisher m_pub_pose;
  ros::Publisher m_pub_vis_marker;

	config_server::Parameter<float> m_max_object_height;
  config_server::Parameter<float> m_max_object_width;
  config_server::Parameter<float> m_max_object_altitude;
  config_server::Parameter<float> m_min_certainty_thresh;

  config_server::Parameter<float> m_cluster_tolerance;
  config_server::Parameter<int> m_min_cluster_size;
  config_server::Parameter<int> m_max_cluster_size;

  config_server::Parameter<bool> m_apply_geofencing;
  config_server::Parameter<float> m_geofencing_min_x;
  config_server::Parameter<float> m_geofencing_max_x;
  config_server::Parameter<float> m_geofencing_min_y;
  config_server::Parameter<float> m_geofencing_max_y;

  config_server::Parameter<bool> m_apply_radius_filter;
  config_server::Parameter<float> m_filter_radius;

  std::string m_fixed_frame;
  std::string m_input_topic;
};

}

#endif
