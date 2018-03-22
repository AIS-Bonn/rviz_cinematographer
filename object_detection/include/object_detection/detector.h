/** @file
 *
 * This class detects potential objects in point clouds
 *
 * @author Jan Razlaw
 */

#ifndef DETECTOR_H
#define DETECTOR_H

#include <laser_segmentation/point_type.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <tf/transform_listener.h>

#include <config_server/parameter.h>

#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>

namespace object_detection
{

/**
 *  Class to detect objects in 3D point clouds.
 */

class Detector
{
public:

	typedef velodyne_pointcloud::PointXYZIdsSegment 	InputPoint;
	typedef pcl::PointCloud<InputPoint> 							InputPointCloud;

	/**
	 * @brief Constructor.
	 *
	 * Parameters are needed to use this class within a ros node or nodelet.
	 */
	Detector(ros::NodeHandle node, ros::NodeHandle private_nh);
	virtual ~Detector();

private:

	/**
	 * @brief Transforms the cloud into the target_frame.
	 *
	 * @param[in,out] cloud 				the cloud that is transformed.
	 * @param[in] 		target_frame 	the frame the cloud should be transformed to.
	 *
	 * @return true if transform was successful, false otherwise
	 */
	bool transformCloud(InputPointCloud::Ptr& cloud, std::string target_frame);

	/**
	 * @brief Performs euclidean clustering on the points in cloud.
	 *
	 * Each cluster is represented by point indices that are stored in the entries
	 * of the cluster_indices vector.
	 *
	 * @param[in] 	cloud 						input cloud.
	 * @param[out] 	cluster_indices 	vector of clusters - each cluster represented
	 * 																	by several point indices.
	 * @param[in] 	cluster_tolerance distance threshold for euclidean clustering.
	 * @param[in] 	min_cluster_size 	minimal number of points a cluster has to have.
	 * @param[in] 	max_cluster_size 	maximal number of points a cluster is allowed to have.
	 *
	 * @return true if clustering was successful, false otherwise
	 */
	bool euclideanClustering(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
													 std::vector<pcl::PointIndices>& cluster_indices,
													 float cluster_tolerance,
													 int min_cluster_size,
													 int max_cluster_size);

	/**
	 * @brief Computes the mean point and the size of a cluster.
	 *
	 * @param[in] 	cloud 				input cloud.
	 * @param[in] 	point_indices indices of the cluster's points.
	 * @param[out] 	mean 					mean point of the cluster's points.
	 * @param[out] 	min 					minimum in x, y and z direction of all points.
	 * @param[out] 	max 					maximum in x, y and z direction of all points.
	 */
	void getClusterProperties(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
														const pcl::PointIndices& point_indices,
														Eigen::Vector3f& mean,
														Eigen::Array3f& min,
														Eigen::Array3f& max);

	/**
	 * @brief Detect all objects in a cloud that fit the description.
	 *
	 * First filter out all background points according to segmentation value in each point.
	 * Then cluster the remaining points by distance.
	 * Then filter clusters by description.
	 *
	 * @param[in] 	segmentation 	input cloud with segmentation values for each point.
	 */
	void handleCloud(const InputPointCloud::ConstPtr& segmentation);

	tf::TransformListener m_tf;

	ros::Subscriber m_sub_cloud;
  ros::Publisher m_pub_pose;
  ros::Publisher m_pub_vis_marker;

  config_server::Parameter<float> m_min_certainty_thresh;

  config_server::Parameter<float> m_cluster_tolerance;
  config_server::Parameter<int> m_min_cluster_size;
  config_server::Parameter<int> m_max_cluster_size;

	config_server::Parameter<float> m_min_object_height;
	config_server::Parameter<float> m_max_object_height;
	config_server::Parameter<float> m_max_object_width;
	config_server::Parameter<float> m_max_object_altitude;

  std::string m_fixed_frame;
  std::string m_input_topic;
};

}

#endif
