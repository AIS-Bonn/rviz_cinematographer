/** @file
 *
 * This class detects potential objects in point clouds
 *
 * @author Jan Razlaw
 */

#include <object_detection/detector.h>

#include <pcl_conversions/pcl_conversions.h>

#include <tf_conversions/tf_eigen.h>

#include <eigen_conversions/eigen_msg.h>

#include <pcl/common/transforms.h>

#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/impl/extract_clusters.hpp>
#include <pcl/search/impl/kdtree.hpp>

#include <pcl_ros/point_cloud.h>

namespace object_detection
{

Detector::Detector(ros::NodeHandle node, ros::NodeHandle private_nh)
 : m_min_certainty_thresh("/object_detection/certainty_thresh", 0.0, 0.01, 1.0, 0.5)
   , m_cluster_tolerance("/object_detection/cluster_tolerance_in_m", 0.0, 0.01, 2.0, 1.0)
   , m_min_cluster_size("/object_detection/min_cluster_size", 0, 1, 50, 4)
   , m_max_cluster_size("/object_detection/max_cluster_size", 0, 1, 50000, 25000)
   , m_min_object_height("/object_detection/min_object_height", 0.01, 0.01, 4.0, 0.5)
   , m_max_object_height("/object_detection/max_object_height", 0.01, 0.01, 4.0, 1.8)
   , m_max_object_width("/object_detection/max_object_width", 0.01, 0.01, 4.0, 2.0)
   , m_max_object_altitude("/object_detection/max_object_altitude", 0.01, 0.01, 4.0, 2.0)
   , m_fixed_frame("world")
   , m_input_topic("/laser_segmenter_objects")
{
	ROS_INFO("Object_detector: Init...");
  
  float certainty_thresh;
  if(private_nh.getParam("certainty_thresh", certainty_thresh))
    m_min_certainty_thresh.set(certainty_thresh);
  
  float cluster_tolerance;
  if(private_nh.getParam("cluster_tolerance", cluster_tolerance))
    m_cluster_tolerance.set(cluster_tolerance);
  
  float min_cluster_size;
  if(private_nh.getParam("min_cluster_size", min_cluster_size))
    m_min_cluster_size.set(min_cluster_size);
  
  float max_cluster_size;
  if(private_nh.getParam("max_cluster_size", max_cluster_size))
    m_max_cluster_size.set(max_cluster_size);

  float min_object_height;
  if(private_nh.getParam("min_object_height", min_object_height))
    m_min_object_height.set(min_object_height);

  float max_object_height;
  if(private_nh.getParam("max_object_height", max_object_height))
    m_max_object_height.set(max_object_height);

  float max_object_width;
  if(private_nh.getParam("max_object_width", max_object_width))
    m_max_object_width.set(max_object_width);

  float max_object_altitude;
  if(private_nh.getParam("max_object_altitude", max_object_altitude))
    m_max_object_altitude.set(max_object_altitude);

  private_nh.getParam("fixed_frame", m_fixed_frame);
  private_nh.getParam("input_topic", m_input_topic);

	m_sub_cloud = node.subscribe(m_input_topic, 1, &Detector::handleCloud, this);
	m_pub_pose = node.advertise<geometry_msgs::PoseArray>("object_detection_poses", 1);
  m_pub_vis_marker = node.advertise<visualization_msgs::MarkerArray>("object_detection_markers", 1);
}

Detector::~Detector()
{
}

bool Detector::transformCloud(InputPointCloud::Ptr& cloud, std::string target_frame)
{
  ros::Time stamp = pcl_conversions::fromPCL(cloud->header.stamp);

  Eigen::Affine3f transform;
  {
    if(!m_tf.waitForTransform(target_frame, cloud->header.frame_id, stamp, ros::Duration(0.5)))
    {
      ROS_ERROR_STREAM("Object_detection: Could not wait for transform from cloud frame to frame " << target_frame);
      return false;
    }

    tf::StampedTransform transformTF;
    try
    {
      m_tf.lookupTransform(target_frame, cloud->header.frame_id, stamp, transformTF);
    }
    catch(tf::TransformException& e)
    {
      ROS_ERROR("Object_detection: Could not lookup transform to frame: '%s'", e.what());
      return false;
    }

    Eigen::Affine3d transform_double;
    tf::transformTFToEigen(transformTF, transform_double);
    transform = transform_double.cast<float>();
  }

  InputPointCloud::Ptr cloud_transformed(new InputPointCloud);
  pcl::transformPointCloud(*cloud, *cloud_transformed, transform);
  cloud_transformed->header = cloud->header;
  cloud_transformed->header.frame_id = target_frame;

  cloud.swap(cloud_transformed);

  return true;
}

bool Detector::euclideanClustering(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                   std::vector<pcl::PointIndices>& cluster_indices,
                                   float cluster_tolerance,
                                   int min_cluster_size,
                                   int max_cluster_size)
{
  try
  {
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);

    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(cluster_tolerance); // in m
    ec.setMinClusterSize(min_cluster_size);
    ec.setMaxClusterSize(max_cluster_size);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);
  }
  catch(...)
  {
    ROS_ERROR("Object_detection: Clustering failed.");
    return false;
  }

  ROS_DEBUG_STREAM("Object_detector: Number of found clusters is " << cluster_indices.size() << ". ");
  return true;
}

void Detector::getClusterProperties(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                    const pcl::PointIndices& point_indices,
                                    Eigen::Vector3f& mean,
                                    Eigen::Array3f& min,
                                    Eigen::Array3f& max)
{
  mean = Eigen::Vector3f::Zero();
  min = Eigen::Array3f::Constant(std::numeric_limits<float>::max());
  max = Eigen::Array3f::Constant(-std::numeric_limits<float>::max());

  for(auto idx : point_indices.indices)
  {
    auto pos = (*cloud)[idx].getVector3fMap();
    mean += pos;

    for(std::size_t i = 0; i < 3; ++i)
    {
      min[i] = std::min(min[i], pos[i]);
      max[i] = std::max(max[i], pos[i]);
    }
  }

  mean /= point_indices.indices.size();
}

void Detector::handleCloud(const InputPointCloud::ConstPtr& input_cloud)
{
  // start when subscribers are available
  if(m_pub_pose.getNumSubscribers() == 0 &&
     m_pub_vis_marker.getNumSubscribers() == 0)
  {
    ROS_DEBUG_STREAM("Object_detector: No subscriber.");
    return;
  }
  
  if(input_cloud->size() == 0)
    return;

  pcl::StopWatch timer;

  // filter out background points
  InputPointCloud::Ptr segments_cloud(new InputPointCloud);
  segments_cloud->header = input_cloud->header;
  
  for(const auto& point : *input_cloud)
  {
    if(point.segment < m_min_certainty_thresh())
      continue;

    segments_cloud->push_back(point);
  }
  ROS_DEBUG("Object_detector: segments_cloud size is %lu ", segments_cloud->size());

  if(segments_cloud->size() == 0)
  {
    ROS_DEBUG("Object_detection: No positive segmentation points left for detection");
    return;
  }

  // transform cloud to fixed frame
  if(!transformCloud(segments_cloud, m_fixed_frame))
    return;

  // workaround for usage with nodelet
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::copyPointCloud(*segments_cloud, *cloud_xyz);
  
  if(segments_cloud->size() != cloud_xyz->size())
    ROS_WARN_STREAM("Object_detection: cloud sizes do not match after copy.");

  // Cluster segments by distance
  std::vector<pcl::PointIndices> cluster_indices;
  if(!euclideanClustering(cloud_xyz, cluster_indices, m_cluster_tolerance(), m_min_cluster_size(), m_max_cluster_size()))
    return;

  visualization_msgs::MarkerArray marker_array;

  geometry_msgs::PoseArray pose_msgs;
  pose_msgs.header.frame_id = segments_cloud->header.frame_id;
  pose_msgs.header.stamp = pcl_conversions::fromPCL(segments_cloud->header.stamp);

  // filter clusters and publish as marker array
  for(std::size_t i = 0; i < cluster_indices.size(); ++i)
	{
		Eigen::Vector3f mean;
		Eigen::Array3f min, max;
    getClusterProperties(cloud_xyz, cluster_indices[i], mean, min, max);

		Eigen::Vector3f size = max.matrix() - min.matrix();

    if(min.z() > m_max_object_altitude())
      continue;

		if(size.z() < m_min_object_height())
			continue;

    if(size.z() > m_max_object_height())
      continue;

    float object_width = sqrt(pow(size.x(), 2) + pow(size.y(), 2));
		if(object_width > m_max_object_width())
			continue;

    visualization_msgs::Marker marker;
    marker.header.frame_id = segments_cloud->header.frame_id;
    marker.header.stamp = pcl_conversions::fromPCL(segments_cloud->header.stamp);
    marker.ns = "object_detector_namespace";
    marker.id = i;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = mean.x();
    marker.pose.position.y = mean.y();
    marker.pose.position.z = mean.z();
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = std::max(static_cast<float>(size.x()), 0.2f);
    marker.scale.y = std::max(static_cast<float>(size.y()), 0.2f);
    marker.scale.z = std::max(static_cast<float>(size.z()), 0.2f);
    marker.color.a = 0.8;
    marker.color.r = 1.0;
    marker.color.g = 0.5;
    marker.color.b = 0.0;
    marker.lifetime = ros::Duration(0, 200000000);

    marker_array.markers.push_back(marker);

    geometry_msgs::Pose pose_msg;
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d(mean.x(), mean.y(), mean.z());
    tf::poseEigenToMsg(pose, pose_msg);

    pose_msgs.poses.push_back(pose_msg);
	}
  
  ROS_DEBUG_STREAM("Object_detector: Number of detections after filtering " << marker_array.markers.size() << ". ");
  
  m_pub_vis_marker.publish(marker_array);
  m_pub_pose.publish(pose_msgs);

  ROS_DEBUG_STREAM("Object_detector: time for one cloud in ms : " << timer.getTime());
}

}
