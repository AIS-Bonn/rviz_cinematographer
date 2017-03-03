// Cluster potential objects in point clouds
// Author: Jan Razlaw <s6jarazl@uni-bonn.de>

#include "clusterer.h"

#include <pcl_conversions/pcl_conversions.h>

#include <tf_conversions/tf_eigen.h>

#include <eigen_conversions/eigen_msg.h>

#include <pcl/common/transforms.h>

#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/impl/extract_clusters.hpp>
#include <pcl/search/impl/kdtree.hpp>

#include <pcl_ros/point_cloud.h>

namespace object_clustering
{

Clusterer::Clusterer(ros::NodeHandle node, ros::NodeHandle private_nh)
 : m_max_object_height("/object_clustering/max_object_height", 0.01, 0.01, 4.0, 1.8)
   , m_max_object_width("/object_clustering/max_object_width", 0.01, 0.01, 4.0, 2.0)
   , m_max_object_altitude("/object_clustering/max_object_altitude", 0.01, 0.01, 4.0, 2.0)
   , m_min_certainty_thresh("/object_clustering/certainty_thresh", 0.0, 0.01, 1.0, 0.5)
   , m_cluster_tolerance("/object_clustering/cluster_tolerance_in_m", 0.0, 0.01, 2.0, 1.0)
   , m_min_cluster_size("/object_clustering/min_cluster_size", 0, 1, 50, 4)
   , m_max_cluster_size("/object_clustering/max_cluster_size", 0, 1, 50000, 25000)
   , m_geofencing_min_x("/object_clustering/geofencing_min_x", -100.0, 0.5, 100.0, -100.0)
   , m_geofencing_max_x("/object_clustering/geofencing_max_x", -100.0, 0.5, 100.0, 100.0)
   , m_geofencing_min_y("/object_clustering/geofencing_min_y", -100.0, 0.5, 100.0, -100.0)
   , m_geofencing_max_y("/object_clustering/geofencing_max_y", -100.0, 0.5, 100.0, 100.0)
   , m_fixed_frame("world")
   , m_input_topic("/velodyne_detector_objects")
{
	ROS_INFO("init object clusterer...");

   float max_object_height;
   if(private_nh.getParam("max_object_height", max_object_height))
      m_max_object_height.set(max_object_height);

   float max_object_width;
   if(private_nh.getParam("max_object_width", max_object_width))
      m_max_object_width.set(max_object_width);

   float max_object_altitude;
   if(private_nh.getParam("max_object_altitude", max_object_altitude))
      m_max_object_altitude.set(max_object_altitude);

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

   float geofencing_min_x;
   if(private_nh.getParam("geofencing_min_x", geofencing_min_x))
      m_geofencing_min_x.set(geofencing_min_x);

   float geofencing_max_x;
   if(private_nh.getParam("geofencing_max_x", geofencing_max_x))
      m_geofencing_max_x.set(geofencing_max_x);

   float geofencing_min_y;
   if(private_nh.getParam("geofencing_min_y", geofencing_min_y))
      m_geofencing_min_y.set(geofencing_min_y);

   float geofencing_max_y;
   if(private_nh.getParam("geofencing_max_y", geofencing_max_y))
      m_geofencing_max_y.set(geofencing_max_y);

   private_nh.getParam("fixed_frame", m_fixed_frame);
   private_nh.getParam("input_topic", m_input_topic);

	m_sub_cloud = node.subscribe(m_input_topic, 1, &Clusterer::handleCloud, this);
   m_pub_geofenced_cloud = node.advertise<InputPointCloud>("geofenced_cloud", 1);
	m_pub_pose = node.advertise<geometry_msgs::PoseArray>("object_poses", 1);
   m_pub_vis_marker = node.advertise<visualization_msgs::MarkerArray>("object_cluster_markers", 1 );

}

Clusterer::~Clusterer()
{
}


void Clusterer::handleCloud(const InputPointCloud::ConstPtr& detection)
{
   if(detection->size() == 0)
      return;

	ros::Time stamp = pcl_conversions::fromPCL(detection->header.stamp);

	Eigen::Affine3f transform;
	{
		if(!m_tf.waitForTransform(m_fixed_frame, detection->header.frame_id, stamp, ros::Duration(2.0)))
		{
			ROS_ERROR_STREAM("Could not wait for transform to frame " << m_fixed_frame);
			return;
		}

		tf::StampedTransform transformTF;
		try
		{
			m_tf.lookupTransform(m_fixed_frame, detection->header.frame_id, stamp, transformTF);
		}
		catch(tf::TransformException& e)
		{
			ROS_ERROR("Could not lookup transform to frame: '%s'", e.what());
			return;
		}

		Eigen::Affine3d transform_double;
		tf::transformTFToEigen(transformTF, transform_double);
		transform = transform_double.cast<float>();
	}

	InputPointCloud::Ptr detection_transformed(new InputPointCloud);
	pcl::transformPointCloud(*detection, *detection_transformed, transform);

	InputPointCloud::Ptr positives(new InputPointCloud);
	positives->header = detection->header;
	positives->header.frame_id = m_fixed_frame;

   InputPointCloud::Ptr fenced(new InputPointCloud);
   fenced->header = detection->header;
   fenced->header.frame_id = m_fixed_frame;

   for(const auto& point : *detection_transformed)
   {
      // Field filter
      if(point.x > m_geofencing_max_x() || point.x < m_geofencing_min_x() ||
         point.y > m_geofencing_max_y() || point.y < m_geofencing_min_y())
         continue;

      fenced->push_back(point);
   }

   m_pub_geofenced_cloud.publish(fenced);

	for(const auto& point : *fenced)
	{
		// Detection filter
		if(point.detection < m_min_certainty_thresh())
			continue;

		positives->push_back(point);
	}
	ROS_DEBUG("positives: %lu", positives->size());
   
	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<PointWithDetection> ec;
	ec.setClusterTolerance (m_cluster_tolerance()); // in m
	ec.setMinClusterSize (m_min_cluster_size());
	ec.setMaxClusterSize (m_max_cluster_size());
	ec.setInputCloud(positives);
	ec.extract(cluster_indices);

   ROS_DEBUG_STREAM("Number of found clusters is " << cluster_indices.size() << ". ");

   visualization_msgs::MarkerArray marker_array;

   geometry_msgs::PoseArray pose_msgs;
   pose_msgs.header.frame_id = positives->header.frame_id;
   pose_msgs.header.stamp = stamp;

   for(std::size_t i = 0; i < cluster_indices.size(); ++i)
	{
		Eigen::Vector3f mean = Eigen::Vector3f::Zero();
		Eigen::Array3f min = Eigen::Array3f::Constant(std::numeric_limits<float>::max());
		Eigen::Array3f max = Eigen::Array3f::Constant(-std::numeric_limits<float>::max());

		for(auto idx : cluster_indices[i].indices)
		{
			auto pos = (*positives)[idx].getVector3fMap();
			mean += pos;

			for(std::size_t i = 0; i < 3; ++i)
			{
				min[i] = std::min(min[i], pos[i]);
				max[i] = std::max(max[i], pos[i]);
			}
		}

		Eigen::Vector3f size = max.matrix() - min.matrix();

      if(min.z() > m_max_object_altitude())
         continue;

		if(size.z() > m_max_object_height())
			continue;

		if(size.x() > m_max_object_width() || size.y() > m_max_object_width())
			continue;

		mean /= cluster_indices[i].indices.size();

      visualization_msgs::Marker marker;
      marker.header.frame_id = positives->header.frame_id;
      marker.header.stamp = stamp;
      marker.ns = "object_clusterer_namespace";
      marker.id = i;
      marker.type = visualization_msgs::Marker::CYLINDER;
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
      marker.color.g = 0.0;
      marker.color.b = 0.0;
      marker.lifetime = ros::Duration(0, 200000000);

      marker_array.markers.push_back(marker);

      geometry_msgs::Pose pose_msg;
      Eigen::Affine3d pose;
      pose = Eigen::Translation3d(mean.x(), mean.y(), mean.z());
      tf::poseEigenToMsg(pose, pose_msg);

      pose_msgs.poses.push_back(pose_msg);
	}

   ROS_DEBUG_STREAM("Number of found clusters after filtering " << marker_array.markers.size() << ". ");

   m_pub_vis_marker.publish(marker_array);
   m_pub_pose.publish(pose_msgs);
}

}
