/** @file
 *
 * This class stores and maintains a representation of the dynamics in the environment
 *
 * @author Jan Razlaw
 */

#include <dynamics_mapping/dynamics_mapper.h>

#include <pcl_conversions/pcl_conversions.h>

#include <tf_conversions/tf_eigen.h>

#include <eigen_conversions/eigen_msg.h>

#include <pcl/common/transforms.h>

#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/impl/extract_clusters.hpp>
#include <pcl/search/impl/kdtree.hpp>

#include <pcl_ros/point_cloud.h>

namespace dynamics_mapping
{

DynamicsMapper::DynamicsMapper(ros::NodeHandle node, ros::NodeHandle private_nh)
 : m_max_object_height("/object_clustering/max_object_height", 0.01, 0.01, 4.0, 1.8)
   , m_max_object_width("/object_clustering/max_object_width", 0.01, 0.01, 4.0, 2.0)
   , m_max_object_altitude("/object_clustering/max_object_altitude", 0.01, 0.01, 4.0, 2.0)
   , m_min_certainty_thresh("/object_clustering/certainty_thresh", 0.0, 0.01, 1.0, 0.5)
   , m_cluster_tolerance("/object_clustering/cluster_tolerance_in_m", 0.0, 0.01, 2.0, 1.0)
   , m_min_cluster_size("/object_clustering/min_cluster_size", 0, 1, 50, 4)
   , m_max_cluster_size("/object_clustering/max_cluster_size", 0, 1, 50000, 25000)
   , m_apply_geofencing("/object_clustering/apply_geofencing", false)
   , m_geofencing_min_x("/object_clustering/geofencing_min_x", -200.0, 0.5, 200.0, -200.0)
   , m_geofencing_max_x("/object_clustering/geofencing_max_x", -200.0, 0.5, 200.0, 200.0)
   , m_geofencing_min_y("/object_clustering/geofencing_min_y", -200.0, 0.5, 200.0, -200.0)
   , m_geofencing_max_y("/object_clustering/geofencing_max_y", -200.0, 0.5, 200.0, 200.0)
   , m_apply_radius_filter("/object_clustering/apply_radius_filter", true)
   , m_filter_radius("/object_clustering/filter_radius", 0.0, 0.5, 200.0, 30.0)
   , m_fixed_frame("world")
   , m_input_topic("/laser_segmenter_objects")
{
	ROS_INFO("DynamicsMapper: Init...");

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

  bool apply_geofencing;
  if(private_nh.getParam("apply_geofencing", apply_geofencing))
    m_apply_geofencing.set(apply_geofencing);
  
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

  bool apply_radius_filter;
  if(private_nh.getParam("apply_radius_filter", apply_radius_filter))
    m_apply_radius_filter.set(apply_radius_filter);

  float filter_radius;
  if(private_nh.getParam("filter_radius", filter_radius))
    m_filter_radius.set(filter_radius);

  private_nh.getParam("fixed_frame", m_fixed_frame);
  private_nh.getParam("input_topic", m_input_topic);

	m_sub_cloud = node.subscribe(m_input_topic, 1, &DynamicsMapper::handleCloud, this);
  m_pub_filtered_cloud = node.advertise<InputPointCloud>("filtered_cloud", 1);
	m_pub_pose = node.advertise<geometry_msgs::PoseArray>("object_poses", 1);
  m_pub_vis_marker = node.advertise<visualization_msgs::MarkerArray>("object_cluster_markers", 1 );
}

DynamicsMapper::~DynamicsMapper()
{
}


void DynamicsMapper::handleCloud(const InputPointCloud::ConstPtr& segmentation)
{
  if(m_pub_filtered_cloud.getNumSubscribers() == 0 &&
     m_pub_pose.getNumSubscribers() == 0 &&
     m_pub_vis_marker.getNumSubscribers() == 0)
  {
    ROS_DEBUG_STREAM("DynamicsMapper::handleCloud: No subscriber to object_clusterer.");
    return;
  }
  
}

}
