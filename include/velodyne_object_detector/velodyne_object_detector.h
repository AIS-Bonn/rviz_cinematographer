//
// Created by razlaw on 11/10/16.
//

#ifndef VELODYNE_OBJECT_DETECTOR_H
#define VELODYNE_OBJECT_DETECTOR_H

#include <pcl_ros/point_cloud.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "ros/ros.h"
#include <ros/package.h>

#include "std_msgs/String.h"
#include "std_msgs/Float32.h"

#include <pcl_ros/point_cloud.h>
#include <pcl/common/distances.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h>
#include <pcl/conversions.h>

#include <boost/circular_buffer.hpp>

#include <velodyne_object_detector/point_type.h>

#include <visualization_msgs/Marker.h>

#include <config_server/parameter.h>

namespace velodyne_object_detector
{

class VelodyneObjectDetector
{
public:
   typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
   typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;
   typedef pcl::PointCloud<pcl::PointXYZI> PointCloudIntensity;
   
   typedef velodyne_pointcloud::PointXYZIRDetection PointVelodyne;
   typedef pcl::PointCloud<PointVelodyne> PointCloudVelodyne;
   
   VelodyneObjectDetector();
   virtual ~VelodyneObjectDetector(){};

   void nop(){ROS_INFO_STREAM("new threshold");};

   void splitCloudByRing(PointCloudVelodyne &cloud, std::vector<std::vector<unsigned int> > &clouds_per_ring);

//    void detectSegments(PointCloudVelodyne &cloud,
//                        std::vector<std::vector<unsigned int> > &clouds_per_ring,
//                        std::vector<std::vector<std::pair<unsigned int, unsigned int> > > &segment_indices_cloud);

   void medianFilter(std::vector<float> &input,
                     std::vector<float> &filtered_output,
                     int kernel_size);

   void detectSegmentsMedian(PointCloudVelodyne &cloud,
                       std::vector<std::vector<unsigned int> > &clouds_per_ring,
                       std::vector<std::vector<std::pair<unsigned int, unsigned int> > > &segment_indices_cloud);

   void filterSegmentsBySize(PointCloudVelodyne &cloud,
                             std::vector<std::vector<unsigned int> > &clouds_per_ring,
                             std::vector<std::vector<std::pair<unsigned int, unsigned int> > > &segment_indices_cloud,
                             float size_filter);

   void velodyneCallback(const PointCloudVelodyne& input_cloud);

//    void detectObstacles(pcl::PointCloud<velodyne_pointcloud::PointXYZIR> &cloud,
//                         std::vector<velodyne_pointcloud::PointXYZIR> &currentObstaclesList,
//                         pcl::PointCloud<velodyne_pointcloud::PointXYZIR> &modifiedCloud,
//                         std::map<uint16_t, std::vector<double> > &distanceByPrevious);
private:
   ros::NodeHandle m_nh;
   //tf::TransformListener m_tf;

   const int PUCK_NUM_RINGS;

   ros::Subscriber m_velodyne_sub;
   ros::Publisher m_pub;
   ros::Publisher m_pub_median;
   ros::Publisher m_pub_cluster_marker;

   float m_cluster_distance_threshold;
   float m_max_internal_distance;
   float m_cluster_radius_threshold;
   int m_init_cluster_size;
   float m_min_cluster_radius;
   float m_max_cluster_radius;

   std::string m_points_topic;

   config_server::Parameter<float> m_certainty_threshold;
   config_server::Parameter<float> m_dist_coeff;
   config_server::Parameter<float> m_intensity_coeff;
   config_server::Parameter<float> m_max_box_width;
   config_server::Parameter<int> m_min_box_intensity;
   config_server::Parameter<int> m_max_box_intensity;
   config_server::Parameter<int> m_min_box_intensity_diff;
   config_server::Parameter<int> m_intensity_diff_cap;

   config_server::Parameter<float> m_median_min_dist;
   config_server::Parameter<float> m_median_thresh1_dist;
   config_server::Parameter<float> m_median_thresh2_dist;
   config_server::Parameter<float> m_median_max_dist;

};

}

#endif
