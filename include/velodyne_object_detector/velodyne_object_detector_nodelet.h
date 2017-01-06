//
// Created by razlaw on 1/4/17.
//

#ifndef VELODYNE_OBJECT_DETECTOR_NODELET_H
#define VELODYNE_OBJECT_DETECTOR_NODELET_H

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <pcl_ros/point_cloud.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

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

class VelodyneObjectDetectorNodelet : public nodelet::Nodelet
{
public:
   typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
   typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;
   typedef pcl::PointCloud<pcl::PointXYZI> PointCloudIntensity;

   typedef velodyne_pointcloud::PointXYZIDR PointVelodyneWithDist;
   typedef pcl::PointCloud<PointVelodyneWithDist> PointCloudVelodyneWithDist;

   typedef velodyne_pointcloud::PointXYZIRDetection PointVelodyneWithDetection;
   typedef pcl::PointCloud<PointVelodyneWithDetection> PointCloudVelodyneWithDetection;

   typedef PointVelodyneWithDist             InputPoint;
   typedef PointVelodyneWithDetection        OutputPoint;

   typedef pcl::PointCloud<InputPoint>       InputPointCloud;
   typedef pcl::PointCloud<OutputPoint>      OutputPointCloud;

   VelodyneObjectDetectorNodelet();
   virtual ~VelodyneObjectDetectorNodelet(){};
   virtual void onInit();

   void nop(){ROS_INFO_STREAM("new threshold");};

   void splitCloudByRing(InputPointCloud &cloud,
                         std::vector<std::vector<unsigned int> > &clouds_per_ring);

   void medianFilter(std::vector<float> &input,
                     std::vector<float> &filtered_output,
                     int kernel_size,
                     float max_distance_difference = 0.f);

   void detectSegmentsMedian(InputPointCloud &cloud,
                             std::vector<std::vector<unsigned int> > &clouds_per_ring);

   void velodyneCallback(const InputPointCloud& input_cloud);

private:
   const int PUCK_NUM_RINGS;

   ros::Subscriber m_velodyne_sub;
   ros::Publisher m_pub_obstacle_cloud;

   float m_max_prob_by_distance;
   float m_max_intensity_range;

   float m_certainty_threshold_launch;

   config_server::Parameter<float> m_certainty_threshold;
   config_server::Parameter<float> m_dist_coeff;
   config_server::Parameter<float> m_intensity_coeff;
   config_server::Parameter<float> m_weight_for_small_intensities;

   config_server::Parameter<float> m_median_min_dist;
   config_server::Parameter<float> m_median_thresh1_dist;
   config_server::Parameter<float> m_median_thresh2_dist;
   config_server::Parameter<float> m_median_max_dist;

   config_server::Parameter<float> m_max_dist_for_median_computation;

   std::string m_points_topic;
};

}
#endif //VELODYNE_OBJECT_DETECTOR_NODELET_H
