//
// Nodelet to detect small obstacles in velodyne point clouds 
//

#ifndef VELODYNE_OBJECT_DETECTOR_NODELET_H
#define VELODYNE_OBJECT_DETECTOR_NODELET_H

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h>
#include <pcl/conversions.h>

#include <boost/circular_buffer.hpp>
#include <boost/thread/mutex.hpp>

#include <velodyne_object_detector/point_type.h>

#include <config_server/parameter.h>


namespace velodyne_object_detector
{

class VelodyneObjectDetectorNodelet : public nodelet::Nodelet
{
public:
   typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
   typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;
   typedef pcl::PointCloud<pcl::PointXYZI> PointCloudIntensity;

   typedef velodyne_pointcloud::PointXYZIDR           PointVelodyneWithDist;
   typedef velodyne_pointcloud::PointXYZIRDetection   PointVelodyneWithDetection;
   typedef velodyne_pointcloud::PointXYZDetection     PointWithDetection;


   typedef PointVelodyneWithDist                      InputPoint;
   typedef PointVelodyneWithDetection                 DebugOutputPoint;
   typedef PointWithDetection                         OutputPoint;

   typedef pcl::PointCloud<InputPoint>                InputPointCloud;
   typedef pcl::PointCloud<DebugOutputPoint>          DebugOutputPointCloud;
   typedef pcl::PointCloud<OutputPoint>               OutputPointCloud;

   VelodyneObjectDetectorNodelet();
   virtual ~VelodyneObjectDetectorNodelet(){};
   virtual void onInit();

   void changeParameterSavely();
   void resizeBuffers();

   void velodyneCallback(const InputPointCloud::ConstPtr &input_cloud);

   void splitCloudByRing(const InputPointCloud::ConstPtr &cloud,
                         std::shared_ptr<std::vector<std::vector<unsigned int> > > clouds_per_ring);

   void filterRing(const InputPointCloud::ConstPtr &cloud,
                   const std::vector<unsigned int> &indices_of_ring,
                   int ring_index,
                   std::shared_ptr<std::vector<float> > distances_ring_filtered_small_kernel,
                   std::shared_ptr<std::vector<float> > distances_ring_filtered_big_kernel,
                   std::shared_ptr<std::vector<float> > intensities_ring_filtered_small_kernel,
                   std::shared_ptr<std::vector<float> > intensities_ring_filtered_big_kernel);

   float computeCertainty(float difference_distances, float difference_intensities);

   void detectObstacles(const InputPointCloud::ConstPtr &cloud,
                        const std::shared_ptr<std::vector<std::vector<unsigned int> > > clouds_per_ring);


private:
   const int PUCK_NUM_RINGS;

   ros::Subscriber m_velodyne_sub;
   ros::Publisher m_pub_obstacle_cloud;
   ros::Publisher m_pub_debug_obstacle_cloud;
   ros::Publisher m_pub_filtered_cloud;

   tf::TransformListener m_tf_listener;

   float m_max_prob_by_distance;
   float m_max_intensity_range;

   float m_certainty_threshold_launch;
   int m_median_small_kernel_size_launch;
   int m_median_big_kernel_size_launch;
   int m_distance_to_comparison_points_launch;

   config_server::Parameter<float> m_certainty_threshold;
   config_server::Parameter<float> m_dist_coeff;
   config_server::Parameter<float> m_intensity_coeff;
   config_server::Parameter<float> m_weight_for_small_intensities;

   config_server::Parameter<int> m_median_small_kernel_size;
   config_server::Parameter<int> m_median_big_kernel_size_parameter;
   int m_median_big_kernel_size;
   config_server::Parameter<int> m_distance_to_comparison_points;

   config_server::Parameter<float> m_median_min_dist;
   config_server::Parameter<float> m_median_thresh1_dist;
   config_server::Parameter<float> m_median_thresh2_dist;
   config_server::Parameter<float> m_median_max_dist;

   config_server::Parameter<float> m_max_dist_for_median_computation;

   std::string m_points_topic;

   bool m_publish_filtered_cloud;
   bool m_publish_debug_cloud;

   boost::mutex m_parameter_change_lock;
   std::vector<boost::circular_buffer<float> > m_distance_median_circ_buffer_vector;
   std::vector<boost::circular_buffer<float> > m_intensity_median_circ_buffer_vector;

   std::shared_ptr<std::vector<std::vector<unsigned int> > > m_clouds_per_ring;
   std::shared_ptr<std::vector<std::vector<unsigned int> > > m_old_clouds_per_ring;

   std::vector<std::shared_ptr<std::vector<float> > > m_old_distances_all_rings_filtered_small_kernel;
   std::vector<std::shared_ptr<std::vector<float> > > m_old_distances_all_rings_filtered_big_kernel;
   std::vector<std::shared_ptr<std::vector<float> > > m_old_intensities_all_rings_filtered_small_kernel;
   std::vector<std::shared_ptr<std::vector<float> > > m_old_intensities_all_rings_filtered_big_kernel;

   InputPointCloud::ConstPtr m_old_cloud;

   std::vector<int> m_ring_counter;
};

}
#endif //VELODYNE_OBJECT_DETECTOR_NODELET_H
