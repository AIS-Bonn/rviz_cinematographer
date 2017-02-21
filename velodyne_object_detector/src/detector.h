/* -*- mode: C++ -*- */
/** @file

    This class detects objects of a specific size in velodyne laser point clouds

*/

#ifndef _DETECTOR_H_
#define _DETECTOR_H_ 1

#include <functional>

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
#include <pcl/visualization/histogram_visualizer.h>
#include <pcl/visualization/pcl_plotter.h>

#include <boost/circular_buffer.hpp>
#include <boost/thread/mutex.hpp>

#include <velodyne_object_detector/point_type.h>

#include <config_server/parameter.h>


namespace velodyne_object_detector
{
class Detector
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

   typedef typename boost::circular_buffer<InputPoint>::iterator buffer_iterator;
   typedef typename boost::circular_buffer<InputPoint>::const_iterator buffer_const_iterator;

   typedef boost::circular_buffer< InputPoint > BufferInputPoints;
   typedef std::shared_ptr<BufferInputPoints> BufferInputPointsPtr;

    struct MedianFiltered {
      MedianFiltered(InputPoint p):point(p){};
      MedianFiltered(){};
      InputPoint point;
      float dist_small_kernel;
      float dist_big_kernel;
      float intens_small_kernel;
      float intens_big_kernel;
    };	EIGEN_ALIGN16; 
    
   typedef boost::circular_buffer< MedianFiltered > BufferMedians;
   typedef std::shared_ptr<BufferMedians> BufferMediansPtr;
   typedef typename BufferMedians::iterator median_iterator;
   typedef typename BufferMedians::const_iterator median_const_iterator;
 

   Detector(ros::NodeHandle node, ros::NodeHandle private_nh);
   ~Detector(){};

   void changeParameterSavely();
   void resizeBuffers();

   void velodyneCallback(const InputPointCloud::ConstPtr &input_cloud);

   void filterRing(std::shared_ptr<boost::circular_buffer<MedianFiltered> > buffer_median_filtered,
			  median_iterator& iter
 			);

   float computeCertainty(float difference_distances, float difference_intensities);

   void detectObstacles(std::shared_ptr<boost::circular_buffer<MedianFiltered> > buffer_median_filtered,
			               median_iterator& current_element,
                        OutputPointCloud::Ptr obstacle_cloud, DebugOutputPointCloud::Ptr debug_obstacle_cloud);

   void fillFilteredCloud(const DebugOutputPointCloud::ConstPtr &cloud,
                          DebugOutputPointCloud::Ptr filtered_cloud);

   void plot();

   void calcMedianFromBuffer(const int kernel_size,
                             const int big_kernel_size,
			                     const BufferMediansPtr& buffer,
                             const median_const_iterator& current_element,
                             std::function<float(Detector::InputPoint)> f,
                             float max_dist_for_median_computation,
                             float& small_kernel_val, float& big_kernel_val) const;
private:
   const int PUCK_NUM_RINGS;

   ros::Subscriber m_velodyne_sub;
   ros::Publisher m_pub_obstacle_cloud;
   ros::Publisher m_pub_debug_obstacle_cloud;
   ros::Publisher m_pub_filtered_cloud;

   tf::TransformListener m_tf_listener;

   pcl::visualization::PCLPlotter *m_plotter;

   int m_circular_buffer_capacity_launch;
   float m_angle_between_scanpoints_launch;
   float m_certainty_threshold_launch;
   float m_dist_weight_launch;
   float m_intensity_weight_launch;
   float m_object_size_launch;
   float m_distance_to_comparison_points_launch;
   int m_kernel_size_diff_factor_launch;
   float m_median_min_dist_launch;
   float m_median_thresh1_dist_launch;
   float m_median_thresh2_dist_launch;
   float m_median_max_dist_launch;
   float m_max_dist_for_median_computation_launch;
   int m_max_kernel_size;

   float m_max_prob_by_distance;
   float m_max_intensity_range;

   config_server::Parameter<float> m_certainty_threshold;
   config_server::Parameter<float> m_dist_weight;
   config_server::Parameter<float> m_intensity_weight;
   config_server::Parameter<float> m_weight_for_small_intensities;

   config_server::Parameter<float> m_object_size;
   config_server::Parameter<float> m_distance_to_comparison_points;
   config_server::Parameter<int> m_kernel_size_diff_factor;

   config_server::Parameter<float> m_median_min_dist;
   config_server::Parameter<float> m_median_thresh1_dist;
   config_server::Parameter<float> m_median_thresh2_dist;
   config_server::Parameter<float> m_median_max_dist;

   config_server::Parameter<float> m_max_dist_for_median_computation;

   std::string m_points_topic;

   bool m_publish_debug_clouds;

   boost::mutex m_parameter_change_lock;
   std::vector<BufferMediansPtr> m_median_filtered_circ_buffer_vector;

   std::vector<boost::optional<median_iterator>> m_median_iters_by_ring;
   std::vector<boost::optional<median_iterator>> m_detection_iters_by_ring;

   std::vector<float> m_filtering_factors;
};

} // namespace velodyne_object_detector

#endif // _DETECTOR_H_
