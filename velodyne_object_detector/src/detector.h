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
struct MedianFiltered {
   float dist_small_kernel;
   float dist_big_kernel;
   float intens_small_kernel;
   float intens_big_kernel;
};
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

   typedef boost::circular_buffer< InputPoint > BufferInputPoints;
   typedef std::shared_ptr<BufferInputPoints> BufferInputPointsPtr;

   typedef boost::circular_buffer< MedianFiltered > BufferMedians;
   typedef std::shared_ptr<BufferMedians> BufferMediansPtr;


   Detector(ros::NodeHandle node, ros::NodeHandle private_nh);
   ~Detector(){};




   void changeParameterSavely();
   void resizeBuffers();

   void velodyneCallback(const InputPointCloud::ConstPtr &input_cloud);

   void splitCloudByRing(const InputPointCloud::ConstPtr &cloud,
                         std::shared_ptr<std::vector<std::vector<unsigned int> > > clouds_per_ring);

   void filterRing(std::shared_ptr<boost::circular_buffer<InputPoint> > buffer,
                   std::shared_ptr<boost::circular_buffer<MedianFiltered> > buffer_median_filtered);

   float computeCertainty(float difference_distances, float difference_intensities);

   void detectObstacles(std::shared_ptr<boost::circular_buffer<InputPoint> > buffer,
                        std::shared_ptr<boost::circular_buffer<MedianFiltered> > buffer_median_filtered,
                        OutputPointCloud::Ptr obstacle_cloud, DebugOutputPointCloud::Ptr debug_obstacle_cloud);

   bool fillCircularBuffer(const InputPointCloud::ConstPtr &cloud,
                           const std::vector<unsigned int> &indices_of_ring,
                           int ring_index);

   void fillFilteredCloud(const InputPointCloud::ConstPtr &cloud,
                          InputPointCloud::Ptr filtered_cloud,
                          const std::vector<unsigned int> &indices_of_ring,
                          std::shared_ptr<std::vector<float> > distances_ring_filtered_big_kernel);

   void plot();

   void calcMedianFromBuffer(const int kernel_size,
                             const int kernel_size_half,
                             const int big_kernel_size,
                             const int big_kernel_size_half,
                             boost::cb_details::iterator<boost::circular_buffer<Detector::InputPoint, std::allocator<Detector::InputPoint>>, boost::cb_details::nonconst_traits<std::allocator<Detector::InputPoint>>> &it,
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

   float m_max_prob_by_distance;
   float m_max_intensity_range;

   float m_certainty_threshold_launch;
   float m_object_size_launch;
   int m_circular_buffer_capacity_launch;
   int m_distance_to_comparison_points_launch;

   config_server::Parameter<float> m_certainty_threshold;
   config_server::Parameter<float> m_dist_coeff;
   config_server::Parameter<float> m_intensity_coeff;
   config_server::Parameter<float> m_weight_for_small_intensities;

   config_server::Parameter<int> m_object_size;
   config_server::Parameter<int> m_circular_buffer_capacity;
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
   std::vector<BufferInputPointsPtr> m_points_circ_buffer_vector;
   std::vector<BufferMediansPtr> m_median_filtered_circ_buffer_vector;

   std::shared_ptr<std::vector<std::vector<unsigned int> > > m_clouds_per_ring;
   std::shared_ptr<std::vector<std::vector<unsigned int> > > m_old_clouds_per_ring;

   std::vector<std::shared_ptr<std::vector<float> > > m_old_distances_all_rings_filtered_small_kernel;
   std::vector<std::shared_ptr<std::vector<float> > > m_old_distances_all_rings_filtered_big_kernel;
   std::vector<std::shared_ptr<std::vector<float> > > m_old_intensities_all_rings_filtered_small_kernel;
   std::vector<std::shared_ptr<std::vector<float> > > m_old_intensities_all_rings_filtered_big_kernel;

   InputPointCloud::ConstPtr m_old_cloud;

   std::vector<int> m_ring_counter;
};

} // namespace velodyne_object_detector

#endif // _DETECTOR_H_
