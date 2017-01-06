//
// Created by razlaw on 1/4/17.
//

#include "velodyne_object_detector/velodyne_object_detector_nodelet.h"

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(velodyne_object_detector::VelodyneObjectDetectorNodelet, nodelet::Nodelet)

namespace velodyne_object_detector
{

VelodyneObjectDetectorNodelet::VelodyneObjectDetectorNodelet()
: PUCK_NUM_RINGS(16)
 , m_max_prob_by_distance(0.75f)
 , m_max_intensity_range(100.f)
 , m_certainty_threshold_launch(0.5)
 , m_median_small_kernel_size_launch(5)
 , m_median_big_kernel_size_launch(21)
 , m_distance_to_comparison_points_launch(10)
 , m_certainty_threshold("certainty_threshold", 0.0, 0.01, 1.0, m_certainty_threshold_launch)
 , m_dist_coeff("dist_coeff", 0.0, 0.1, 10.0, 1.0)
 , m_intensity_coeff("intensity_coeff", 0.0, 0.0001, 0.01, (1.f - m_max_prob_by_distance)/m_max_intensity_range)
 , m_weight_for_small_intensities("weight_for_small_intensities", 1.f, 1.f, 30.f, 11.f)
 , m_median_small_kernel_size("median_small_kernel_size", 1, 2, m_median_small_kernel_size_launch*2, m_median_small_kernel_size_launch)
 , m_median_big_kernel_size("median_big_kernel_size", 1, 2, m_median_big_kernel_size_launch*2, m_median_big_kernel_size_launch)
 , m_distance_to_comparison_points("distance_to_comparison_points", 1, 1, m_distance_to_comparison_points_launch*2, m_distance_to_comparison_points_launch)
 , m_median_min_dist("median_min_dist", 0.0, 0.01, .2, 0.1)
 , m_median_thresh1_dist("median_thresh1_dist", 0.0, 0.05, 0.5, 0.25)
 , m_median_thresh2_dist("median_thresh2_dist", 0.0, 0.1, 2.0, 1.7)
 , m_median_max_dist("median_max_dist", 0.0, 0.5, 3.0, 3.0)
 , m_max_dist_for_median_computation("max_dist_for_median_computation", 0.0, 0.25, 10.0, 6.0)
 , m_points_topic("/velodyne_points")
 , m_publish_filtered_cloud(false)
{
   ROS_INFO("Initializing velodyne object detector nodelet.. ");
}

void VelodyneObjectDetectorNodelet::onInit()
{
   NODELET_DEBUG("onInit velodyne object detector nodelet...");

   ros::NodeHandle& ph = getPrivateNodeHandle();

   ph.getParam("points_topic", m_points_topic);
   m_velodyne_sub = ph.subscribe(m_points_topic, 1000, &VelodyneObjectDetectorNodelet::velodyneCallback, this);

   ph.getParam("publish_filtered_cloud", m_publish_filtered_cloud);
   m_pub_obstacle_cloud = ph.advertise<OutputPointCloud >("obstacles", 1);
   if(m_publish_filtered_cloud)
      m_pub_filtered_cloud = ph.advertise<InputPointCloud >("filtered", 1);

   if(ph.getParam("certainty_threshold_launch", m_certainty_threshold_launch))
      m_certainty_threshold.set(m_certainty_threshold_launch);

   if(ph.getParam("median_small_kernel_size_launch", m_median_small_kernel_size_launch))
      m_median_small_kernel_size.set(m_median_small_kernel_size_launch);

   if(ph.getParam("median_big_kernel_size_launch", m_median_big_kernel_size_launch))
      m_median_big_kernel_size.set(m_median_big_kernel_size_launch);

   if(ph.getParam("distance_to_comparison_points_launch", m_distance_to_comparison_points_launch))
      m_distance_to_comparison_points.set(m_distance_to_comparison_points_launch);

   m_certainty_threshold.setCallback(boost::bind(&VelodyneObjectDetectorNodelet::nop, this));
   m_dist_coeff.setCallback(boost::bind(&VelodyneObjectDetectorNodelet::nop, this));
   m_intensity_coeff.setCallback(boost::bind(&VelodyneObjectDetectorNodelet::nop, this));
   m_weight_for_small_intensities.setCallback(boost::bind(&VelodyneObjectDetectorNodelet::nop, this));

   m_median_small_kernel_size.setCallback(boost::bind(&VelodyneObjectDetectorNodelet::nop, this));
   m_median_big_kernel_size.setCallback(boost::bind(&VelodyneObjectDetectorNodelet::nop, this));
   m_distance_to_comparison_points.setCallback(boost::bind(&VelodyneObjectDetectorNodelet::nop, this));

   m_median_min_dist.setCallback(boost::bind(&VelodyneObjectDetectorNodelet::nop, this));
   m_median_thresh1_dist.setCallback(boost::bind(&VelodyneObjectDetectorNodelet::nop, this));
   m_median_thresh2_dist.setCallback(boost::bind(&VelodyneObjectDetectorNodelet::nop, this));
   m_median_max_dist.setCallback(boost::bind(&VelodyneObjectDetectorNodelet::nop, this));

   m_max_dist_for_median_computation.setCallback(boost::bind(&VelodyneObjectDetectorNodelet::nop, this));
}

void VelodyneObjectDetectorNodelet::velodyneCallback(const InputPointCloud& input_cloud)
{
   ROS_DEBUG_STREAM("Callback with threshold " << m_certainty_threshold());

   InputPointCloud cloud = input_cloud;

   // save indices of points in one ring in one vector
   // and each vector representing a ring in another vector containing all indices of the cloud
   std::vector<std::vector<unsigned int> > clouds_per_ring(PUCK_NUM_RINGS, std::vector<unsigned int>(0));
   splitCloudByRing(cloud, clouds_per_ring);

   pcl::StopWatch timer;
   double start = timer.getTime();

   detectObstacles(cloud, clouds_per_ring);

   NODELET_INFO_STREAM("Computation time for obstacle detection in ms " << (timer.getTime()- start) << "   \n");
}

// sort points by ring number and save indices in vector
void VelodyneObjectDetectorNodelet::splitCloudByRing(InputPointCloud &cloud, std::vector<std::vector<unsigned int> > &clouds_per_ring)
{
   for(unsigned int point_index = 0; point_index < cloud.size(); point_index++)
   {
      clouds_per_ring[cloud.points[point_index].ring].push_back(point_index);
   }
}

void VelodyneObjectDetectorNodelet::medianFilter(InputPointCloud &cloud,
                                                 std::vector<unsigned int> &indices_of_ring,
                                                 std::vector<float> &filtered_output,
                                                 int kernel_size,
                                                 bool median_of_distances,
                                                 float max_distance_difference)
{
   // make sure kernel size is not even
   if(kernel_size % 2 == 0)
   {
      NODELET_DEBUG("Kernel size has to be odd. Increasing kernel size by 1!");
      kernel_size++;
   }

   for(int ring_point_index = 0; ring_point_index < (int)indices_of_ring.size(); ring_point_index++)
   {
      // get the index of the point in the cloud we are currently looking at
      int current_cloud_point_index = indices_of_ring[ring_point_index];

      std::vector<float> neighborhood_values;
      // get distances of neighbors
      for(int i = -kernel_size/2; i < kernel_size/2; i++)
      {
         // compute index of current neighbor, take into account that it's a scan ring
         int neighbor_ring_point_index = ring_point_index + i;
         if(neighbor_ring_point_index < 0)
         {
            neighbor_ring_point_index += indices_of_ring.size();
         }
         else if(neighbor_ring_point_index >= (int)indices_of_ring.size())
         {
            neighbor_ring_point_index -= indices_of_ring.size();
         }

         int neighbor_cloud_point_index = indices_of_ring[neighbor_ring_point_index];

         // check whether to use distances or intensities for median computation
         if(median_of_distances)
         {
            // filter if difference of distances of neighbor and the current point exceeds a threshold
            if(max_distance_difference == 0.f)
            {
               neighborhood_values.push_back(cloud.points[neighbor_cloud_point_index].distance);
            }
            else
            {
               float abs_distance_difference_to_current_point = fabs(cloud.points[current_cloud_point_index].distance - cloud.points[neighbor_cloud_point_index].distance);
               if(abs_distance_difference_to_current_point < max_distance_difference)
               {
                  neighborhood_values.push_back(cloud.points[neighbor_cloud_point_index].distance);
               }
            }
         }
         else
         {
            neighborhood_values.push_back(cloud.points[neighbor_cloud_point_index].intensity);
         }
      }
      // get median of neighborhood distances
      size_t middle = neighborhood_values.size() / 2;
      std::nth_element(neighborhood_values.begin(), neighborhood_values.begin() + middle, neighborhood_values.end());
      filtered_output[ring_point_index] = neighborhood_values[middle];
   }
}

float VelodyneObjectDetectorNodelet::computeCertainty(float difference_distances, float difference_intensities)
{
   float certainty_value = 0.f;
   // cap absolute difference to 0 - m_max_intensity_range
   // and do some kind of weighting, bigger weight -> bigger weight for smaller intensity differences
   difference_intensities = static_cast<float>(fabs(difference_intensities));
   difference_intensities = std::min(difference_intensities, m_max_intensity_range/m_weight_for_small_intensities());
   difference_intensities *= m_weight_for_small_intensities();

   if(difference_distances < m_median_min_dist() || difference_distances > m_median_max_dist())
   {
      certainty_value = 0.f;
   }
   else{
      if(difference_distances >= m_median_min_dist() && difference_distances < m_median_thresh1_dist())
      {
         certainty_value = difference_distances * m_dist_coeff() * (m_max_prob_by_distance/m_median_thresh1_dist()) + difference_intensities * m_intensity_coeff();
      }
      if(difference_distances >= m_median_thresh1_dist() && difference_distances < m_median_thresh2_dist())
      {
         certainty_value = m_dist_coeff() * m_max_prob_by_distance + difference_intensities * m_intensity_coeff();
      }
      if(difference_distances >= m_median_thresh2_dist() && difference_distances < m_median_max_dist())
      {
         certainty_value = (m_max_prob_by_distance / (m_median_max_dist() - m_median_thresh2_dist())) * (m_median_max_dist() - difference_distances * m_dist_coeff()) + difference_intensities * m_intensity_coeff();
      }
   }
   certainty_value = std::min(certainty_value, 1.0f);
   certainty_value = std::max(certainty_value, 0.0f);

   return certainty_value;
}

void VelodyneObjectDetectorNodelet::detectObstacles(InputPointCloud &cloud,
                                                  std::vector<std::vector<unsigned int> > &clouds_per_ring)
{
   OutputPointCloud obstacle_cloud;
   obstacle_cloud.header = cloud.header;

   InputPointCloud filtered_cloud;
   filtered_cloud.header = cloud.header;

   for(unsigned int ring_index = 0; ring_index < clouds_per_ring.size(); ring_index++)
   {
      bool compute_median_on_distances = true;
      // median filter on distances
      std::vector<float> distances_ring_filtered_small_kernel(clouds_per_ring[ring_index].size(), 0.f);
      std::vector<float> distances_ring_filtered_big_kernel(clouds_per_ring[ring_index].size(), 0.f);
      medianFilter(cloud, clouds_per_ring[ring_index], distances_ring_filtered_small_kernel, m_median_small_kernel_size(), compute_median_on_distances, m_max_dist_for_median_computation());
      // TODO: Test: change this to a kernelsize of ~20cm
      medianFilter(cloud, clouds_per_ring[ring_index], distances_ring_filtered_big_kernel, m_median_big_kernel_size(), compute_median_on_distances, m_max_dist_for_median_computation());

      if(m_publish_filtered_cloud)
      {
         // move the cloud points to the place they would have been if the median filter would have been applied to them
         for(int ring_point_index = 0; ring_point_index < (int) clouds_per_ring[ring_index].size(); ring_point_index++)
         {
            int current_cloud_point_index = clouds_per_ring[ring_index][ring_point_index];
            float factor = distances_ring_filtered_big_kernel[ring_point_index] /
                           cloud.points[current_cloud_point_index].distance;

            InputPoint inputPoint;
            inputPoint.x = cloud.points[current_cloud_point_index].x * factor;
            inputPoint.y = cloud.points[current_cloud_point_index].y * factor;
            inputPoint.z = cloud.points[current_cloud_point_index].z * factor;
            inputPoint.intensity = cloud.points[current_cloud_point_index].intensity;
            inputPoint.ring = cloud.points[current_cloud_point_index].ring;
            inputPoint.distance = cloud.points[current_cloud_point_index].distance;

            filtered_cloud.push_back(inputPoint);
         }
      }

      compute_median_on_distances = false;
      // median filter on intensities
      std::vector<float> intensities_ring_filtered(clouds_per_ring[ring_index].size(), 0.f);
      std::vector<float> intensities_ring_filtered_more(clouds_per_ring[ring_index].size(), 0.f);
      medianFilter(cloud, clouds_per_ring[ring_index], intensities_ring_filtered, m_median_small_kernel_size(), compute_median_on_distances);
      // TODO: Test: change this to a kernelsize of ~20cm
      medianFilter(cloud, clouds_per_ring[ring_index], intensities_ring_filtered_more, m_median_big_kernel_size(), compute_median_on_distances);

      for(int ring_point_index = 0; ring_point_index < (int)clouds_per_ring[ring_index].size(); ring_point_index++)
      {
         // compute index of neighbors to compare to, take into account that it's a scan ring
         int ring_lower_neighbor_index = ring_point_index - m_distance_to_comparison_points();
         if(ring_lower_neighbor_index < 0)
         {
            ring_lower_neighbor_index += (int)clouds_per_ring[ring_index].size();
         }
         int ring_upper_neighbor_index = ring_point_index + m_distance_to_comparison_points();
         if(ring_upper_neighbor_index >= (int)clouds_per_ring[ring_index].size())
         {
            ring_upper_neighbor_index -= (int)clouds_per_ring[ring_index].size();
         }

         // compute differences and resulting certainty value
         float difference_distances = -(distances_ring_filtered_small_kernel[ring_point_index] * 2.f
                                        - distances_ring_filtered_big_kernel[ring_lower_neighbor_index]
                                        - distances_ring_filtered_big_kernel[ring_upper_neighbor_index]);

         float difference_intensities = intensities_ring_filtered[ring_point_index] * 2.f
                                        - intensities_ring_filtered_more[ring_lower_neighbor_index]
                                        - intensities_ring_filtered_more[ring_upper_neighbor_index];

         float certainty_value = computeCertainty(difference_distances, difference_intensities);

         if(certainty_value >= m_certainty_threshold())
         {
            unsigned int index_of_current_point = clouds_per_ring[ring_index][ring_point_index];
            OutputPoint outputPoint;
            outputPoint.x = cloud.points[index_of_current_point].x;
            outputPoint.y = cloud.points[index_of_current_point].y;
            outputPoint.z = cloud.points[index_of_current_point].z;
            outputPoint.detection_distance = difference_distances;
            outputPoint.detection_intensity = difference_intensities;
            outputPoint.detection = certainty_value;

            obstacle_cloud.push_back(outputPoint);
         }
      }
   }
   m_pub_obstacle_cloud.publish(obstacle_cloud);

   if(m_publish_filtered_cloud)
      m_pub_filtered_cloud.publish(filtered_cloud);
}

}