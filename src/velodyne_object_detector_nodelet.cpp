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
 , m_median_big_kernel_size_parameter("median_big_kernel_size", 1, 2, m_median_big_kernel_size_launch*2, m_median_big_kernel_size_launch)
 , m_median_big_kernel_size(m_median_big_kernel_size_parameter())
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
   NODELET_INFO("onInit velodyne object detector nodelet...");

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
      m_median_big_kernel_size_parameter.set(m_median_big_kernel_size_launch);

   if(ph.getParam("distance_to_comparison_points_launch", m_distance_to_comparison_points_launch))
      m_distance_to_comparison_points.set(m_distance_to_comparison_points_launch);

   for(int i = 0; i < PUCK_NUM_RINGS; i++)
   {
      boost::circular_buffer<float> distances_circ_buffer(m_median_big_kernel_size_launch);
      m_distance_median_circ_buffer_vector.push_back(distances_circ_buffer);
      boost::circular_buffer<float> intensities_circ_buffer(m_median_big_kernel_size_launch);
      m_intensity_median_circ_buffer_vector.push_back(intensities_circ_buffer);

      std::shared_ptr<std::vector<float> > tmp_pointer(new std::vector<float>());
      m_old_distances_all_rings_filtered_small_kernel.push_back(tmp_pointer);
      m_old_distances_all_rings_filtered_big_kernel.push_back(tmp_pointer);
      m_old_intensities_all_rings_filtered_small_kernel.push_back(tmp_pointer);
      m_old_intensities_all_rings_filtered_big_kernel.push_back(tmp_pointer);
   }

   for(int i = 0; i < PUCK_NUM_RINGS; i++)
   {
      m_distance_median_circ_buffer_vector[i].set_capacity(m_median_big_kernel_size);
      m_intensity_median_circ_buffer_vector[i].set_capacity(m_median_big_kernel_size);
      m_distance_median_circ_buffer_vector[i].clear();
      m_intensity_median_circ_buffer_vector[i].clear();
   }

   m_ring_counter = std::vector<int>(PUCK_NUM_RINGS, 0);

   m_certainty_threshold.setCallback(boost::bind(&VelodyneObjectDetectorNodelet::changeParameterSavely, this));
   m_dist_coeff.setCallback(boost::bind(&VelodyneObjectDetectorNodelet::changeParameterSavely, this));
   m_intensity_coeff.setCallback(boost::bind(&VelodyneObjectDetectorNodelet::changeParameterSavely, this));
   m_weight_for_small_intensities.setCallback(boost::bind(&VelodyneObjectDetectorNodelet::changeParameterSavely, this));

   m_median_small_kernel_size.setCallback(boost::bind(&VelodyneObjectDetectorNodelet::changeParameterSavely, this));
   m_median_big_kernel_size_parameter.setCallback(boost::bind(&VelodyneObjectDetectorNodelet::resizeBuffers, this));
   m_distance_to_comparison_points.setCallback(boost::bind(&VelodyneObjectDetectorNodelet::changeParameterSavely, this));

   m_median_min_dist.setCallback(boost::bind(&VelodyneObjectDetectorNodelet::changeParameterSavely, this));
   m_median_thresh1_dist.setCallback(boost::bind(&VelodyneObjectDetectorNodelet::changeParameterSavely, this));
   m_median_thresh2_dist.setCallback(boost::bind(&VelodyneObjectDetectorNodelet::changeParameterSavely, this));
   m_median_max_dist.setCallback(boost::bind(&VelodyneObjectDetectorNodelet::changeParameterSavely, this));

   m_max_dist_for_median_computation.setCallback(boost::bind(&VelodyneObjectDetectorNodelet::changeParameterSavely, this));
}

void VelodyneObjectDetectorNodelet::changeParameterSavely()
{
   boost::mutex::scoped_lock lock(m_parameter_change_lock);
   NODELET_INFO("New parameter");
}

void VelodyneObjectDetectorNodelet::resizeBuffers()
{
   boost::mutex::scoped_lock lock(m_parameter_change_lock);

   if(m_median_big_kernel_size_parameter() != (int)m_distance_median_circ_buffer_vector[0].capacity())
   {
      m_median_big_kernel_size = m_median_big_kernel_size_parameter();
      if(m_median_big_kernel_size_parameter() < m_median_small_kernel_size())
      {
         NODELET_ERROR("Resizing buffers. Big kernel size has to be equal or greater than small kernel size! Resetting big kernel size to small kernel size.");
         m_median_big_kernel_size = m_median_small_kernel_size();
      }
      else if(m_median_big_kernel_size % 2 == 0)
      {
         m_median_big_kernel_size++;
         NODELET_ERROR("Resizing buffers. Kernel size has to be odd. Setting kernel size increased by 1 now!");
      }

      for(int i = 0; i < PUCK_NUM_RINGS; i++)
      {
         m_distance_median_circ_buffer_vector[i].set_capacity(m_median_big_kernel_size);
         m_intensity_median_circ_buffer_vector[i].set_capacity(m_median_big_kernel_size);
         m_distance_median_circ_buffer_vector[i].clear();
         m_intensity_median_circ_buffer_vector[i].clear();
         m_ring_counter[i] = 0;
      }
   }
}

void VelodyneObjectDetectorNodelet::velodyneCallback(const InputPointCloud::ConstPtr &input_cloud)
{
   pcl::StopWatch timer;
   double start = timer.getTime();

   // save indices of points in one ring in one vector
   // and each vector representing a ring in another vector containing all indices of the cloud
   m_clouds_per_ring = std::shared_ptr<std::vector<std::vector<unsigned int> > >(new std::vector<std::vector<unsigned int> >(PUCK_NUM_RINGS, std::vector<unsigned int>(0)));
   splitCloudByRing(input_cloud, m_clouds_per_ring);

   detectObstacles(input_cloud, m_clouds_per_ring);
   NODELET_INFO_STREAM("Computation time for obstacle detection in ms " << (timer.getTime()- start) << "   \n");

   m_old_cloud = input_cloud;
   m_old_clouds_per_ring = m_clouds_per_ring;
}

// sort points by ring number and save indices in vector
void VelodyneObjectDetectorNodelet::splitCloudByRing(const InputPointCloud::ConstPtr &cloud,
                                                     std::shared_ptr<std::vector<std::vector<unsigned int> > > clouds_per_ring)
{
   for(unsigned int point_index = 0; point_index < cloud->size(); point_index++)
   {
      (*clouds_per_ring)[cloud->points[point_index].ring].push_back(point_index);
   }
}

void VelodyneObjectDetectorNodelet::filterRing(const InputPointCloud::ConstPtr &cloud,
                                               const std::vector<unsigned int> &indices_of_ring,
                                               int ring_index,
                                               std::shared_ptr<std::vector<float> > distances_ring_filtered_small_kernel,
                                               std::shared_ptr<std::vector<float> > distances_ring_filtered_big_kernel,
                                               std::shared_ptr<std::vector<float> > intensities_ring_filtered_small_kernel,
                                               std::shared_ptr<std::vector<float> > intensities_ring_filtered_big_kernel)
{
   int kernel_size = m_median_big_kernel_size;
   // make sure kernel size is not even
   if(kernel_size % 2 == 0)
   {
      NODELET_ERROR("Kernel size has to be odd!");
   }

   int last_index = (int)indices_of_ring.size() - kernel_size/2;
   int first_index = (-kernel_size/2);
   if(m_ring_counter[ring_index] == 0)
      first_index = 0;

   for(int ring_point_index = first_index; ring_point_index < last_index; ring_point_index++)
   {
      // prepare circular buffers for next iteration
      int point_to_push_back_index = indices_of_ring[ring_point_index + kernel_size/2];
      m_distance_median_circ_buffer_vector[ring_index].push_back(cloud->points[point_to_push_back_index].distance);
      m_intensity_median_circ_buffer_vector[ring_index].push_back(cloud->points[point_to_push_back_index].intensity);

      // get distances of neighbors
      std::vector<float> neighborhood_distances;
      std::vector<float> neighborhood_distances_small_dist_kernel;

      // filter if difference of distances of neighbor and the current point exceeds a threshold
      if(m_max_dist_for_median_computation() == 0.f)
      {
         neighborhood_distances = std::vector<float>(m_distance_median_circ_buffer_vector[ring_index].array_one().first,
                                                  m_distance_median_circ_buffer_vector[ring_index].array_one().first
                                                  + m_distance_median_circ_buffer_vector[ring_index].array_one().second);

         neighborhood_distances.insert(neighborhood_distances.end(),
                                    &m_distance_median_circ_buffer_vector[ring_index].array_two().first[0],
                                    &m_distance_median_circ_buffer_vector[ring_index].array_two().first[m_distance_median_circ_buffer_vector[ring_index].array_two().second]);
      }
      else
      {
         // save distance of midpoint in the buffer aka the current point we are looking at 
         float distance_of_current_point = m_distance_median_circ_buffer_vector[ring_index][m_distance_median_circ_buffer_vector[ring_index].size()/2];
         // check for each point in the buffer if it exceeds the distance threshold to the current point 
         int small_kernel_min_index = m_distance_median_circ_buffer_vector[ring_index].size()/2 - m_median_small_kernel_size()/2;
         int small_kernel_max_index = m_distance_median_circ_buffer_vector[ring_index].size()/2 + m_median_small_kernel_size()/2;
         for(int circ_buffer_index = 0; circ_buffer_index < (int)m_distance_median_circ_buffer_vector[ring_index].size(); circ_buffer_index++)
         {
            float abs_distance_difference_to_current_point = fabs(distance_of_current_point - m_distance_median_circ_buffer_vector[ring_index][circ_buffer_index]);
            if(abs_distance_difference_to_current_point < m_max_dist_for_median_computation())
            {
               neighborhood_distances.push_back(m_distance_median_circ_buffer_vector[ring_index][circ_buffer_index]);
               if(circ_buffer_index >= small_kernel_min_index && circ_buffer_index <= small_kernel_max_index)
                  neighborhood_distances_small_dist_kernel.push_back(m_distance_median_circ_buffer_vector[ring_index][circ_buffer_index]);
            }
         }
      }

      // get median of neighborhood distances with smaller kernel
      size_t middle = neighborhood_distances.size() / 2;
      size_t middle_small_kernel = neighborhood_distances_small_dist_kernel.size() / 2;
      std::nth_element(neighborhood_distances_small_dist_kernel.begin(), neighborhood_distances_small_dist_kernel.begin() + middle_small_kernel, neighborhood_distances_small_dist_kernel.end());
      // if values for the previous/old cloud should be computed
      if(ring_point_index < 0)
      {
         int index_of_old = (int)(*m_old_distances_all_rings_filtered_small_kernel[ring_index]).size() + ring_point_index;
         (*m_old_distances_all_rings_filtered_small_kernel[ring_index])[index_of_old] = neighborhood_distances_small_dist_kernel[middle_small_kernel];
      }
      else
      {
         (*distances_ring_filtered_small_kernel)[ring_point_index] = neighborhood_distances_small_dist_kernel[middle_small_kernel];
      }

      // get median of neighborhood distances with bigger kernel
      std::nth_element(neighborhood_distances.begin(), neighborhood_distances.begin() + middle, neighborhood_distances.end());
      // if values for the previous/old cloud should be computed
      if(ring_point_index < 0)
      {
         int index_of_old = (int)(*m_old_distances_all_rings_filtered_big_kernel[ring_index]).size() + ring_point_index;
         (*m_old_distances_all_rings_filtered_big_kernel[ring_index])[index_of_old] = neighborhood_distances[middle];
      }
      else
      {
         (*distances_ring_filtered_big_kernel)[ring_point_index] = neighborhood_distances[middle];
      }

      // do the same for intensities

      // get intensities of neighbors
      std::vector<float> neighborhood_intensities(m_intensity_median_circ_buffer_vector[ring_index].array_one().first,
                                                  m_intensity_median_circ_buffer_vector[ring_index].array_one().first
                                                  + m_intensity_median_circ_buffer_vector[ring_index].array_one().second);

      neighborhood_intensities.insert(neighborhood_intensities.end(),
                                    &m_intensity_median_circ_buffer_vector[ring_index].array_two().first[0],
                                    &m_intensity_median_circ_buffer_vector[ring_index].array_two().first[m_intensity_median_circ_buffer_vector[ring_index].array_two().second]);

      // get median of neighborhood intensities with smaller kernel
      int offset_to_endings = (kernel_size - m_median_small_kernel_size()) / 2;
      std::nth_element(neighborhood_intensities.begin() + offset_to_endings, neighborhood_intensities.begin() + middle, neighborhood_intensities.end() - offset_to_endings);
      // if values for the previous/old cloud should be computed
      if(ring_point_index < 0)
      {
         int index_of_old = (int)(*m_old_intensities_all_rings_filtered_small_kernel[ring_index]).size() + ring_point_index;
         (*m_old_intensities_all_rings_filtered_small_kernel[ring_index])[index_of_old] = neighborhood_intensities[middle];
      }
      else
      {
         (*intensities_ring_filtered_small_kernel)[ring_point_index] = neighborhood_intensities[middle];
      }

      // get median of neighborhood distances with bigger kernel
      std::nth_element(neighborhood_intensities.begin(), neighborhood_intensities.begin() + middle, neighborhood_intensities.end());
      // if values for the previous/old cloud should be computed
      if(ring_point_index < 0)
      {
         int index_of_old = (int)(*m_old_intensities_all_rings_filtered_big_kernel[ring_index]).size() + ring_point_index;
         (*m_old_intensities_all_rings_filtered_big_kernel[ring_index])[index_of_old] = neighborhood_intensities[middle];
      }
      else
      {
         (*intensities_ring_filtered_big_kernel)[ring_point_index] = neighborhood_intensities[middle];
      }
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

void VelodyneObjectDetectorNodelet::detectObstacles(const InputPointCloud::ConstPtr &cloud,
                                                    const std::shared_ptr<std::vector<std::vector<unsigned int> > > clouds_per_ring)
{
   boost::mutex::scoped_lock lock(m_parameter_change_lock);

   OutputPointCloud::Ptr obstacle_cloud (new OutputPointCloud);
   obstacle_cloud->header = cloud->header;

   InputPointCloud::Ptr filtered_cloud (new InputPointCloud);
   filtered_cloud->header = cloud->header;

   for(unsigned int ring_index = 0; ring_index < clouds_per_ring->size(); ring_index++)
   {
      // check if there are enough points in the ring to compute at least one detection value
      if((int)(*clouds_per_ring)[ring_index].size() < (m_median_big_kernel_size + 2*m_distance_to_comparison_points()))
      {
         m_ring_counter[ring_index] = 0;
         m_distance_median_circ_buffer_vector[ring_index].clear();
         continue;
      }

      // initial filling for circular buffer of distances for this scan ring
      if(!m_distance_median_circ_buffer_vector[ring_index].full())
      {
         // for first filling take zeros where there are no values ( + 1 because this one is going to be replaced later )
         if(m_distance_median_circ_buffer_vector[ring_index].empty())
         {
            for(int i = 0; i < (int)m_distance_median_circ_buffer_vector[ring_index].capacity()/2 + 1; i++)
            {
               m_distance_median_circ_buffer_vector[ring_index].push_back(0.f);
               m_intensity_median_circ_buffer_vector[ring_index].push_back(0.f);
            }
         }

         // fill rest
         for(int ring_point_index = 0; ring_point_index < (int)(*clouds_per_ring)[ring_index].size(); ring_point_index++)
         {
            int current_cloud_point_index = (*clouds_per_ring)[ring_index][ring_point_index];
            m_distance_median_circ_buffer_vector[ring_index].push_back(cloud->points[current_cloud_point_index].distance);
            m_intensity_median_circ_buffer_vector[ring_index].push_back(cloud->points[current_cloud_point_index].intensity);
            if(m_distance_median_circ_buffer_vector[ring_index].full())
               break;
         }
      }

      // probably not necessary
      // if there were not enough points to fill the buffer, skip this ring for now
      if(!m_distance_median_circ_buffer_vector[ring_index].full())
      {
         m_ring_counter[ring_index] = 0;
         m_distance_median_circ_buffer_vector[ring_index].clear();
         continue;
      }

      ros::Time start_median = ros::Time::now();

      // median filter on distances
      std::shared_ptr<std::vector<float> > distances_ring_filtered_small_kernel(new std::vector<float>((*clouds_per_ring)[ring_index].size(), 0.f));
      std::shared_ptr<std::vector<float> > distances_ring_filtered_big_kernel(new std::vector<float>((*clouds_per_ring)[ring_index].size(), 0.f));
      std::shared_ptr<std::vector<float> > intensities_ring_filtered_small_kernel(new std::vector<float>((*clouds_per_ring)[ring_index].size(), 0.f));
      std::shared_ptr<std::vector<float> > intensities_ring_filtered_big_kernel(new std::vector<float>((*clouds_per_ring)[ring_index].size(), 0.f));
      filterRing(cloud, (*clouds_per_ring)[ring_index], ring_index, distances_ring_filtered_small_kernel,
                 distances_ring_filtered_big_kernel, intensities_ring_filtered_small_kernel,
                 intensities_ring_filtered_big_kernel);

      ros::Duration split_dur = ros::Time::now() - start_median;
//      NODELET_INFO_STREAM("Computation time for 4 times median in ns " << split_dur.toNSec() << "   \n");

      if(m_publish_filtered_cloud)
      {
         // move the cloud points to the place they would have been if the median filter would have been applied to them
         int last_index = (int)(*clouds_per_ring)[ring_index].size() - m_median_big_kernel_size/2;
         for(int ring_point_index = 0; ring_point_index < last_index; ring_point_index++)
         {
            int current_cloud_point_index = (*clouds_per_ring)[ring_index][ring_point_index];
            float factor = (*distances_ring_filtered_big_kernel)[ring_point_index] /
                           cloud->points[current_cloud_point_index].distance;

            InputPoint inputPoint;
            inputPoint.x = cloud->points[current_cloud_point_index].x * factor;
            inputPoint.y = cloud->points[current_cloud_point_index].y * factor;
            inputPoint.z = cloud->points[current_cloud_point_index].z * factor;
            inputPoint.intensity = cloud->points[current_cloud_point_index].intensity;
            inputPoint.ring = cloud->points[current_cloud_point_index].ring;
            inputPoint.distance = cloud->points[current_cloud_point_index].distance;

            filtered_cloud->push_back(inputPoint);
         }
         m_pub_filtered_cloud.publish(filtered_cloud);
      }

      ros::Time start_cert_comp = ros::Time::now();

      int last_index = (int)(*clouds_per_ring)[ring_index].size() - m_median_big_kernel_size/2 - m_distance_to_comparison_points();
      int first_index = (-m_median_big_kernel_size/2) - m_distance_to_comparison_points();
      // for first points of first cloud no certainty value can be computed
      if(m_ring_counter[ring_index] == 0)
         first_index = m_distance_to_comparison_points();

      for(int ring_point_index = first_index; ring_point_index < last_index; ring_point_index++)
      {
         // compute index of neighbors to compare to, take into account that it's a scan ring
         // TODO: convert to a distance in meters
         int ring_lower_neighbor_index = ring_point_index - m_distance_to_comparison_points();
         int ring_upper_neighbor_index = ring_point_index + m_distance_to_comparison_points();

         // compute differences and resulting certainty value
         float difference_distances;
         float difference_intensities;

         if(ring_lower_neighbor_index < 0)
         {
            if(ring_point_index < 0)
            {
               if(ring_upper_neighbor_index < 0)
               {
                  int ring_point_index_for_old = (int)(*m_old_clouds_per_ring)[ring_index].size() + ring_point_index;
                  int ring_lower_neighbor_index_for_old = ring_point_index_for_old - m_distance_to_comparison_points();
                  int ring_upper_neighbor_index_for_old = ring_point_index_for_old + m_distance_to_comparison_points();
                  difference_distances = -((*m_old_distances_all_rings_filtered_small_kernel[ring_index])[ring_point_index_for_old] * 2.f
                                           - (*m_old_distances_all_rings_filtered_big_kernel[ring_index])[ring_lower_neighbor_index_for_old]
                                           - (*m_old_distances_all_rings_filtered_big_kernel[ring_index])[ring_upper_neighbor_index_for_old]);

                  difference_intensities = (*m_old_intensities_all_rings_filtered_small_kernel[ring_index])[ring_point_index_for_old] * 2.f
                                           - (*m_old_intensities_all_rings_filtered_big_kernel[ring_index])[ring_lower_neighbor_index_for_old]
                                           - (*m_old_intensities_all_rings_filtered_big_kernel[ring_index])[ring_upper_neighbor_index_for_old];
               }
               else
               {
                  int ring_point_index_for_old = (int)(*m_old_clouds_per_ring)[ring_index].size() + ring_point_index;
                  int ring_lower_neighbor_index_for_old = ring_point_index_for_old - m_distance_to_comparison_points();
                  difference_distances = -((*m_old_distances_all_rings_filtered_small_kernel[ring_index])[ring_point_index_for_old] * 2.f
                                         - (*m_old_distances_all_rings_filtered_big_kernel[ring_index])[ring_lower_neighbor_index_for_old]
                                         - (*distances_ring_filtered_big_kernel)[ring_upper_neighbor_index]);

                  difference_intensities = (*m_old_intensities_all_rings_filtered_small_kernel[ring_index])[ring_point_index_for_old] * 2.f
                                         - (*m_old_intensities_all_rings_filtered_big_kernel[ring_index])[ring_lower_neighbor_index_for_old]
                                         - (*intensities_ring_filtered_big_kernel)[ring_upper_neighbor_index];
               }
            }
            else
            {
               int ring_lower_neighbor_index_for_old = (int)(*m_old_clouds_per_ring)[ring_index].size() - m_distance_to_comparison_points() + ring_point_index;
               difference_distances = -((*distances_ring_filtered_small_kernel)[ring_point_index] * 2.f
                                        - (*m_old_distances_all_rings_filtered_big_kernel[ring_index])[ring_lower_neighbor_index_for_old]
                                        - (*distances_ring_filtered_big_kernel)[ring_upper_neighbor_index]);

               difference_intensities = (*intensities_ring_filtered_small_kernel)[ring_point_index] * 2.f
                                        - (*m_old_intensities_all_rings_filtered_big_kernel[ring_index])[ring_lower_neighbor_index_for_old]
                                        - (*intensities_ring_filtered_big_kernel)[ring_upper_neighbor_index];
            }
         }
         else
         {
            difference_distances = -((*distances_ring_filtered_small_kernel)[ring_point_index] * 2.f
                                   - (*distances_ring_filtered_big_kernel)[ring_lower_neighbor_index]
                                   - (*distances_ring_filtered_big_kernel)[ring_upper_neighbor_index]);

            difference_intensities = (*intensities_ring_filtered_small_kernel)[ring_point_index] * 2.f
                                      - (*intensities_ring_filtered_big_kernel)[ring_lower_neighbor_index]
                                      - (*intensities_ring_filtered_big_kernel)[ring_upper_neighbor_index];
         }

         float certainty_value = computeCertainty(difference_distances, difference_intensities);

         if(certainty_value >= m_certainty_threshold())
         {
            OutputPoint outputPoint;
            if(ring_point_index < 0)
            {
               int ring_point_index_for_old = (int)(*m_old_clouds_per_ring)[ring_index].size() + ring_point_index;
               unsigned int index_of_current_point = (*m_old_clouds_per_ring)[ring_index][ring_point_index_for_old];
               outputPoint.x = m_old_cloud->points[index_of_current_point].x;
               outputPoint.y = m_old_cloud->points[index_of_current_point].y;
               outputPoint.z = m_old_cloud->points[index_of_current_point].z;
            }
            else
            {
               unsigned int index_of_current_point = (*clouds_per_ring)[ring_index][ring_point_index];
               outputPoint.x = cloud->points[index_of_current_point].x;
               outputPoint.y = cloud->points[index_of_current_point].y;
               outputPoint.z = cloud->points[index_of_current_point].z;
            }

            outputPoint.detection_distance = difference_distances;
            outputPoint.detection_intensity = difference_intensities;
            outputPoint.detection = certainty_value;

            obstacle_cloud->push_back(outputPoint);
         }
      }
      m_ring_counter[ring_index] += 1;

      m_old_distances_all_rings_filtered_small_kernel[ring_index] = distances_ring_filtered_small_kernel;
      m_old_distances_all_rings_filtered_big_kernel[ring_index] = distances_ring_filtered_big_kernel;
      m_old_intensities_all_rings_filtered_small_kernel[ring_index] = intensities_ring_filtered_small_kernel;
      m_old_intensities_all_rings_filtered_big_kernel[ring_index] = intensities_ring_filtered_big_kernel;

      split_dur = ros::Time::now() - start_cert_comp;
//      NODELET_INFO_STREAM("Computation time for certainty of 1 ring in ns " << split_dur.toNSec() << "   \n");

   }
   m_pub_obstacle_cloud.publish(obstacle_cloud);
}

}