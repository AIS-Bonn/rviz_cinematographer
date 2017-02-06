/** @file

    This class detects objects of a specific size in velodyne laser point clouds

*/

#include "detector.h"

namespace velodyne_object_detector
{
/** @brief Constructor. */
Detector::Detector(ros::NodeHandle node, ros::NodeHandle private_nh)
: PUCK_NUM_RINGS(16)
 , m_max_prob_by_distance(0.75f)
 , m_max_intensity_range(100.f)
 , m_certainty_threshold_launch(0.5f)
 , m_object_size_launch(1.2f)
 , m_circular_buffer_capacity_launch(1500)
 , m_distance_to_comparison_points_launch(10)
 , m_certainty_threshold("velodyne_object_detector/certainty_threshold", 0.0, 0.01, 1.0, m_certainty_threshold_launch)
 , m_dist_coeff("velodyne_object_detector/dist_coeff", 0.0, 0.1, 10.0, 0.7)//1.0)
 , m_intensity_coeff("velodyne_object_detector/intensity_coeff", 0.0, 0.0001, 0.01, 0.0039)//(1.f - m_max_prob_by_distance)/m_max_intensity_range)
 , m_weight_for_small_intensities("velodyne_object_detector/weight_for_small_intensities", 1.f, 1.f, 30.f, 11.f)
 , m_object_size("velodyne_object_detector/object_size_in_m", 0.005, 0.005, m_object_size_launch*2, m_object_size_launch)
 , m_circular_buffer_capacity("velodyne_object_detector/circular_buffer_capacity", 1, 100, m_circular_buffer_capacity_launch*2, m_circular_buffer_capacity_launch)
 , m_distance_to_comparison_points("velodyne_object_detector/distance_to_comparison_points", 1, 1, m_distance_to_comparison_points_launch*2, m_distance_to_comparison_points_launch)
 , m_median_min_dist("velodyne_object_detector/median_min_dist", 0.0, 0.01, .2, 0.1)
 , m_median_thresh1_dist("velodyne_object_detector/median_thresh1_dist", 0.0, 0.05, 2.5, 0.35)
 , m_median_thresh2_dist("velodyne_object_detector/median_thresh2_dist", 0.0, 0.1, 6.0, 3.7)
 , m_median_max_dist("velodyne_object_detector/median_max_dist", 0.0, 0.5, 20.0, 10.5)
 , m_max_dist_for_median_computation("velodyne_object_detector/max_dist_for_median_computation", 0.0, 0.25, 10.0, 6.0)
 , m_points_topic("/velodyne_points")
 , m_publish_filtered_cloud(false)
 , m_publish_debug_cloud(false)
{
   ROS_INFO("init velodyne object detector...");

   private_nh.getParam("points_topic", m_points_topic);
   m_velodyne_sub = node.subscribe(m_points_topic, 1000, &Detector::velodyneCallback, this);

   private_nh.getParam("publish_filtered_cloud", m_publish_filtered_cloud);
   private_nh.getParam("publish_debug_cloud", m_publish_debug_cloud);

   if(m_publish_debug_cloud)
   {
      m_plotter = new pcl::visualization::PCLPlotter ("Detection Plotter");
      m_plotter->setShowLegend (true);
      m_plotter->setXTitle("distance difference in meters");
      m_plotter->setYTitle("object certainty");
      plot();

      m_pub_debug_obstacle_cloud = node.advertise<DebugOutputPointCloud >("/velodyne_detector_debug_objects", 1);
   }

   if(m_publish_filtered_cloud)
      m_pub_filtered_cloud = node.advertise<InputPointCloud >("/velodyne_detector_filtered", 1);

   m_pub_obstacle_cloud = node.advertise<OutputPointCloud >("/velodyne_detector_objects", 1);

   if(private_nh.getParam("certainty_threshold", m_certainty_threshold_launch))
      m_certainty_threshold.set(m_certainty_threshold_launch);

   if(private_nh.getParam("object_size_in_m", m_object_size_launch))
      m_object_size.set(m_object_size_launch);

   if(private_nh.getParam("circular_buffer_capacity", m_circular_buffer_capacity_launch))
   {
      m_circular_buffer_capacity.set(m_circular_buffer_capacity_launch);
   }

   if(private_nh.getParam("distance_to_comparison_points", m_distance_to_comparison_points_launch))
      m_distance_to_comparison_points.set(m_distance_to_comparison_points_launch);

   for(int i = 0; i < PUCK_NUM_RINGS; i++)
   {
      boost::circular_buffer<InputPoint> points_circ_buffer(m_circular_buffer_capacity());
      m_points_circ_buffer_vector.push_back(points_circ_buffer);

      boost::circular_buffer<MedianFiltered> median_filtered_circ_buffer(m_circular_buffer_capacity());
      m_median_filtered_circ_buffer_vector.push_back(median_filtered_circ_buffer);
   }

   // TODO: anpassen
   m_ring_counter = std::vector<int>(PUCK_NUM_RINGS, 0);

   m_certainty_threshold.setCallback(boost::bind(&Detector::changeParameterSavely, this));
   m_dist_coeff.setCallback(boost::bind(&Detector::changeParameterSavely, this));
   m_intensity_coeff.setCallback(boost::bind(&Detector::changeParameterSavely, this));
   m_weight_for_small_intensities.setCallback(boost::bind(&Detector::changeParameterSavely, this));

   m_object_size.setCallback(boost::bind(&Detector::changeParameterSavely, this));
   m_circular_buffer_capacity.setCallback(boost::bind(&Detector::resizeBuffers, this));
   m_distance_to_comparison_points.setCallback(boost::bind(&Detector::changeParameterSavely, this));

   m_median_min_dist.setCallback(boost::bind(&Detector::changeParameterSavely, this));
   m_median_thresh1_dist.setCallback(boost::bind(&Detector::changeParameterSavely, this));
   m_median_thresh2_dist.setCallback(boost::bind(&Detector::changeParameterSavely, this));
   m_median_max_dist.setCallback(boost::bind(&Detector::changeParameterSavely, this));

   m_max_dist_for_median_computation.setCallback(boost::bind(&Detector::changeParameterSavely, this));
}

void Detector::changeParameterSavely()
{
   boost::mutex::scoped_lock lock(m_parameter_change_lock);
   ROS_DEBUG("New parameter");
   if(m_publish_debug_cloud)
      plot();
}

//void Detector::resizeBuffers()
//{
//   boost::mutex::scoped_lock lock(m_parameter_change_lock);
//
//   if(m_median_big_kernel_size_parameter() != (int)m_distance_median_circ_buffer_vector[0].capacity())
//   {
//      m_median_big_kernel_size = m_median_big_kernel_size_parameter();
//      if(m_median_big_kernel_size_parameter() < m_median_small_kernel_size())
//      {
//         ROS_ERROR("Resizing buffers. Big kernel size has to be equal or greater than small kernel size! Resetting big kernel size to small kernel size.");
//         m_median_big_kernel_size = m_median_small_kernel_size();
//      }
//      else if(m_median_big_kernel_size % 2 == 0)
//      {
//         m_median_big_kernel_size++;
//         ROS_ERROR("Resizing buffers. Kernel size has to be odd. Setting kernel size increased by 1 now!");
//      }
//
//      for(int i = 0; i < PUCK_NUM_RINGS; i++)
//      {
//         m_distance_median_circ_buffer_vector[i].set_capacity(m_median_big_kernel_size);
//         m_intensity_median_circ_buffer_vector[i].set_capacity(m_median_big_kernel_size);
//         m_distance_median_circ_buffer_vector[i].clear();
//         m_intensity_median_circ_buffer_vector[i].clear();
//         m_ring_counter[i] = 0;
//      }
//   }
//}

void Detector::velodyneCallback(const InputPointCloud::ConstPtr &input_cloud)
{
//   pcl::StopWatch timer;
//   double start = timer.getTime();

   for(const auto& point : input_cloud->points)
   {
      m_points_circ_buffer_vector[point.ring].push_back(point);
   }

//   // save indices of points in one ring in one vector
//   // and each vector representing a ring in another vector containing all indices of the cloud
//   m_clouds_per_ring = std::shared_ptr<std::vector<std::vector<unsigned int> > >(new std::vector<std::vector<unsigned int> >(PUCK_NUM_RINGS, std::vector<unsigned int>(0)));
//   splitCloudByRing(input_cloud, m_clouds_per_ring);
//
//   detectObstacles(input_cloud, m_clouds_per_ring);
//   ROS_DEBUG_STREAM("Computation time for obstacle detection in ms " << (timer.getTime()- start) << "   \n");
//
//   m_old_cloud = input_cloud;
//   m_old_clouds_per_ring = m_clouds_per_ring;
}

// sort points by ring number and save indices in vector
void Detector::splitCloudByRing(const InputPointCloud::ConstPtr &cloud,
                                                     std::shared_ptr<std::vector<std::vector<unsigned int> > > clouds_per_ring)
{
   for(unsigned int point_index = 0; point_index < cloud->size(); point_index++)
   {
      (*clouds_per_ring)[cloud->points[point_index].ring].push_back(point_index);
   }
}

void Detector::filterRing(std::shared_ptr<boost::circular_buffer<InputPoint> > buffer,
                          std::shared_ptr<boost::circular_buffer<MedianFiltered> > buffer_median_filtered)
{
   int kernel_size = m_median_big_kernel_size;
   // make sure kernel size is not even
   if(kernel_size % 2 == 0)
   {
      ROS_ERROR("Kernel size has to be odd!");
   }

   int last_index = (int)indices_of_ring.size() - kernel_size/2;
   int first_index = (-kernel_size/2);

   typedef typename boost::circular_buffer<InputPoint>::iterator buffer_iterator;
   buffer_iterator it = buffer->begin();
   while (!buffer->empty())
   {
      float alpha = static_cast<float>(std::atan((m_object_size()/2.f)/(*it)) * 180.f / M_PI);
      const int kernel_size = (int)std::ceil(alpha / ANGLE_BETWEEN_SCANPOINTS) + 1;

      const int kernel_size_half = kernel_size/2;
      const int big_kernel_size = kernel_size*4;
      const int big_kernel_size_half = kernel_size*2;


      // TODO check if kernel_size is valid

      if ( (buffer->end() - big_kernel_size_half) > it && (buffer->begin() + big_kernel_size_half) <= it ){




         // get distances of neighbors
         std::vector<float> neighborhood_distances;
         neighborhood_distances.reserve(big_kernel_size);
         std::vector<float> neighborhood_distances_small_dist_kernel;
         neighborhood_distances_small_dist_kernel.reserve(kernel_size);

         // filter if difference of distances of neighbor and the current point exceeds a threshold
         if(m_max_dist_for_median_computation() == 0.f){
            neighborhood_distances = std::vector<float>( it - big_kernel_size_half, it + big_kernel_size_half);
         }
         else{
            // save distance of midpoint in the buffer aka the current point we are looking at
            float distance_of_current_point = (*it).distance ;

            buffer_iterator small_kernel_start = it - kernel_size_half;
            buffer_iterator small_kernel_end = it + kernel_size_half;
            buffer_iterator big_kernel_start = it - big_kernel_size_half;
            buffer_iterator big_kernel_end = it + big_kernel_size_half;

            // check for each point in the buffer if it exceeds the distance threshold to the current point
            while ( big_kernel_end != big_kernel_start++ ){
               float abs_distance_difference_to_current_point = fabs(
                       distance_of_current_point - big_kernel_start->distance );
               if(abs_distance_difference_to_current_point < m_max_dist_for_median_computation()){
                  neighborhood_distances.push_back(big_kernel_start->distance);
                  if(big_kernel_start >= small_kernel_start && big_kernel_start <= small_kernel_end)
                     neighborhood_distances_small_dist_kernel.push_back(
                            big_kernel_start->distance);
               }
            }
         }

         // get median of neighborhood distances with smaller kernel
         size_t middle = neighborhood_distances.size() / 2;
         size_t middle_small_kernel = neighborhood_distances_small_dist_kernel.size() / 2;
         std::nth_element(neighborhood_distances_small_dist_kernel.begin(),
                          neighborhood_distances_small_dist_kernel.begin() + middle_small_kernel,
                          neighborhood_distances_small_dist_kernel.end());

         MedianFiltered mfs;
         mfs.dist_small_kernel = neighborhood_distances_small_dist_kernel.at(middle_small_kernel);

         // TODO zuerst small dann big

         // get median of neighborhood distances with bigger kernel
         std::nth_element(neighborhood_distances.begin(), neighborhood_distances.begin() + middle,
                          neighborhood_distances.end());

         mfs.dist_big_kernel = neighborhood_distances.at(middle);



         // do the same for intensities

         // get intensities of neighbors
         std::vector<float> neighborhood_intensities(
                 m_intensity_median_circ_buffer_vector[ring_index].array_one().first,
                 m_intensity_median_circ_buffer_vector[ring_index].array_one().first
                 + m_intensity_median_circ_buffer_vector[ring_index].array_one().second);

         neighborhood_intensities.insert(neighborhood_intensities.end(),
                                         &m_intensity_median_circ_buffer_vector[ring_index].array_two().first[0],
                                         &m_intensity_median_circ_buffer_vector[ring_index].array_two().first[m_intensity_median_circ_buffer_vector[ring_index].array_two().second]);

         // get median of neighborhood intensities with smaller kernel
         int offset_to_endings = (kernel_size - m_median_small_kernel_size()) / 2;
         std::nth_element(neighborhood_intensities.begin() + offset_to_endings,
                          neighborhood_intensities.begin() + middle,
                          neighborhood_intensities.end() - offset_to_endings);
         // if values for the previous/old cloud should be computed
         if(ring_point_index < 0){
            int index_of_old =
                    (int) (*m_old_intensities_all_rings_filtered_small_kernel[ring_index]).size() + ring_point_index;
            (*m_old_intensities_all_rings_filtered_small_kernel[ring_index])[index_of_old] = neighborhood_intensities[middle];
         }
         else{
            (*intensities_ring_filtered_small_kernel)[ring_point_index] = neighborhood_intensities[middle];
         }

         // get median of neighborhood distances with bigger kernel
         std::nth_element(neighborhood_intensities.begin(), neighborhood_intensities.begin() + middle,
                          neighborhood_intensities.end());
         // if values for the previous/old cloud should be computed
         if(ring_point_index < 0){
            int index_of_old =
                    (int) (*m_old_intensities_all_rings_filtered_big_kernel[ring_index]).size() + ring_point_index;
            (*m_old_intensities_all_rings_filtered_big_kernel[ring_index])[index_of_old] = neighborhood_intensities[middle];
         }
         else{
            (*intensities_ring_filtered_big_kernel)[ring_point_index] = neighborhood_intensities[middle];
         }
      }
      else
      {
         break;
      }

   }

}

float Detector::computeCertainty(float difference_distances, float difference_intensities)
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

void Detector::detectObstacles(const InputPointCloud::ConstPtr &cloud,
                                                    const std::shared_ptr<std::vector<std::vector<unsigned int> > > clouds_per_ring)
{
   boost::mutex::scoped_lock lock(m_parameter_change_lock);

   DebugOutputPointCloud::Ptr debug_obstacle_cloud (new DebugOutputPointCloud);
   debug_obstacle_cloud->header = cloud->header;

   OutputPointCloud::Ptr obstacle_cloud (new OutputPointCloud);
   obstacle_cloud->header = cloud->header;

   InputPointCloud::Ptr filtered_cloud (new InputPointCloud);
   filtered_cloud->header = cloud->header;
   filtered_cloud->header.frame_id = "/velodyne";

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
      if(!fillCircularBuffer(cloud, (*clouds_per_ring)[ring_index], ring_index))
      {
         continue;
      }

      // median filter on distances
      std::shared_ptr<std::vector<float> > distances_ring_filtered_small_kernel(new std::vector<float>((*clouds_per_ring)[ring_index].size(), 0.f));
      std::shared_ptr<std::vector<float> > distances_ring_filtered_big_kernel(new std::vector<float>((*clouds_per_ring)[ring_index].size(), 0.f));
      std::shared_ptr<std::vector<float> > intensities_ring_filtered_small_kernel(new std::vector<float>((*clouds_per_ring)[ring_index].size(), 0.f));
      std::shared_ptr<std::vector<float> > intensities_ring_filtered_big_kernel(new std::vector<float>((*clouds_per_ring)[ring_index].size(), 0.f));
      filterRing(cloud, (*clouds_per_ring)[ring_index], ring_index, distances_ring_filtered_small_kernel,
                 distances_ring_filtered_big_kernel, intensities_ring_filtered_small_kernel,
                 intensities_ring_filtered_big_kernel);

      if(m_publish_filtered_cloud)
      {
         fillFilteredCloud(cloud, filtered_cloud, (*clouds_per_ring)[ring_index], distances_ring_filtered_big_kernel);
      }

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
            if(m_publish_debug_cloud)
            {
               DebugOutputPoint debug_output_point;
               if(ring_point_index < 0)
               {
                  int ring_point_index_for_old = (int)(*m_old_clouds_per_ring)[ring_index].size() + ring_point_index;
                  unsigned int index_of_current_point = (*m_old_clouds_per_ring)[ring_index][ring_point_index_for_old];
                  debug_output_point.x = m_old_cloud->points[index_of_current_point].x;
                  debug_output_point.y = m_old_cloud->points[index_of_current_point].y;
                  debug_output_point.z = m_old_cloud->points[index_of_current_point].z;
                  debug_output_point.intensity = m_old_cloud->points[index_of_current_point].intensity;
                  debug_output_point.ring = m_old_cloud->points[index_of_current_point].ring;
               }
               else
               {
                  unsigned int index_of_current_point = (*clouds_per_ring)[ring_index][ring_point_index];
                  debug_output_point.x = cloud->points[index_of_current_point].x;
                  debug_output_point.y = cloud->points[index_of_current_point].y;
                  debug_output_point.z = cloud->points[index_of_current_point].z;
                  debug_output_point.intensity = cloud->points[index_of_current_point].intensity;
                  debug_output_point.ring = cloud->points[index_of_current_point].ring;
               }

               debug_output_point.detection_distance = difference_distances;
               debug_output_point.detection_intensity = difference_intensities;
               debug_output_point.detection = certainty_value;

               debug_obstacle_cloud->push_back(debug_output_point);
            }

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

            outputPoint.detection = certainty_value;

            obstacle_cloud->push_back(outputPoint);
         }
      }
      m_ring_counter[ring_index] += 1;

      m_old_distances_all_rings_filtered_small_kernel[ring_index] = distances_ring_filtered_small_kernel;
      m_old_distances_all_rings_filtered_big_kernel[ring_index] = distances_ring_filtered_big_kernel;
      m_old_intensities_all_rings_filtered_small_kernel[ring_index] = intensities_ring_filtered_small_kernel;
      m_old_intensities_all_rings_filtered_big_kernel[ring_index] = intensities_ring_filtered_big_kernel;
   }

   if(m_publish_debug_cloud)
      m_pub_debug_obstacle_cloud.publish(debug_obstacle_cloud);

   if(m_publish_filtered_cloud)
      m_pub_filtered_cloud.publish(filtered_cloud);

   m_pub_obstacle_cloud.publish(obstacle_cloud);
}

bool Detector::fillCircularBuffer(const InputPointCloud::ConstPtr &cloud,
                                                       const std::vector<unsigned int> &indices_of_ring,
                                                       int ring_index)
{
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
      for(int ring_point_index = 0; ring_point_index < (int)indices_of_ring.size(); ring_point_index++)
      {
         int current_cloud_point_index = indices_of_ring[ring_point_index];
         m_distance_median_circ_buffer_vector[ring_index].push_back(cloud->points[current_cloud_point_index].distance);
         m_intensity_median_circ_buffer_vector[ring_index].push_back(cloud->points[current_cloud_point_index].intensity);
         if(m_distance_median_circ_buffer_vector[ring_index].full())
            break;
      }
   }

   // if there were not enough points to fill the buffer, skip this ring for now
   if(!m_distance_median_circ_buffer_vector[ring_index].full())
   {
      m_ring_counter[ring_index] = 0;
      m_distance_median_circ_buffer_vector[ring_index].clear();
      return false;
   }

   return true;
}

void Detector::fillFilteredCloud(const InputPointCloud::ConstPtr &cloud,
                                                      InputPointCloud::Ptr filtered_cloud,
                                                      const std::vector<unsigned int> &indices_of_ring,
                                                      std::shared_ptr<std::vector<float> > distances_ring_filtered_big_kernel)
{
   tf::StampedTransform velodyne_link_transform;
   bool transform_found = true;
   try
   {
      m_tf_listener.lookupTransform("/velodyne", cloud->header.frame_id, pcl_conversions::fromPCL(cloud->header.stamp), velodyne_link_transform);
   }
   catch(tf::TransformException& ex)
   {
      ROS_ERROR("Transform unavailable %s", ex.what());
      transform_found = false;
   }

   if(transform_found)
   {
      Eigen::Affine3d velodyne_link_transform_eigen;
      tf::transformTFToEigen(velodyne_link_transform, velodyne_link_transform_eigen);

      InputPointCloud::Ptr cloud_transformed(new InputPointCloud);
      pcl::transformPointCloud(*cloud, *cloud_transformed, velodyne_link_transform_eigen);

      // move the cloud points to the place they would have been if the median filter would have been applied to them
      int last_index = (int) indices_of_ring.size() - m_median_big_kernel_size / 2;
      for(int ring_point_index = 0; ring_point_index < last_index; ring_point_index++)
      {
         int current_cloud_point_index = indices_of_ring[ring_point_index];
         float factor = (*distances_ring_filtered_big_kernel)[ring_point_index] /
                        cloud->points[current_cloud_point_index].distance;

         InputPoint inputPoint;
         inputPoint.x = cloud_transformed->points[current_cloud_point_index].x * factor;
         inputPoint.y = cloud_transformed->points[current_cloud_point_index].y * factor;
         inputPoint.z = cloud_transformed->points[current_cloud_point_index].z * factor;
         inputPoint.intensity = cloud_transformed->points[current_cloud_point_index].intensity;
         inputPoint.ring = cloud_transformed->points[current_cloud_point_index].ring;
         inputPoint.distance = cloud_transformed->points[current_cloud_point_index].distance;

         filtered_cloud->push_back(inputPoint);
      }
   }
}

void Detector::plot()
{
   // set up x-axis
   double epsilon = 0.00000001;
   const int range = 8;
   std::vector<double> xAxis(range, 0.0);
   xAxis[0] = 0.0;
   xAxis[1] = m_median_min_dist();
   xAxis[2] = m_median_min_dist() + epsilon;
   xAxis[3] = m_median_thresh1_dist();
   xAxis[4] = m_median_thresh2_dist();
   xAxis[5] = m_median_max_dist();
   xAxis[6] = m_median_max_dist() + epsilon;
   xAxis[7] = m_median_max_dist() + 0.5;

   std::vector<double> constant_one(range, 0.0);
   for(int i = 0; i < range; i++) constant_one[i] = 1.0 + epsilon;

   std::vector<double> distance_proportion(range, 0.0);
   distance_proportion[0] = 0.0;
   distance_proportion[1] = 0.0;
   distance_proportion[2] = 0.0;
   distance_proportion[3] = m_max_prob_by_distance * m_dist_coeff();
   distance_proportion[4] = m_max_prob_by_distance * m_dist_coeff();
   distance_proportion[5] = 0.0;
   distance_proportion[6] = 0.0;
   distance_proportion[7] = 0.0;

   std::vector<double> intensity_proportion(range, 0.0);
   intensity_proportion[0] = 0.0;
   intensity_proportion[1] = 0.0;
   intensity_proportion[2] = m_max_intensity_range * m_intensity_coeff();
   intensity_proportion[3] = m_max_prob_by_distance * m_dist_coeff() + m_max_intensity_range * m_intensity_coeff();
   intensity_proportion[4] = m_max_prob_by_distance * m_dist_coeff() + m_max_intensity_range * m_intensity_coeff();
   intensity_proportion[5] = m_max_intensity_range * m_intensity_coeff();
   intensity_proportion[6] = 0.0;
   intensity_proportion[7] = 0.0;

   // add histograms to plotter
   std::vector<char> black{0, 0, 0, (char) 255};
   std::vector<char> red{(char) 255, 0, 0, (char) 255};
   std::vector<char> green{0, (char) 255, 0, (char) 255};

   m_plotter->clearPlots();

   m_plotter->setYRange(0.0, std::max(1.1, intensity_proportion[3]));
   m_plotter->addPlotData(xAxis, distance_proportion, "Distance proportion", vtkChart::LINE, red);
   m_plotter->addPlotData(xAxis, intensity_proportion, "Intensity proportion", vtkChart::LINE, green);
   m_plotter->addPlotData(xAxis, constant_one, "One", vtkChart::LINE, black);

   m_plotter->spinOnce(0);
}

} // namespace velodyne_object_detector
