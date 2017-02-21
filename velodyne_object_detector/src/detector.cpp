/** @file

    This class detects objects of a specific size in velodyne laser point clouds

*/

#include "detector.h"

namespace velodyne_object_detector
{
/** @brief Constructor. */
Detector::Detector(ros::NodeHandle node, ros::NodeHandle private_nh)
: PUCK_NUM_RINGS(16)
 , m_circular_buffer_capacity_launch(6000)
 , m_angle_between_scanpoints_launch(0.2f) // 0.1 for 5Hz 0.2 for 10Hz 0.4 for 20Hz
 , m_certainty_threshold_launch(0.0)
 , m_dist_weight_launch(0.75)
 , m_intensity_weight_launch(0.25)
 , m_object_size_launch(1.2f)
 , m_distance_to_comparison_points_launch(2.f)
 , m_kernel_size_diff_factor_launch(5)
 , m_median_min_dist_launch(0.15)
 , m_median_thresh1_dist_launch(0.45)
 , m_median_thresh2_dist_launch(7.0)
 , m_median_max_dist_launch(10.0)
 , m_max_dist_for_median_computation_launch(0.0)
 , m_max_kernel_size(100)
 , m_max_prob_by_distance(1.f)
 , m_max_intensity_range(100.f)
 , m_certainty_threshold("velodyne_object_detector/certainty_threshold", 0.0, 0.01, 1.0, m_certainty_threshold_launch)
 , m_dist_weight("velodyne_object_detector/dist_weight", 0.0, 0.1, 10.0, m_dist_weight_launch)
 , m_intensity_weight("velodyne_object_detector/intensity_weight", 0.0, 0.01, 10.0, m_intensity_weight_launch)
 , m_weight_for_small_intensities("velodyne_object_detector/weight_for_small_intensities", 1.f, 1.f, 30.f, 10.f)
 , m_object_size("velodyne_object_detector/object_size_in_m", 0.005, 0.005, m_object_size_launch*2, m_object_size_launch)
 , m_distance_to_comparison_points("velodyne_object_detector/distance_to_comparison_points", 0.0, 0.01, m_distance_to_comparison_points_launch*2, 0.38f)
 , m_kernel_size_diff_factor("velodyne_object_detector/kernel_size_diff_factor", 1, 1, 20, m_kernel_size_diff_factor_launch)
 , m_median_min_dist("velodyne_object_detector/median_min_dist", 0.0, 0.01, .2, m_median_min_dist_launch)
 , m_median_thresh1_dist("velodyne_object_detector/median_thresh1_dist", 0.0, 0.05, 2.5, m_median_thresh1_dist_launch)
 , m_median_thresh2_dist("velodyne_object_detector/median_thresh2_dist", 0.0, 0.1, 8.0, m_median_thresh2_dist_launch)
 , m_median_max_dist("velodyne_object_detector/median_max_dist", 0.0, 0.5, 20.0, m_median_max_dist_launch)
 , m_max_dist_for_median_computation("velodyne_object_detector/max_dist_for_median_computation", 0.0, 0.25, 10.0, 6.0)
 , m_points_topic("/velodyne_points")
 , m_publish_debug_clouds(false)
{
   ROS_INFO("init velodyne object detector...");

   private_nh.getParam("points_topic", m_points_topic);
   m_velodyne_sub = node.subscribe(m_points_topic, 1000, &Detector::velodyneCallback, this);

   private_nh.getParam("publish_debug_cloud", m_publish_debug_clouds);

   if(m_publish_debug_clouds)
   {
      m_plotter = new pcl::visualization::PCLPlotter ("Detection Plotter");
      m_plotter->setShowLegend (true);
      m_plotter->setXTitle("distance difference in meters");
      m_plotter->setYTitle("object certainty");
      plot();

      m_pub_debug_obstacle_cloud = node.advertise<DebugOutputPointCloud >("/velodyne_detector_debug_objects", 1);
      m_pub_filtered_cloud = node.advertise<DebugOutputPointCloud >("/velodyne_detector_filtered", 1);
   }

   m_pub_obstacle_cloud = node.advertise<OutputPointCloud >("/velodyne_detector_objects", 1);

   if(private_nh.getParam("certainty_threshold", m_certainty_threshold_launch))
      m_certainty_threshold.set(m_certainty_threshold_launch);

   if(private_nh.getParam("dist_weight", m_dist_weight_launch))
      m_dist_weight.set(m_dist_weight_launch);

   if(private_nh.getParam("intensity_weight", m_intensity_weight_launch))
      m_intensity_weight.set(m_intensity_weight_launch);

   if(private_nh.getParam("object_size_in_m", m_object_size_launch))
      m_object_size.set(m_object_size_launch);

   if(private_nh.getParam("distance_to_comparison_points", m_distance_to_comparison_points_launch))
      m_distance_to_comparison_points.set(m_distance_to_comparison_points_launch);

   if(private_nh.getParam("kernel_size_diff_factor", m_kernel_size_diff_factor_launch))
      m_kernel_size_diff_factor.set(m_kernel_size_diff_factor_launch);

   if(private_nh.getParam("median_min_dist", m_median_min_dist_launch))
      m_median_min_dist.set(m_median_min_dist_launch);

   if(private_nh.getParam("median_thresh1_dist", m_median_thresh1_dist_launch))
      m_median_thresh1_dist.set(m_median_thresh1_dist_launch);

   if(private_nh.getParam("median_thresh2_dist", m_median_thresh2_dist_launch))
      m_median_thresh2_dist.set(m_median_thresh2_dist_launch);

   if(private_nh.getParam("median_max_dist", m_median_max_dist_launch))
      m_median_max_dist.set(m_median_max_dist_launch);

   if(private_nh.getParam("max_dist_for_median_computation", m_max_dist_for_median_computation_launch))
      m_max_dist_for_median_computation.set(m_max_dist_for_median_computation_launch);

   private_nh.getParam("circular_buffer_capacity", m_circular_buffer_capacity_launch);
   private_nh.getParam("max_kernel_size", m_max_kernel_size);
   private_nh.getParam("angle_between_scanpoints", m_angle_between_scanpoints_launch);

      for(int i = 0; i < PUCK_NUM_RINGS; i++)
   {    
      BufferMediansPtr median_filtered_circ_buffer(new BufferMedians(m_circular_buffer_capacity_launch));
      m_median_filtered_circ_buffer_vector.push_back(median_filtered_circ_buffer);
   }
   
   m_median_iters_by_ring.resize(PUCK_NUM_RINGS);
   m_detection_iters_by_ring.resize(PUCK_NUM_RINGS);

   m_certainty_threshold.setCallback(boost::bind(&Detector::changeParameterSavely, this));
   m_dist_weight.setCallback(boost::bind(&Detector::changeParameterSavely, this));
   m_intensity_weight.setCallback(boost::bind(&Detector::changeParameterSavely, this));
   m_weight_for_small_intensities.setCallback(boost::bind(&Detector::changeParameterSavely, this));

   m_object_size.setCallback(boost::bind(&Detector::changeParameterSavely, this));
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
   if(m_publish_debug_clouds)
      plot();
}

void Detector::calcMedianFromBuffer(const int kernel_size,
                             const int big_kernel_size,
			                    const BufferMediansPtr& buffer,
                             const median_const_iterator& current_element,
                             std::function<float(Detector::InputPoint)> f,
                             float max_dist_for_median_computation,
                             float& small_kernel_val, float& big_kernel_val) const
{
  assert(std::distance(buffer->begin(), buffer->end())>big_kernel_size);
  
  const int kernel_size_half = kernel_size / 2;
  const int big_kernel_size_half = big_kernel_size / 2;
  
  median_const_iterator small_kernel_start = current_element - kernel_size_half;
  median_const_iterator small_kernel_end = current_element + kernel_size_half;
  median_const_iterator big_kernel_start = current_element - big_kernel_size_half;
  median_const_iterator big_kernel_end = current_element + big_kernel_size_half;
   long int small_kernel_start_offset = -1;
   long int small_kernel_end_offset = -1;

  // get distances of neighbors
  std::vector<float> neighborhood_values;
  neighborhood_values.reserve(big_kernel_size);
  std::vector<float> neighborhood_values_small_dist_kernel;
  neighborhood_values_small_dist_kernel.reserve(kernel_size);

  
  // filter if difference of distances of neighbor and the current point exceeds a threshold
  if(max_dist_for_median_computation == 0.f)
  {
    median_const_iterator it_tmp = big_kernel_start;
     small_kernel_start_offset = std::distance(big_kernel_start, small_kernel_start);
     small_kernel_end_offset = std::distance(big_kernel_start, small_kernel_end);
    // use advance to cast const
    while (it_tmp <= big_kernel_end)
      neighborhood_values.push_back(f((*it_tmp++).point));
  }
  else
  {
      // save distance of midpoint in the buffer aka the current point we are looking at
      const float distance_of_current_point = f((*current_element).point);

      // check for each point in the buffer if it exceeds the distance threshold to the current point
      median_const_iterator it_tmp = big_kernel_start;
     int counter = 0;
      while ( it_tmp <= big_kernel_end )
      {
         const float val_tmp = f((*it_tmp).point);
         const float abs_distance_difference_to_current_point = fabsf(distance_of_current_point - val_tmp);

         if(abs_distance_difference_to_current_point < max_dist_for_median_computation)
         {
            neighborhood_values.push_back(val_tmp);
            if(it_tmp >= small_kernel_start && it_tmp <= small_kernel_end)
            {
               if(small_kernel_start_offset < 0)
                  small_kernel_start_offset = counter;

               small_kernel_end_offset = counter;
            }
         }
         counter++;
         ++it_tmp;
      }
  }

  // get median of neighborhood distances with smaller kernel
   long int small_kernel_middle_offset = (small_kernel_end_offset + small_kernel_start_offset) / 2;
  std::nth_element(neighborhood_values.begin() + small_kernel_start_offset,
                   neighborhood_values.begin() + small_kernel_middle_offset,
                   neighborhood_values.begin() + small_kernel_end_offset);

  small_kernel_val = neighborhood_values[small_kernel_middle_offset];
  
   // get median of neighborhood distances with bigger kernel
  std::nth_element(neighborhood_values.begin(), neighborhood_values.begin() + neighborhood_values.size() / 2, neighborhood_values.end());

  big_kernel_val = neighborhood_values[neighborhood_values.size() / 2];
}

void Detector::velodyneCallback(const InputPointCloud::ConstPtr &input_cloud)
{
   boost::mutex::scoped_lock lock(m_parameter_change_lock);

   pcl::StopWatch timer;

   OutputPointCloud::Ptr obstacle_cloud (new OutputPointCloud);
   obstacle_cloud->header = input_cloud->header;

   DebugOutputPointCloud::Ptr debug_obstacle_cloud (new DebugOutputPointCloud);
   debug_obstacle_cloud->header = input_cloud->header;

   DebugOutputPointCloud::Ptr filtered_cloud (new DebugOutputPointCloud);
   filtered_cloud->header = input_cloud->header;

   for(const auto& point : input_cloud->points)
   {
      m_median_filtered_circ_buffer_vector[point.ring]->push_back(point);
   }

   for (auto ring = 0; ring < PUCK_NUM_RINGS; ++ring)
   {
     // initialize member iterators
     if (!m_median_filtered_circ_buffer_vector.at(ring)->empty() && !m_median_iters_by_ring[ring])
     {
       m_median_iters_by_ring[ring] = m_median_filtered_circ_buffer_vector.at(ring)->begin();
     }

     if (!m_median_filtered_circ_buffer_vector.at(ring)->empty() && !m_detection_iters_by_ring[ring])
     {
       m_detection_iters_by_ring[ring] = m_median_filtered_circ_buffer_vector.at(ring)->begin();
     }
     
     if (m_median_iters_by_ring[ring])
     {
         filterRing(m_median_filtered_circ_buffer_vector.at(ring), *m_median_iters_by_ring.at(ring));
     }

     if (m_detection_iters_by_ring[ring])
     {
	      detectObstacles(m_median_filtered_circ_buffer_vector.at(ring), *m_detection_iters_by_ring.at(ring),
                          obstacle_cloud, debug_obstacle_cloud);
     }
//      ROS_INFO_STREAM("ring : " << ring << " " << m_median_filtered_circ_buffer_vector.at(ring)->size() << " points: " << obstacle_cloud->points.size());
   }

   ROS_DEBUG_STREAM("time for one cloud in ms : " << timer.getTime() );

   if(m_publish_debug_clouds)
   {
      fillFilteredCloud(debug_obstacle_cloud, filtered_cloud);
      m_pub_filtered_cloud.publish(filtered_cloud);
      m_pub_debug_obstacle_cloud.publish(debug_obstacle_cloud);
   }

   m_pub_obstacle_cloud.publish(obstacle_cloud);

}

void Detector::filterRing(std::shared_ptr<boost::circular_buffer<MedianFiltered> > buffer_median_filtered,
                          median_iterator& iter)
{
   while (!buffer_median_filtered->empty() && iter != buffer_median_filtered->end())
   {
      float alpha = static_cast<float>(std::atan((m_object_size()/2.f)/(*iter).point.distance) * 180.f / M_PI);
      int kernel_size = (int)std::ceil(alpha / m_angle_between_scanpoints_launch) + 1;

      kernel_size = std::max(kernel_size, 1);
      kernel_size = std::min(kernel_size, m_max_kernel_size);

      int big_kernel_size = kernel_size * m_kernel_size_diff_factor();
      big_kernel_size = std::max(big_kernel_size, 9);

      const int big_kernel_size_half = big_kernel_size / 2;
      
      if(std::distance(buffer_median_filtered->begin(), iter) >= big_kernel_size_half && std::distance(iter, buffer_median_filtered->end()) > big_kernel_size_half)
      {
        calcMedianFromBuffer(kernel_size, big_kernel_size, buffer_median_filtered, median_const_iterator(iter),
                             [&](const InputPoint &fn) -> float { return fn.distance; },
                             m_max_dist_for_median_computation(),
                             (*iter).dist_small_kernel, (*iter).dist_big_kernel);

        calcMedianFromBuffer(kernel_size, big_kernel_size, buffer_median_filtered, median_const_iterator(iter),
                             [&](const InputPoint &fn) -> float { return fn.intensity; },
                             0.f, 
			     (*iter).intens_small_kernel, (*iter).intens_big_kernel);
      }

      if(std::distance(iter, buffer_median_filtered->end()) <= big_kernel_size_half)
      {
         break;
      }

      ++iter;
   }

}

float Detector::computeCertainty(float difference_distances, float difference_intensities)
{
   float certainty_value = 0.f;
   // cap absolute difference to 0 - m_max_intensity_range
   // and do some kind of weighting, bigger weight -> bigger weight for smaller intensity differences
   difference_intensities = std::max(0.f, difference_intensities);
   difference_intensities = std::min(difference_intensities, m_max_intensity_range/m_weight_for_small_intensities());
   difference_intensities *= m_weight_for_small_intensities();

//   ROS_INFO_STREAM("difference_intensities inner " << (difference_intensities * m_intensity_weight()));


   if(difference_distances < m_median_min_dist() || difference_distances > m_median_max_dist())
   {
      return 0.f;
   }
   else
   {
      if(difference_distances >= m_median_min_dist() && difference_distances < m_median_thresh1_dist())
      {
         certainty_value = difference_distances * m_dist_weight() * (m_max_prob_by_distance/m_median_thresh1_dist()) + difference_intensities * (m_intensity_weight()/m_max_intensity_range);
      }
      if(difference_distances >= m_median_thresh1_dist() && difference_distances < m_median_thresh2_dist())
      {
         certainty_value = m_dist_weight() * m_max_prob_by_distance + difference_intensities * (m_intensity_weight()/m_max_intensity_range);
      }
      if(difference_distances >= m_median_thresh2_dist() && difference_distances < m_median_max_dist())
      {
         certainty_value = (m_max_prob_by_distance / (m_median_max_dist() - m_median_thresh2_dist())) * (m_median_max_dist() - difference_distances * m_dist_weight()) + difference_intensities * (m_intensity_weight()/m_max_intensity_range);
      }
   }
   certainty_value = std::min(certainty_value, 1.0f);
   certainty_value = std::max(certainty_value, 0.0f);

   return certainty_value;
}

void Detector::detectObstacles(std::shared_ptr<boost::circular_buffer<MedianFiltered> > buffer_median_filtered,	median_iterator& median_it,
                               OutputPointCloud::Ptr obstacle_cloud, DebugOutputPointCloud::Ptr debug_obstacle_cloud)

{
   float alpha = static_cast<float>(std::atan(m_distance_to_comparison_points()/(*median_it).dist_small_kernel) * 180.f / M_PI);
   int dist_to_comparsion_point = (int)std::round(alpha / m_angle_between_scanpoints_launch);

   dist_to_comparsion_point = std::max(dist_to_comparsion_point, 0);
   dist_to_comparsion_point = std::min(dist_to_comparsion_point, m_max_kernel_size);

   median_iterator::difference_type dist_to_comparsion_point_bounded = dist_to_comparsion_point;

   if((int)buffer_median_filtered->size() <= 2*dist_to_comparsion_point_bounded+1 || std::distance( median_it, buffer_median_filtered->end()) <= dist_to_comparsion_point_bounded + 1)
   {
      ROS_WARN("not enough medians in buffer");
      return;
   }
   
   for(; median_it != buffer_median_filtered->end(); ++median_it)
   {
      // compute index of neighbors to compare to
      alpha = static_cast<float>(std::atan(m_distance_to_comparison_points()/(*median_it).dist_small_kernel) * 180.f / M_PI);
      dist_to_comparsion_point = (int)std::round(alpha / m_angle_between_scanpoints_launch);

      dist_to_comparsion_point = std::max(dist_to_comparsion_point, 0);
      dist_to_comparsion_point = std::min(dist_to_comparsion_point, m_max_kernel_size);

      dist_to_comparsion_point_bounded = dist_to_comparsion_point;

      if(std::distance(median_it, buffer_median_filtered->end()) <= dist_to_comparsion_point_bounded + 1)
         break;

      auto window_start = median_it;
      if(std::distance(buffer_median_filtered->begin(), median_it) > dist_to_comparsion_point_bounded)
         window_start -= dist_to_comparsion_point_bounded;
      else
         window_start = buffer_median_filtered->begin();

      // probably not necessary, due to if - break statement above
      auto window_end = median_it + dist_to_comparsion_point_bounded;
      if(std::distance(window_end, buffer_median_filtered->end()) < 0)
         window_end = buffer_median_filtered->end() - 1;

      // compute differences and resulting certainty value
      float difference_distance_start = (*median_it).dist_small_kernel - (*window_start).dist_big_kernel;
      float difference_distance_end = (*median_it).dist_small_kernel - (*window_end).dist_big_kernel;

      float difference_distance_sum = difference_distance_start + difference_distance_end;
      float difference_distance_max = std::max(difference_distance_start, difference_distance_end);
      float difference_distances = std::max(difference_distance_sum, difference_distance_max);


      float difference_intensities_start = (*median_it).intens_small_kernel - (*window_start).intens_big_kernel;
      float difference_intensities_end = (*median_it).intens_small_kernel - (*window_end).intens_big_kernel;

      float difference_intensities_sum = difference_intensities_start + difference_intensities_end;
      float difference_intensities_min = std::min(difference_intensities_start, difference_intensities_end);
      float difference_intensities = std::min(difference_intensities_sum, difference_intensities_min);


      float certainty_value = computeCertainty(-difference_distances, difference_intensities);

      if(certainty_value >= m_certainty_threshold())
      {
         const auto& current_point = (*median_it).point;

         OutputPoint output_point;
         output_point.x = current_point.x;
         output_point.y = current_point.y;
         output_point.z = current_point.z;
         output_point.detection = certainty_value;
         obstacle_cloud->push_back(output_point);

         if(m_publish_debug_clouds)
         {
            DebugOutputPoint debug_output_point;
            
            debug_output_point.x = current_point.x;
            debug_output_point.y = current_point.y;
            debug_output_point.z = current_point.z;
            debug_output_point.intensity = current_point.intensity;
            debug_output_point.ring = current_point.ring;
            
            debug_output_point.detection_distance = difference_distances;
            debug_output_point.detection_intensity = debug_obstacle_cloud->size();//difference_intensities;
            debug_output_point.detection = certainty_value;
            
            debug_obstacle_cloud->push_back(debug_output_point);
            
            // save factors for median filtered cloud 
            float factor = 1.f;
            if(!isnan((*median_it).dist_small_kernel) && !isnan(current_point.distance))
               factor = (*median_it).dist_small_kernel / current_point.distance;

            m_filtering_factors.push_back(factor);
         }
      }
   }
}

void Detector::fillFilteredCloud(const DebugOutputPointCloud::ConstPtr &cloud,
                                 DebugOutputPointCloud::Ptr filtered_cloud)
{
   if(cloud->size() != m_filtering_factors.size())
   {
      ROS_ERROR("fillFilteredCloud: cloud and factors have different sizes");
      return;
   }

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

      DebugOutputPointCloud::Ptr cloud_transformed(new DebugOutputPointCloud);
      pcl::transformPointCloud(*cloud, *cloud_transformed, velodyne_link_transform_eigen);

      filtered_cloud->header.frame_id = "/velodyne";

      // move the cloud points to the place they would have been if the median filter would have been applied to them
      for(int point_index = 0; point_index < (int)cloud_transformed->size(); point_index++)
      {
         DebugOutputPoint output_point;
         output_point.x = cloud_transformed->points[point_index].x * m_filtering_factors[point_index];
         output_point.y = cloud_transformed->points[point_index].y * m_filtering_factors[point_index];
         output_point.z = cloud_transformed->points[point_index].z * m_filtering_factors[point_index];
         output_point.detection = cloud_transformed->points[point_index].detection;
         output_point.ring = cloud_transformed->points[point_index].ring;

         filtered_cloud->push_back(output_point);
      }
   }
   m_filtering_factors.clear();
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
   distance_proportion[3] = m_max_prob_by_distance * m_dist_weight();
   distance_proportion[4] = m_max_prob_by_distance * m_dist_weight();
   distance_proportion[5] = 0.0;
   distance_proportion[6] = 0.0;
   distance_proportion[7] = 0.0;

   std::vector<double> intensity_proportion(range, 0.0);
   intensity_proportion[0] = 0.0;
   intensity_proportion[1] = 0.0;
   intensity_proportion[2] = m_max_intensity_range * m_intensity_weight();
   intensity_proportion[3] = m_max_prob_by_distance * m_dist_weight() + m_max_intensity_range * m_intensity_weight();
   intensity_proportion[4] = m_max_prob_by_distance * m_dist_weight() + m_max_intensity_range * m_intensity_weight();
   intensity_proportion[5] = m_max_intensity_range * m_intensity_weight();
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
