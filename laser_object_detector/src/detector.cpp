/** @file

    This class detects objects of a specific size in laser point clouds

*/

#include "detector.h"

namespace laser_object_detector
{
/** @brief Constructor. */
Detector::Detector(ros::NodeHandle node, ros::NodeHandle private_nh)
: PUCK_NUM_RINGS(16)
 , HOKUYO_NUM_RINGS(1)  
 , m_input_is_velodyne(true)       
 , m_circular_buffer_capacity_launch(6000)
 , m_angle_between_scanpoints_launch(0.2f) // 0.1 for 5Hz 0.2 for 10Hz 0.4 for 20Hz
 , m_certainty_threshold_launch(0.0)
 , m_dist_weight_launch(0.75)
 , m_intensity_weight_launch(0.25)
 , m_object_size_launch(1.2f)
 , m_distance_to_comparison_points_launch(2.f)
 , m_kernel_size_diff_factor_launch(5.0)
 , m_median_min_dist_launch(2.5)
 , m_median_thresh1_dist_launch(5.0)
 , m_median_thresh2_dist_launch(200.0)
 , m_median_max_dist_launch(200.0)
 , m_max_dist_for_median_computation_launch(0.0)
 , m_max_kernel_size(100)
 , m_max_prob_by_distance(1.f)
 , m_max_intensity_range(100.f)
 , m_certainty_threshold("laser_object_detector/certainty_threshold", 0.0, 0.01, 1.0, m_certainty_threshold_launch)
 , m_dist_weight("laser_object_detector/dist_weight", 0.0, 0.1, 10.0, m_dist_weight_launch)
 , m_intensity_weight("laser_object_detector/intensity_weight", 0.0, 0.01, 10.0, m_intensity_weight_launch)
 , m_weight_for_small_intensities("laser_object_detector/weight_for_small_intensities", 1.f, 1.f, 30.f, 10.f)
 , m_object_size("laser_object_detector/object_size_in_m", 0.005, 0.005, 5.0, m_object_size_launch)
 , m_distance_to_comparison_points("laser_object_detector/distance_to_comparison_points", 0.0, 0.01, 10.0, 0.38f)
 , m_kernel_size_diff_factor("laser_object_detector/kernel_size_diff_factor", 1.0, 0.1, 5.0, m_kernel_size_diff_factor_launch)
 , m_median_min_dist("laser_object_detector/median_min_dist", 0.0, 0.01, 5.0, m_median_min_dist_launch)
 , m_median_thresh1_dist("laser_object_detector/median_thresh1_dist", 0.0001, 0.05, 12.5, m_median_thresh1_dist_launch)
 , m_median_thresh2_dist("laser_object_detector/median_thresh2_dist", 0.0, 0.1, 200.0, m_median_thresh2_dist_launch)
 , m_median_max_dist("laser_object_detector/median_max_dist", 0.0, 0.5, 200.0, m_median_max_dist_launch)
 , m_max_dist_for_median_computation("laser_object_detector/max_dist_for_median_computation", 0.0, 0.25, 10.0, 6.0)
 , m_input_topic("/velodyne_points")
 , m_publish_debug_clouds(false)
 , m_buffer_initialized(false)
{
   ROS_INFO("init laser object detector...");

   private_nh.getParam("input_topic", m_input_topic);
   private_nh.getParam("input_is_velodyne", m_input_is_velodyne);
   if(m_input_is_velodyne)
     m_velodyne_sub = node.subscribe(m_input_topic, 1, &Detector::velodyneCallback, this);
   else
     m_hokuyo_sub = node.subscribe(m_input_topic, 1, &Detector::hokuyoCallback, this);
  
   private_nh.getParam("publish_debug_cloud", m_publish_debug_clouds);

   if(m_publish_debug_clouds)
   {
      m_plotter = new pcl::visualization::PCLPlotter ("Detection Plotter");
      m_plotter->setShowLegend (true);
      m_plotter->setXTitle("distance difference in meters");
      m_plotter->setYTitle("object certainty");
      plot();

      m_pub_debug_obstacle_cloud = node.advertise<DebugOutputPointCloud >("/laser_detector_debug_objects", 1);
      m_pub_filtered_cloud = node.advertise<DebugOutputPointCloud >("/laser_detector_filtered", 1);
   }

   m_pub_obstacle_cloud = node.advertise<OutputPointCloud >("/laser_detector_objects", 1);

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

void Detector::initBuffer(int number_of_rings)
{
  for(int i = 0; i < number_of_rings; i++)
  {
    BufferMediansPtr median_filtered_circ_buffer(new BufferMedians(m_circular_buffer_capacity_launch));
    m_median_filtered_circ_buffer_vector.push_back(median_filtered_circ_buffer);
  }

  m_median_iters_by_ring.resize(number_of_rings);
  m_detection_iters_by_ring.resize(number_of_rings);

  m_buffer_initialized = true;
}

void Detector::resetBuffer()
{
  for(int ring = 0; ring < (int)m_median_filtered_circ_buffer_vector.size(); ring++)
  {
    m_median_filtered_circ_buffer_vector.at(ring)->clear();
    m_median_iters_by_ring.at(ring).reset();
    m_detection_iters_by_ring.at(ring).reset();
  }
}

void Detector::hokuyoCallback(const sensor_msgs::LaserScanConstPtr& input_scan)
{
  if(m_pub_obstacle_cloud.getNumSubscribers() == 0 &&
     m_pub_debug_obstacle_cloud.getNumSubscribers() == 0 &&
     m_pub_filtered_cloud.getNumSubscribers() == 0)
  {
    ROS_DEBUG_STREAM("hokuyoCallback: no subscriber to laser_object_detector. resetting buffer");
    resetBuffer();
    return;
  }

  if(!m_buffer_initialized)
    initBuffer(HOKUYO_NUM_RINGS);
  
  boost::mutex::scoped_lock lock(m_parameter_change_lock);

  sensor_msgs::PointCloud2 cloud;
  InputPointCloud::Ptr cloud_transformed(new InputPointCloud());

  std::string frame_id = "base_link";

  if (!m_tf_listener.waitForTransform(frame_id, input_scan->header.frame_id, input_scan->header.stamp + ros::Duration().fromSec((input_scan->ranges.size()) * input_scan->time_increment), ros::Duration(0.1)))
  {
    ROS_ERROR_THROTTLE(10.0, "hokuyoCallback: could not wait for transform");
    return;
  }

  // transform 2D scan line to 3D point cloud
  try
  {
    m_scan_projector.transformLaserScanToPointCloud(frame_id, *input_scan, cloud, m_tf_listener, 35.f,
                                                    (laser_geometry::channel_option::Intensity | laser_geometry::channel_option::Distance));

    // fix fields.count member
    for (unsigned int i = 0; i < cloud.fields.size(); i++)
      cloud.fields[i].count = 1;

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
    sensor_msgs::PointCloud2Iterator<float> iter_intensity(cloud, "intensity");
    sensor_msgs::PointCloud2Iterator<float> iter_distance(cloud, "distances");

    cloud_transformed->points.reserve(cloud.height * cloud.width);
    for(; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_intensity, ++iter_distance)
    {
      if(std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z))
        continue;

      InputPoint point;
      point.x = *iter_x;
      point.y = *iter_y;
      point.z = *iter_z;
      point.intensity = *iter_intensity;
      point.distance = *iter_distance;
      point.ring = 0;

      m_median_filtered_circ_buffer_vector[point.ring]->push_back(point);
    }
  }
  catch (tf::TransformException& exc)
  {
    ROS_ERROR_THROTTLE(10.0, "hokuyoCallback: No transform found");
    ROS_ERROR_THROTTLE(10.0, "message: '%s'", exc.what());
  }

  processScan(pcl_conversions::toPCL(cloud.header));
}

void Detector::velodyneCallback(const InputPointCloud::ConstPtr &input_cloud)
{
  if(m_pub_obstacle_cloud.getNumSubscribers() == 0 &&
     m_pub_debug_obstacle_cloud.getNumSubscribers() == 0 &&
     m_pub_filtered_cloud.getNumSubscribers() == 0)
  {
    ROS_DEBUG_STREAM("velodyneCallback: no subscriber to laser_object_detector. resetting buffer");
    resetBuffer();
    return;
  }

  if(!m_buffer_initialized)
    initBuffer(PUCK_NUM_RINGS);
  
  boost::mutex::scoped_lock lock(m_parameter_change_lock);

  for(const auto& point : input_cloud->points)
  {
    m_median_filtered_circ_buffer_vector[point.ring]->push_back(point);
  }
  
  processScan(input_cloud->header);
}

void Detector::processScan(pcl::PCLHeader header)
{
   pcl::StopWatch timer;

   OutputPointCloud::Ptr obstacle_cloud (new OutputPointCloud);
   obstacle_cloud->header = header;

   DebugOutputPointCloud::Ptr debug_obstacle_cloud (new DebugOutputPointCloud);
   debug_obstacle_cloud->header = header;

   DebugOutputPointCloud::Ptr filtered_cloud (new DebugOutputPointCloud);
   filtered_cloud->header = header;
  
   for (auto ring = 0; ring < (int)m_median_filtered_circ_buffer_vector.size(); ++ring)
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
	      detectObstacles(m_median_filtered_circ_buffer_vector.at(ring),
                        *m_detection_iters_by_ring.at(ring),
                        *m_median_iters_by_ring.at(ring),
                        obstacle_cloud,
                        debug_obstacle_cloud);
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
                   neighborhood_values.begin() + small_kernel_end_offset + 1);

  small_kernel_val = neighborhood_values[small_kernel_middle_offset];

  // get median of neighborhood distances with bigger kernel
  std::nth_element(neighborhood_values.begin(), neighborhood_values.begin() + neighborhood_values.size() / 2, neighborhood_values.end());

  big_kernel_val = neighborhood_values[neighborhood_values.size() / 2];
}

void Detector::filterRing(std::shared_ptr<boost::circular_buffer<MedianFiltered> > buffer_median_filtered,
                          median_iterator& iter)
{
   while (!buffer_median_filtered->empty() && iter != buffer_median_filtered->end())
   {
      // compute the kernel size in number of points corresponding to the desired object size
      // in other words, compute how many points are approximately on the object itself
      float alpha = static_cast<float>(std::atan((m_object_size()/2.f)/(*iter).point.distance) * (180.0 / M_PI));
      int kernel_size = (int)std::floor(alpha * 2.f / m_angle_between_scanpoints_launch);
      // the point where the desired object gets filtered out is when there are all points on the object in the kernel
      // and slightly more points that are >not< on the object. Therefore we multiply by two to be roughly at that border.
      kernel_size *= 2;

      if(kernel_size < 0)
         ROS_ERROR("laser_object_detector: filterRing: kernel size negative");

      kernel_size = std::max(kernel_size, 1);
      kernel_size = std::min(kernel_size, m_max_kernel_size);

      int big_kernel_size = (int)std::ceil(kernel_size * m_kernel_size_diff_factor());
      big_kernel_size = std::max(big_kernel_size, 2);

      const int big_kernel_size_half = big_kernel_size / 2;

      if(std::distance(buffer_median_filtered->begin(), iter) >= big_kernel_size_half && std::distance(iter, buffer_median_filtered->end()) > big_kernel_size_half)
      {
         if(m_dist_weight() != 0.f)
            calcMedianFromBuffer(kernel_size, big_kernel_size, buffer_median_filtered, median_const_iterator(iter),
                             [&](const InputPoint &fn) -> float { return fn.distance; },
                             m_max_dist_for_median_computation(),
                             (*iter).dist_small_kernel, (*iter).dist_big_kernel);

         if(m_intensity_weight() != 0.f)
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
      // TODO: adapt the term in the first if statement to the one that was intended in the first place
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
         certainty_value = (m_max_prob_by_distance / (m_median_max_dist() - m_median_thresh2_dist())) * ((m_median_max_dist() - difference_distances) * m_dist_weight()) + difference_intensities * (m_intensity_weight()/m_max_intensity_range);
      }
   }
   certainty_value = std::min(certainty_value, 1.0f);
   certainty_value = std::max(certainty_value, 0.0f);

   return certainty_value;
}

void Detector::detectObstacles(std::shared_ptr<boost::circular_buffer<MedianFiltered> > buffer_median_filtered,
                               median_iterator& median_it,
                               median_iterator& end,
                               OutputPointCloud::Ptr obstacle_cloud,
                               DebugOutputPointCloud::Ptr debug_obstacle_cloud)

{
   // iterator to the first element in the buffer where a computation of the median value was not possible
   // so to say the .end() of median values in the buffer
   median_iterator end_of_values = end + 1;

   float alpha = static_cast<float>(std::atan(m_distance_to_comparison_points()/(*median_it).dist_small_kernel) * 180.f / M_PI);
   int dist_to_comparsion_point = (int)std::round(alpha / m_angle_between_scanpoints_launch);

   dist_to_comparsion_point = std::max(dist_to_comparsion_point, 0);
   dist_to_comparsion_point = std::min(dist_to_comparsion_point, m_max_kernel_size);

   median_iterator::difference_type dist_to_comparsion_point_bounded = dist_to_comparsion_point;

   if(std::distance( buffer_median_filtered->begin(), end_of_values) <= 2*dist_to_comparsion_point_bounded+1 ||
       std::distance( median_it, end_of_values) <= dist_to_comparsion_point_bounded + 1)
   {
      ROS_WARN("laser_object_detector: detectObstacles: not enough medians in buffer");
      return;
   }
   
   for(; median_it != end_of_values; ++median_it)
   {
      // compute index of neighbors to compare to
      alpha = static_cast<float>(std::atan(m_distance_to_comparison_points()/(*median_it).dist_small_kernel) * 180.f / M_PI);
      dist_to_comparsion_point = (int)std::round(alpha / m_angle_between_scanpoints_launch);

      dist_to_comparsion_point = std::max(dist_to_comparsion_point, 0);
      dist_to_comparsion_point = std::min(dist_to_comparsion_point, m_max_kernel_size / 2);

      dist_to_comparsion_point_bounded = dist_to_comparsion_point;

      if(std::distance(median_it, end_of_values) <= dist_to_comparsion_point_bounded + 1)
         break;

      auto window_start = median_it;
      // handle special case for first points in buffer
      if(std::distance(buffer_median_filtered->begin(), median_it) > dist_to_comparsion_point_bounded)
         window_start -= dist_to_comparsion_point_bounded;
      else
         window_start = buffer_median_filtered->begin();

      auto window_end = median_it + dist_to_comparsion_point_bounded;

      // compute differences and resulting certainty value
      float difference_distances = 0.f;
      if(m_dist_weight() != 0.f)
      {
         float difference_distance_start = (*median_it).dist_small_kernel - (*window_start).dist_big_kernel;
         float difference_distance_end = (*median_it).dist_small_kernel - (*window_end).dist_big_kernel;

         float difference_distance_sum = difference_distance_start + difference_distance_end;
         float difference_distance_max = std::max(difference_distance_start, difference_distance_end);
         difference_distances = std::max(difference_distance_sum, difference_distance_max);
      }


      float difference_intensities = 0.f;
      if(m_intensity_weight() != 0.f)
      {
         float difference_intensities_start = (*median_it).intens_small_kernel - (*window_start).intens_big_kernel;
         float difference_intensities_end = (*median_it).intens_small_kernel - (*window_end).intens_big_kernel;

         float difference_intensities_sum = difference_intensities_start + difference_intensities_end;
         float difference_intensities_min = std::min(difference_intensities_start, difference_intensities_end);
         difference_intensities = std::min(difference_intensities_sum, difference_intensities_min);
      }


      float certainty_value = computeCertainty(-difference_distances, difference_intensities);

      if(certainty_value >= m_certainty_threshold())
      {
         const auto& current_point = (*median_it).point;

         OutputPoint output_point;
         output_point.x = current_point.x;
         output_point.y = current_point.y;
         output_point.z = current_point.z;
         output_point.ring = current_point.ring;
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
            debug_output_point.detection_intensity = difference_intensities;
            debug_output_point.detection = certainty_value;
            
            debug_obstacle_cloud->push_back(debug_output_point);
            
            // save factors for median filtered cloud 
            float factor = 1.f;
            if(!std::isnan((*median_it).dist_small_kernel) && !std::isnan(current_point.distance))
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
      m_filtering_factors.clear();
      ROS_ERROR("fillFilteredCloud: cloud and factors have different sizes");
      return;
   }

   std::string laser_frame_id = "/velodyne";
   if(!m_input_is_velodyne)
     laser_frame_id = "/laser_scanner_center";

   tf::StampedTransform laser_link_transform;
   bool transform_found = true;
   try
   {
      m_tf_listener.lookupTransform(laser_frame_id, cloud->header.frame_id, pcl_conversions::fromPCL(cloud->header.stamp), laser_link_transform);
   }
   catch(tf::TransformException& ex)
   {
      ROS_ERROR("Transform unavailable %s", ex.what());
      transform_found = false;
   }

   if(transform_found)
   {
      Eigen::Affine3d laser_link_transform_eigen;
      tf::transformTFToEigen(laser_link_transform, laser_link_transform_eigen);

      DebugOutputPointCloud::Ptr cloud_transformed(new DebugOutputPointCloud);
      pcl::transformPointCloud(*cloud, *cloud_transformed, laser_link_transform_eigen);

      filtered_cloud->header.frame_id = laser_frame_id;

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
   intensity_proportion[2] = m_max_intensity_range * (m_intensity_weight()/100.f);
   intensity_proportion[3] = m_max_prob_by_distance * m_dist_weight() + m_max_intensity_range * (m_intensity_weight()/100.f);
   intensity_proportion[4] = m_max_prob_by_distance * m_dist_weight() + m_max_intensity_range * (m_intensity_weight()/100.f);
   intensity_proportion[5] = m_max_intensity_range * (m_intensity_weight()/100.f);
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

} // namespace laser_object_detector
