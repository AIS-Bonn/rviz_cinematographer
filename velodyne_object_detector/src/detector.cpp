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
//      plot();

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
      BufferInputPointsPtr points_circ_buffer(new BufferInputPoints(m_circular_buffer_capacity()));
      m_points_circ_buffer_vector.push_back(points_circ_buffer);

      BufferMediansPtr median_filtered_circ_buffer(new BufferMedians(m_circular_buffer_capacity()));
      m_median_filtered_circ_buffer_vector.push_back(median_filtered_circ_buffer);
   }

   // TODO: anpassen
   m_ring_counter = std::vector<int>(PUCK_NUM_RINGS, 0);

//   m_certainty_threshold.setCallback(boost::bind(&Detector::changeParameterSavely, this));
//   m_dist_coeff.setCallback(boost::bind(&Detector::changeParameterSavely, this));
//   m_intensity_coeff.setCallback(boost::bind(&Detector::changeParameterSavely, this));
//   m_weight_for_small_intensities.setCallback(boost::bind(&Detector::changeParameterSavely, this));
//
//   m_object_size.setCallback(boost::bind(&Detector::changeParameterSavely, this));
//   m_circular_buffer_capacity.setCallback(boost::bind(&Detector::resizeBuffers, this));
//   m_distance_to_comparison_points.setCallback(boost::bind(&Detector::changeParameterSavely, this));
//
//   m_median_min_dist.setCallback(boost::bind(&Detector::changeParameterSavely, this));
//   m_median_thresh1_dist.setCallback(boost::bind(&Detector::changeParameterSavely, this));
//   m_median_thresh2_dist.setCallback(boost::bind(&Detector::changeParameterSavely, this));
//   m_median_max_dist.setCallback(boost::bind(&Detector::changeParameterSavely, this));
//
//   m_max_dist_for_median_computation.setCallback(boost::bind(&Detector::changeParameterSavely, this));
}

void Detector::changeParameterSavely()
{
   boost::mutex::scoped_lock lock(m_parameter_change_lock);
   ROS_DEBUG("New parameter");
//   if(m_publish_debug_cloud)
//      plot();
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

void Detector::calcMedianFromBuffer(const int kernel_size,
                             const int big_kernel_size,
			     const BufferInputPointsPtr& buffer,
                             const buffer_const_iterator& current_element,
                             std::function<float(Detector::InputPoint)> f,
                             float max_dist_for_median_computation,
                             float& small_kernel_val, float& big_kernel_val) const
{
  assert(std::distance(buffer->begin(), buffer->end())>big_kernel_size);
  
  const int kernel_size_half = kernel_size / 2;
  const int big_kernel_size_half = big_kernel_size / 2;
  
  buffer_const_iterator small_kernel_start = current_element - kernel_size_half;
  buffer_const_iterator small_kernel_end = current_element + kernel_size_half;
  buffer_const_iterator big_kernel_start = current_element - big_kernel_size_half;
  buffer_const_iterator big_kernel_end = current_element + big_kernel_size_half;

  
  // get distances of neighbors
  std::vector<float> neighborhood_values;
  neighborhood_values.reserve(big_kernel_size);
  std::vector<float> neighborhood_values_small_dist_kernel;
  neighborhood_values_small_dist_kernel.reserve(kernel_size);

  
  // filter if difference of distances of neighbor and the current point exceeds a threshold
  if(m_max_dist_for_median_computation() == 0.f)
  {
    buffer_iterator it_tmp = buffer->begin(); 
    std::advance(it_tmp, std::distance<buffer_const_iterator> ( it_tmp, big_kernel_start));
    while (it_tmp != big_kernel_end)
      neighborhood_values.push_back(f(*it_tmp++));
  }
  else
  {
    // save distance of midpoint in the buffer aka the current point we are looking at
    const float distance_of_current_point = f(*current_element);

    // check for each point in the buffer if it exceeds the distance threshold to the current point
    buffer_iterator it_tmp = buffer->begin(); 
    std::advance(it_tmp, std::distance<buffer_const_iterator> ( it_tmp, big_kernel_start));
    while ( it_tmp != big_kernel_start )
    {
      const float val_tmp = f(*it_tmp);
      const float abs_distance_difference_to_current_point = fabsf(distance_of_current_point - val_tmp);
      
      if(abs_distance_difference_to_current_point < max_dist_for_median_computation)
      {
	neighborhood_values.push_back(val_tmp);
	if(it_tmp >= small_kernel_start && it_tmp != small_kernel_end)
	{
	  neighborhood_values_small_dist_kernel.push_back( val_tmp );
	}
      }
      ++it_tmp;
    }
  }

  // get median of neighborhood distances with smaller kernel
  std::nth_element(neighborhood_values_small_dist_kernel.begin(),
		  neighborhood_values_small_dist_kernel.begin() + neighborhood_values_small_dist_kernel.size() / 2,
		  neighborhood_values_small_dist_kernel.end());

  small_kernel_val = neighborhood_values_small_dist_kernel[neighborhood_values_small_dist_kernel.size() / 2];

   // TODO zuerst small dann big
  // TODO: wieso zwei vektoren?
  
   // get median of neighborhood distances with bigger kernel
  std::nth_element(neighborhood_values.begin(), neighborhood_values.begin() + neighborhood_values.size() / 2, neighborhood_values.end());

  big_kernel_val = neighborhood_values[neighborhood_values.size() / 2];
}

void Detector::velodyneCallback(const InputPointCloud::ConstPtr &input_cloud)
{
//   pcl::StopWatch timer;
//   double start = timer.getTime();

   DebugOutputPointCloud::Ptr debug_obstacle_cloud (new DebugOutputPointCloud);
   debug_obstacle_cloud->header = input_cloud->header;

   OutputPointCloud::Ptr obstacle_cloud (new OutputPointCloud);
   obstacle_cloud->header = input_cloud->header;

   InputPointCloud::Ptr filtered_cloud (new InputPointCloud);
   filtered_cloud->header = input_cloud->header;
   filtered_cloud->header.frame_id = "/velodyne";

   for(const auto& point : input_cloud->points)
   {
      m_points_circ_buffer_vector[point.ring]->push_back(point);
   }

   for (auto ring = 0; ring < PUCK_NUM_RINGS; ++ring)
   {
      filterRing(m_points_circ_buffer_vector.at(ring), m_median_filtered_circ_buffer_vector.at(ring));

      detectObstacles(m_points_circ_buffer_vector.at(ring), m_median_filtered_circ_buffer_vector.at(ring),
                      obstacle_cloud, debug_obstacle_cloud);
      ROS_INFO_STREAM("ring : " << ring << " " << m_points_circ_buffer_vector.at(ring)->size() << " " << m_median_filtered_circ_buffer_vector.at(ring)->size() << " points: " << obstacle_cloud->points.size());
   }


   if(m_publish_debug_cloud)
      m_pub_debug_obstacle_cloud.publish(debug_obstacle_cloud);

   if(m_publish_filtered_cloud)
      m_pub_filtered_cloud.publish(filtered_cloud);

   m_pub_obstacle_cloud.publish(obstacle_cloud);

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

   ROS_INFO_STREAM("buffer: " << buffer->size());
   buffer_iterator it = buffer->begin();
   while (!buffer->empty() && it != buffer->end())
   {
      const float ANGLE_BETWEEN_SCANPOINTS = 0.2f;

      float alpha = static_cast<float>(std::atan((m_object_size()/2.f)/(*it).distance) * 180.f / M_PI);
      const int kernel_size = (int)std::ceil(alpha / ANGLE_BETWEEN_SCANPOINTS) + 1;

      const int kernel_size_half = kernel_size/2;
      const int big_kernel_size = kernel_size*4;
      const int big_kernel_size_half = kernel_size*2;


      // TODO check if kernel_size is valid

      MedianFiltered median_filtered_value;
      
      if ( std::distance(buffer->begin(), it)  > big_kernel_size_half  && std::distance(it, buffer->end()) > big_kernel_size_half )
      {
        calcMedianFromBuffer(kernel_size, big_kernel_size, buffer, buffer_const_iterator(it),
                             [&](const InputPoint &fn) -> float { return fn.distance; },
                             m_max_dist_for_median_computation(),
                             median_filtered_value.dist_small_kernel, median_filtered_value.dist_big_kernel);

        calcMedianFromBuffer(kernel_size, big_kernel_size, buffer, buffer_const_iterator(it),
                             [&](const InputPoint &fn) -> float { return fn.intensity; },
                             0.f, 
			     median_filtered_value.intens_small_kernel, median_filtered_value.intens_big_kernel);
      }
      buffer_median_filtered->push_back(median_filtered_value);
      
      if ( std::distance(it, buffer->end()) > big_kernel_size_half )
      {

         break;
      }

      ++it;
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

void Detector::detectObstacles(std::shared_ptr<boost::circular_buffer<InputPoint> > buffer,
                               std::shared_ptr<boost::circular_buffer<MedianFiltered> > buffer_median_filtered,
                               OutputPointCloud::Ptr obstacle_cloud, DebugOutputPointCloud::Ptr debug_obstacle_cloud)

{
   boost::mutex::scoped_lock lock(m_parameter_change_lock);


   int dist_to_comparsion_point = m_distance_to_comparison_points();

   if (buffer_median_filtered->size() <= 2*dist_to_comparsion_point+1)
   {
      ROS_WARN("not enough medians in buffer");
      return;
   }

   for(auto median_it = buffer_median_filtered->begin() + dist_to_comparsion_point; median_it <  buffer_median_filtered->end()-1 - dist_to_comparsion_point; ++median_it )
   {
      // compute index of neighbors to compare to, take into account that it's a scan ring
      // TODO: convert to a distance in meters
      auto window_start = median_it -dist_to_comparsion_point;
      auto window_end = median_it + dist_to_comparsion_point;

      // compute differences and resulting certainty value
      float difference_distances = -((*median_it).dist_small_kernel * 2.f
                               - (*window_start).dist_big_kernel
                               - (*window_end).dist_big_kernel);

      float difference_intensities = (*median_it).intens_small_kernel * 2.f
                               - (*window_start).intens_big_kernel
                               - (*window_end).intens_big_kernel;

      float certainty_value = computeCertainty(difference_distances, difference_intensities);

      if(certainty_value >= m_certainty_threshold())
      {
	BufferMedians::difference_type offset = std::min<BufferMedians::difference_type>(buffer->size()-1, median_it-buffer_median_filtered->begin());
	
	auto current_point_it = buffer->begin() + offset ;

	OutputPoint output_point;
	output_point.x = (*current_point_it).x;
	output_point.y = (*current_point_it).y;
	output_point.z = (*current_point_it).z;
	output_point.detection = certainty_value;
	obstacle_cloud->push_back(output_point);

	if(m_publish_debug_cloud)
	{
	  DebugOutputPoint debug_output_point;

	  debug_output_point.x = (*current_point_it).x;
	  debug_output_point.y = (*current_point_it).y;
	  debug_output_point.z = (*current_point_it).z;
	  debug_output_point.intensity = (*current_point_it).intensity;
	  debug_output_point.ring = (*current_point_it).ring;

	  debug_output_point.detection_distance = difference_distances;
	  debug_output_point.detection_intensity = difference_intensities;
	  debug_output_point.detection = certainty_value;

	  debug_obstacle_cloud->push_back(debug_output_point);
	}
      }
   }

}
//
//bool Detector::fillCircularBuffer(const InputPointCloud::ConstPtr &cloud,
//                                                       const std::vector<unsigned int> &indices_of_ring,
//                                                       int ring_index)
//{
//   if(!m_distance_median_circ_buffer_vector[ring_index].full())
//   {
//      // for first filling take zeros where there are no values ( + 1 because this one is going to be replaced later )
//      if(m_distance_median_circ_buffer_vector[ring_index].empty())
//      {
//         for(int i = 0; i < (int)m_distance_median_circ_buffer_vector[ring_index].capacity()/2 + 1; i++)
//         {
//            m_distance_median_circ_buffer_vector[ring_index].push_back(0.f);
//            m_intensity_median_circ_buffer_vector[ring_index].push_back(0.f);
//         }
//      }
//
//      // fill rest
//      for(int ring_point_index = 0; ring_point_index < (int)indices_of_ring.size(); ring_point_index++)
//      {
//         int current_cloud_point_index = indices_of_ring[ring_point_index];
//         m_distance_median_circ_buffer_vector[ring_index].push_back(cloud->points[current_cloud_point_index].distance);
//         m_intensity_median_circ_buffer_vector[ring_index].push_back(cloud->points[current_cloud_point_index].intensity);
//         if(m_distance_median_circ_buffer_vector[ring_index].full())
//            break;
//      }
//   }
//
//   // if there were not enough points to fill the buffer, skip this ring for now
//   if(!m_distance_median_circ_buffer_vector[ring_index].full())
//   {
//      m_ring_counter[ring_index] = 0;
//      m_distance_median_circ_buffer_vector[ring_index].clear();
//      return false;
//   }
//
//   return true;
//}
//
//void Detector::fillFilteredCloud(const InputPointCloud::ConstPtr &cloud,
//                                                      InputPointCloud::Ptr filtered_cloud,
//                                                      const std::vector<unsigned int> &indices_of_ring,
//                                                      std::shared_ptr<std::vector<float> > distances_ring_filtered_big_kernel)
//{
//   tf::StampedTransform velodyne_link_transform;
//   bool transform_found = true;
//   try
//   {
//      m_tf_listener.lookupTransform("/velodyne", cloud->header.frame_id, pcl_conversions::fromPCL(cloud->header.stamp), velodyne_link_transform);
//   }
//   catch(tf::TransformException& ex)
//   {
//      ROS_ERROR("Transform unavailable %s", ex.what());
//      transform_found = false;
//   }
//
//   if(transform_found)
//   {
//      Eigen::Affine3d velodyne_link_transform_eigen;
//      tf::transformTFToEigen(velodyne_link_transform, velodyne_link_transform_eigen);
//
//      InputPointCloud::Ptr cloud_transformed(new InputPointCloud);
//      pcl::transformPointCloud(*cloud, *cloud_transformed, velodyne_link_transform_eigen);
//
//      // move the cloud points to the place they would have been if the median filter would have been applied to them
//      int last_index = (int) indices_of_ring.size() - m_median_big_kernel_size / 2;
//      for(int ring_point_index = 0; ring_point_index < last_index; ring_point_index++)
//      {
//         int current_cloud_point_index = indices_of_ring[ring_point_index];
//         float factor = (*distances_ring_filtered_big_kernel)[ring_point_index] /
//                        cloud->points[current_cloud_point_index].distance;
//
//         InputPoint inputPoint;
//         inputPoint.x = cloud_transformed->points[current_cloud_point_index].x * factor;
//         inputPoint.y = cloud_transformed->points[current_cloud_point_index].y * factor;
//         inputPoint.z = cloud_transformed->points[current_cloud_point_index].z * factor;
//         inputPoint.intensity = cloud_transformed->points[current_cloud_point_index].intensity;
//         inputPoint.ring = cloud_transformed->points[current_cloud_point_index].ring;
//         inputPoint.distance = cloud_transformed->points[current_cloud_point_index].distance;
//
//         filtered_cloud->push_back(inputPoint);
//      }
//   }
//}
//
//void Detector::plot()
//{
//   // set up x-axis
//   double epsilon = 0.00000001;
//   const int range = 8;
//   std::vector<double> xAxis(range, 0.0);
//   xAxis[0] = 0.0;
//   xAxis[1] = m_median_min_dist();
//   xAxis[2] = m_median_min_dist() + epsilon;
//   xAxis[3] = m_median_thresh1_dist();
//   xAxis[4] = m_median_thresh2_dist();
//   xAxis[5] = m_median_max_dist();
//   xAxis[6] = m_median_max_dist() + epsilon;
//   xAxis[7] = m_median_max_dist() + 0.5;
//
//   std::vector<double> constant_one(range, 0.0);
//   for(int i = 0; i < range; i++) constant_one[i] = 1.0 + epsilon;
//
//   std::vector<double> distance_proportion(range, 0.0);
//   distance_proportion[0] = 0.0;
//   distance_proportion[1] = 0.0;
//   distance_proportion[2] = 0.0;
//   distance_proportion[3] = m_max_prob_by_distance * m_dist_coeff();
//   distance_proportion[4] = m_max_prob_by_distance * m_dist_coeff();
//   distance_proportion[5] = 0.0;
//   distance_proportion[6] = 0.0;
//   distance_proportion[7] = 0.0;
//
//   std::vector<double> intensity_proportion(range, 0.0);
//   intensity_proportion[0] = 0.0;
//   intensity_proportion[1] = 0.0;
//   intensity_proportion[2] = m_max_intensity_range * m_intensity_coeff();
//   intensity_proportion[3] = m_max_prob_by_distance * m_dist_coeff() + m_max_intensity_range * m_intensity_coeff();
//   intensity_proportion[4] = m_max_prob_by_distance * m_dist_coeff() + m_max_intensity_range * m_intensity_coeff();
//   intensity_proportion[5] = m_max_intensity_range * m_intensity_coeff();
//   intensity_proportion[6] = 0.0;
//   intensity_proportion[7] = 0.0;
//
//   // add histograms to plotter
//   std::vector<char> black{0, 0, 0, (char) 255};
//   std::vector<char> red{(char) 255, 0, 0, (char) 255};
//   std::vector<char> green{0, (char) 255, 0, (char) 255};
//
//   m_plotter->clearPlots();
//
//   m_plotter->setYRange(0.0, std::max(1.1, intensity_proportion[3]));
//   m_plotter->addPlotData(xAxis, distance_proportion, "Distance proportion", vtkChart::LINE, red);
//   m_plotter->addPlotData(xAxis, intensity_proportion, "Intensity proportion", vtkChart::LINE, green);
//   m_plotter->addPlotData(xAxis, constant_one, "One", vtkChart::LINE, black);
//
//   m_plotter->spinOnce(0);
//}

} // namespace velodyne_object_detector
