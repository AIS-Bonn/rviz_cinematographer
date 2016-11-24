#include <velodyne_object_detector/velodyne_object_detector.h>

namespace velodyne_object_detector
{

VelodyneObjectDetector::VelodyneObjectDetector()
: m_nh("~")
 , PUCK_NUM_RINGS(16)
 , m_certainty_threshold("certainty_threshold", 0.0, 0.1, 4.0, 0.5)
 , m_dist_coeff("dist_coeff", 0.0, 0.1, 10.0, 1.0)
 , m_intensity_coeff("intensity_coeff", 0.0, 0.0001, 0.01, 0.003)
// , m_max_box_width("max_box_width", 0.0, 0.05, 0.9, 0.4)
// , m_min_box_intensity("min_box_intensity", 0, 10, 400, 30)
// , m_max_box_intensity("max_box_intensity", 0, 10, 500, 50)
// , m_min_box_intensity_diff("min_box_intensity_diff", 0, 2, 200, 20)
// , m_intensity_diff_cap("intensity_diff_cap", 0, 2, 150, 20)
 , m_median_min_dist("median_min_dist", 0.0, 0.01, .2, 0.1)
 , m_median_thresh1_dist("median_thresh1_dist", 0.0, 0.05, 0.5, 0.4)
 , m_median_thresh2_dist("median_thresh2_dist", 0.0, 0.1, 2.0, 1.7)
 , m_median_max_dist("median_max_dist", 0.0, 0.5, 3.0, 3.0)
 , m_points_topic("/velodyne_points")
{
   m_nh.getParam("points_topic", m_points_topic);
   m_velodyne_sub = m_nh.subscribe(m_points_topic, 1000, &VelodyneObjectDetector::velodyneCallback, this);

   m_pub = m_nh.advertise<pcl::PointCloud<velodyne_pointcloud::PointXYZIRDetection> >("output", 1);
   m_pub_median = m_nh.advertise<std_msgs::Float32>("median", 1);
   m_pub_cluster_marker = m_nh.advertise<visualization_msgs::Marker>("cluster_marker",0 );

   m_nh.getParam("cluster_distance_threshold", m_cluster_distance_threshold);
   m_nh.getParam("max_internal_distance", m_max_internal_distance);
   m_nh.getParam("cluster_radius_threshold", m_cluster_radius_threshold);
   m_nh.getParam("init_cluster_size", m_init_cluster_size);
   m_nh.getParam("min_cluster_radius", m_min_cluster_radius);
   m_nh.getParam("min_cluster_radius", m_max_cluster_radius);

   m_certainty_threshold.setCallback(boost::bind(&VelodyneObjectDetector::nop, this));
   m_dist_coeff.setCallback(boost::bind(&VelodyneObjectDetector::nop, this));
   m_intensity_coeff.setCallback(boost::bind(&VelodyneObjectDetector::nop, this));
//   m_max_box_width.setCallback(boost::bind(&VelodyneObjectDetector::nop, this));
//   m_min_box_intensity.setCallback(boost::bind(&VelodyneObjectDetector::nop, this));
//   m_max_box_intensity.setCallback(boost::bind(&VelodyneObjectDetector::nop, this));
//   m_min_box_intensity_diff.setCallback(boost::bind(&VelodyneObjectDetector::nop, this));
//   m_intensity_diff_cap.setCallback(boost::bind(&VelodyneObjectDetector::nop, this));

   m_median_min_dist.setCallback(boost::bind(&VelodyneObjectDetector::nop, this));
   m_median_thresh1_dist.setCallback(boost::bind(&VelodyneObjectDetector::nop, this));
   m_median_thresh2_dist.setCallback(boost::bind(&VelodyneObjectDetector::nop, this));
   m_median_max_dist.setCallback(boost::bind(&VelodyneObjectDetector::nop, this));
}

// sort points by ring number and save indices in vector
void VelodyneObjectDetector::splitCloudByRing(PointCloudVelodyne &cloud, std::vector<std::vector<unsigned int> > &clouds_per_ring)
{
   for(unsigned int point_index = 0; point_index < cloud.size(); point_index++)
   {
      clouds_per_ring[cloud.points[point_index].ring].push_back(point_index);
   }
}

// void VelodyneObjectDetector::detectObstacles(pcl::PointCloud<velodyne_pointcloud::PointXYZIR> &cloud,
//                      std::vector<velodyne_pointcloud::PointXYZIR> &currentObstaclesList,
//                      pcl::PointCloud<velodyne_pointcloud::PointXYZIR> &modifiedCloud,
//                      std::map<uint16_t, std::vector<double> > &distanceByPrevious)
// {
//    int order = 0;
//    // a map between a ring and the last point seen so far
//    std::map<uint16_t, velodyne_pointcloud::PointXYZIR> ringPointMap;
// 
//    //std::vector<velodyne_pointcloud::PointXYZIR, Eigen::aligned_allocator<velodyne_pointcloud::PointXYZIR> >::iterator iter = cloud.points.begin();
//    
//    std::vector<uint32_t > index_per_ring(16, 0);
//    // at first detect the obstacle points
//    for(int index = 0; index < cloud.points.size()/10; index++){
//       velodyne_pointcloud::PointXYZIR currentPoint = cloud.points[index];
//       int currentIndex = currentPoint.ring;
// 
//       // Hacky way to exclude points that are not on the ground, for one specific dataset
//       //if(currentIndex > 7) continue;
// 
//       // ???
//       int previousIndex = currentIndex - 1;
//       if(currentIndex == 0) previousIndex = 1;
// 
// 
//       if(ringPointMap.count(previousIndex) > 0){
//          // compute distances from current and previous point to the laser and the expected distance of the current point
//          velodyne_pointcloud::PointXYZIR prevPoint = ringPointMap[previousIndex];
//          double prevNorm = prevPoint.getVector4fMap().norm();
// 
// 
//          // 	    int ring_angle = 75+previousIndex*2;
//          // 	    float angle_to_next = (180-90-ring_angle)*M_PI/180;
//          // 	    float expected_dist = prevPoint.getVector4fMap().norm() / sin(M_PI-(M_PI-angle_to_next)-(2*M_PI/180)) * sin(M_PI-angle_to_next);
// 
// 
//          float height = 0.95f;
//          float delta_angle = (2 * M_PI / 180);
//          float r_delta = height / sin(asin(height / prevNorm) - delta_angle) - prevNorm;
// 
//          float expected_dist = prevNorm + r_delta;
// 
//          double currentTrueNorm = currentPoint.getVector4fMap().norm();
//          double distanceFromPrevious = prevNorm - currentTrueNorm;
//          double scaleFactor = prevNorm;//pow(prevNorm, 2);
//          //             double expected_dist = g_medianFactorByRing[currentIndex]*scaleFactor;
//          //             double difference = distanceFromPrevious - expected_dist;
//          double difference = (expected_dist - currentTrueNorm) / (prevNorm * prevNorm);
//          currentPoint.expected_dist = expected_dist;
//          currentPoint.difference = difference;
//          currentPoint.true_distance = currentTrueNorm;
// 
//          // ???
//          if(currentIndex == 0){
//             difference *= -1;
//          }
// 
//          // use the difference between the actual and expected distance of the current point to detect obstacles
//          if(difference > 0.008){ // this is an obstacle;
//             currentPoint.obstacle = 10000.f;
//             currentObstaclesList.push_back(currentPoint);
//          }
// 
//          currentPoint.order = index_per_ring[currentIndex];
//          index_per_ring[currentIndex] += 1;
// 
//          modifiedCloud.push_back(currentPoint);
// 
//          // save the scaled difference of the current to the previous point BEWARE, the scaleFactor is computed differently than for the "difference"-variable, don't know why
//          distanceByPrevious[currentIndex].push_back(distanceFromPrevious / scaleFactor);
//       }
// 
//       ringPointMap[currentIndex] = currentPoint;
//       // ??? probably does nothing ?
// //      if(currentPoint.ring == 15){
// //         currentPoint.order = order++;
// //      }
//    }
// 
//    for(int ring = 0; ring < index_per_ring.size(); ring++)
//    {
//       std::cout << "points in ring " << ring << " = " << index_per_ring[ring] << std::endl;
//    }
//    std::cout << "all points in cloud " << cloud.points.size() << " per ring should be " << cloud.points.size()/15 << std::endl;
// 
// }

// void VelodyneObjectDetector::detectSegments(PointCloudVelodyne &cloud,
//                                             std::vector<std::vector<unsigned int> > &clouds_per_ring,
//                                             std::vector<std::vector<std::pair<unsigned int, unsigned int> > > &segment_indices_cloud)
// {
//    unsigned int segment_counter = 0;
// 
//    for(unsigned int ring_index = 0; ring_index < clouds_per_ring.size(); ring_index++)
//    {
//       std::vector<std::pair<unsigned int, unsigned int> > segment_indices_ring(0);
//       std::pair<unsigned int, unsigned int> segment_index;
//       segment_index.first = 0;
//       segment_counter = 0;
//       for(unsigned int point_index = 0; point_index < clouds_per_ring[ring_index].size() - 1; point_index++)
//       {
//          unsigned int index_of_first_point = clouds_per_ring[ring_index][point_index];
//          unsigned int index_of_second_point = clouds_per_ring[ring_index][point_index + 1];
// 
//          float dist_first_point = cloud.points[index_of_first_point].getVector3fMap().norm();
//          float dist_second_point = cloud.points[index_of_second_point].getVector3fMap().norm();
//          float normalized_dist = static_cast<float>(fabs((dist_first_point - dist_second_point)/dist_first_point));
// 
//          int intensity_first_point = cloud.points[index_of_first_point].intensity;
//          int intensity_second_point = cloud.points[index_of_second_point].intensity;
//          int abs_intensity_difference = abs(intensity_first_point - intensity_second_point);
//          // cap the intensity difference
//          abs_intensity_difference = std::min(abs_intensity_difference, m_intensity_diff_cap());
// 
//          float certainty_value = normalized_dist * m_dist_coeff() + static_cast<float>(abs_intensity_difference) * m_intensity_coeff();
// 
//          if(certainty_value > m_certainty_threshold())
//          {
//             segment_index.second = point_index;
//             segment_indices_ring.push_back(segment_index);
//             segment_index.first = point_index + 1;
//             segment_counter++;
//          }
// 
//          cloud.points[index_of_first_point].difference = normalized_dist;
//          cloud.points[index_of_first_point].obstacle = certainty_value;
//       }
//       segment_indices_cloud.push_back(segment_indices_ring);
//    }
// }

void VelodyneObjectDetector::medianFilter(std::vector<float> &input,
                                          std::vector<float> &filtered_output,
                                          int kernel_size)
{
   for(int point_index = 0; point_index < input.size(); point_index++)
   {
      int point_index_neighborhood_start = std::max(0, point_index - kernel_size);
      int point_index_neighborhood_end = std::min((int)input.size(), point_index + kernel_size);
      std::vector<float> neighborhood_distances(input.begin() + point_index_neighborhood_start, input.begin() + point_index_neighborhood_end);

      size_t middle = neighborhood_distances.size() / 2;
      std::nth_element(neighborhood_distances.begin(), neighborhood_distances.begin() + middle, neighborhood_distances.end());
      filtered_output[point_index] = neighborhood_distances[middle];
   }
}

void VelodyneObjectDetector::detectSegmentsMedian(PointCloudVelodyne &cloud,
                                            std::vector<std::vector<unsigned int> > &clouds_per_ring,
                                            std::vector<std::vector<std::pair<unsigned int, unsigned int> > > &segment_indices_cloud)
{
   unsigned int segment_counter = 0;

   for(unsigned int ring_index = 0; ring_index < clouds_per_ring.size(); ring_index++)
   {
      std::vector<std::pair<unsigned int, unsigned int> > segment_indices_ring(0);
      std::pair<unsigned int, unsigned int> segment_index;
      segment_index.first = 0;
      segment_counter = 0;

      // median filter on distances
      std::vector<float> distances_ring(clouds_per_ring[ring_index].size(), 0.f);
      for(unsigned int point_index = 0; point_index < clouds_per_ring[ring_index].size(); point_index++)
      {
         unsigned int index_of_point_in_cloud = clouds_per_ring[ring_index][point_index];
         distances_ring[point_index] = cloud.points[index_of_point_in_cloud].getVector3fMap().norm();
      }

      std::vector<float> distances_ring_filtered(distances_ring.size(), 0.f);
      std::vector<float> distances_ring_filtered_more(distances_ring.size(), 0.f);
      medianFilter(distances_ring, distances_ring_filtered, 2);
      // TODO: change this to a kernelsize of ~20cm
      medianFilter(distances_ring, distances_ring_filtered_more, 10);

      // median filter on intensities
      std::vector<float> intensities_ring(clouds_per_ring[ring_index].size(), 0.f);
      for(unsigned int point_index = 0; point_index < clouds_per_ring[ring_index].size(); point_index++)
      {
         unsigned int index_of_point_in_cloud = clouds_per_ring[ring_index][point_index];
         intensities_ring[point_index] = cloud.points[index_of_point_in_cloud].intensity;
      }

      std::vector<float> intensities_ring_filtered(intensities_ring.size(), 0.f);
      std::vector<float> intensities_ring_filtered_more(intensities_ring.size(), 0.f);
      medianFilter(intensities_ring, intensities_ring_filtered, 2);
      // TODO: change this to a kernelsize of ~20cm
      medianFilter(intensities_ring, intensities_ring_filtered_more, 10);

      int distance_to_comparison_points = 10;
      for(unsigned int point_index = distance_to_comparison_points; point_index < clouds_per_ring[ring_index].size() - distance_to_comparison_points; point_index++)
      {
         float certainty_value = 0.f;

         float difference_distances = -(distances_ring_filtered[point_index] * 2.f
                            - distances_ring_filtered_more[point_index - distance_to_comparison_points]
                            - distances_ring_filtered_more[point_index + distance_to_comparison_points]);
         // cap difference to 0 - 1
         float max_diff_dist = m_median_max_dist() + 0.1f;
         difference_distances = std::min(difference_distances, max_diff_dist);
         difference_distances = std::max(difference_distances, 0.0f);

         float difference_intensities = intensities_ring_filtered[point_index] * 2.f
                                        - intensities_ring_filtered_more[point_index - distance_to_comparison_points]
                                        - intensities_ring_filtered_more[point_index + distance_to_comparison_points];
         // cap and shift difference to 0 - 100
         difference_intensities = std::min(difference_intensities, 50.0f);
         difference_intensities = std::max(difference_intensities, -50.0f);
         difference_intensities += 50.0f;

         if(difference_distances < m_median_min_dist() || difference_distances > m_median_max_dist())
         {
            certainty_value = 0.f;
         }
         else{
            if(difference_distances >= m_median_min_dist() && difference_distances < m_median_thresh1_dist())
            {
               certainty_value = difference_distances * m_dist_coeff() * (0.7f/m_median_thresh1_dist()) + difference_intensities * m_intensity_coeff();
            }
            if(difference_distances >= m_median_thresh1_dist() && difference_distances < m_median_thresh2_dist())
            {
               certainty_value = m_dist_coeff() * 0.7f + difference_intensities * m_intensity_coeff();
            }
            if(difference_distances >= m_median_thresh2_dist() && difference_distances < m_median_max_dist())
            {
               certainty_value = (0.7f / (m_median_max_dist() - m_median_thresh2_dist())) * (m_median_max_dist() - difference_distances * m_dist_coeff()) + difference_intensities * m_intensity_coeff();
            }
         }
         certainty_value = std::min(certainty_value, 1.0f);
         certainty_value = std::max(certainty_value, 0.0f);

         unsigned int index_of_current_point = clouds_per_ring[ring_index][point_index];
         cloud.points[index_of_current_point].detection_distance = difference_distances;
         cloud.points[index_of_current_point].detection_intensity = difference_intensities;

//         float certainty_value = -difference_distances * m_dist_coeff() + static_cast<float>(difference_intensities) * m_intensity_coeff();

         if(certainty_value > m_certainty_threshold())
         {
            segment_index.second = point_index;
            segment_indices_ring.push_back(segment_index);
            segment_index.first = point_index + 1;
            segment_counter++;
         }

         cloud.points[index_of_current_point].detection = certainty_value;
      }
      segment_indices_cloud.push_back(segment_indices_ring);
   }
}
//
//void VelodyneObjectDetector::filterSegmentsBySize(PointCloudVelodyne &cloud,
//                                                   std::vector<std::vector<unsigned int> > &clouds_per_ring,
//                                                   std::vector<std::vector<std::pair<unsigned int, unsigned int> > > &segment_indices_cloud,
//                                                   float size_filter)
//{
//   for(unsigned int ring_index = 0; ring_index < segment_indices_cloud.size(); ring_index++)
//   {
//      for(unsigned int point_index = 0; point_index < segment_indices_cloud[ring_index].size(); point_index++)
//      {
//         unsigned int index_of_first_point_in_ring = segment_indices_cloud[ring_index][point_index].first;
//         unsigned int index_of_first_point_in_cloud = clouds_per_ring[ring_index][index_of_first_point_in_ring];
//         unsigned int index_of_last_point_in_ring = segment_indices_cloud[ring_index][point_index].second;
//         unsigned int index_of_last_point_in_cloud = clouds_per_ring[ring_index][index_of_last_point_in_ring];
//
//         auto first_point_eigen = cloud.points[index_of_first_point_in_cloud].getVector3fMap();
//         auto last_point_eigen = cloud.points[index_of_last_point_in_cloud].getVector3fMap();
//         auto difference_of_points = first_point_eigen - last_point_eigen;
//
//         if(difference_of_points.norm() < m_max_box_width())
//         {
////            // compare intensity of segment to intensity near the segment
////            int segment_intensity_sum = 0;
////            for(unsigned int counter = index_of_first_point_in_ring; counter <= index_of_last_point_in_ring; counter++)
////            {
////               unsigned int index_of_point_in_cloud = clouds_per_ring[ring_index][counter];
////               segment_intensity_sum += cloud.points[index_of_point_in_cloud].intensity;
////            }
////            if((index_of_last_point_in_ring - index_of_first_point_in_ring) == 0) continue;
////            int segment_intensity_avrg = segment_intensity_sum / (index_of_last_point_in_ring - index_of_first_point_in_ring);
////
////            int neighborhood_width = 3;
////            int outer_segment_intensity_sum = 0;
////            int neighbor_counter = 0;
////            for(unsigned int counter = 1; counter <= neighborhood_width; counter++)
////            {
////               if(index_of_first_point_in_ring >= counter)
////               {
////                  unsigned int index_of_point_in_cloud = clouds_per_ring[ring_index][index_of_first_point_in_ring - counter];
////                  outer_segment_intensity_sum += cloud.points[index_of_point_in_cloud].intensity;
////                  neighbor_counter++;
////               }
////               if((index_of_last_point_in_ring + counter) < clouds_per_ring[ring_index].size())
////               {
////                  unsigned int index_of_point_in_cloud = clouds_per_ring[ring_index][index_of_last_point_in_ring + counter];
////                  outer_segment_intensity_sum += cloud.points[index_of_point_in_cloud].intensity;
////                  neighbor_counter++;
////               }
////            }
////            if(neighbor_counter == 0) continue;
////            int outer_segment_intensity_avrg = outer_segment_intensity_sum / neighbor_counter;
////
////            int abs_avrg_intensity_difference = abs(segment_intensity_avrg - outer_segment_intensity_avrg);
////
////            //ROS_INFO_STREAM("I was here ##################################################" << abs_avrg_intensity_difference);
////
////            if(abs_avrg_intensity_difference > m_min_box_intensity_diff())
////            {
//            for(unsigned int counter = index_of_first_point_in_ring; counter <= index_of_last_point_in_ring; counter++)
//            {
//               unsigned int index_of_point_in_cloud = clouds_per_ring[ring_index][counter];
////                cloud.points[index_of_point_in_cloud].order = 10000;
//            }
////            }
//         }
//      }
//   }
//}

void VelodyneObjectDetector::velodyneCallback(const PointCloudVelodyne& input_cloud)
{
   ROS_INFO_STREAM("callback with thresh " << m_certainty_threshold());
   PointCloudVelodyne cloud = input_cloud;

   // save indices of points in one ring in one vector
   // and each vector representing a ring in another vector containing all indices of the cloud
   std::vector<std::vector<unsigned int> > clouds_per_ring(PUCK_NUM_RINGS, std::vector<unsigned int>(0));
   splitCloudByRing(cloud, clouds_per_ring);

   // save first and last index of a segment within a ring in a pair
   // all segments of a ring in a vector
   // and all rings in another vector
   std::vector<std::vector<std::pair<unsigned int, unsigned int> > > segment_indices_cloud(0, std::vector<std::pair<unsigned int, unsigned int> >(0));
//   detectSegments(cloud, clouds_per_ring, segment_indices_cloud);
   detectSegmentsMedian(cloud, clouds_per_ring, segment_indices_cloud);

   // check for each segment if it exceeds the width of a box
//   filterSegmentsBySize(cloud, clouds_per_ring, segment_indices_cloud, m_max_box_width());

   // compute angle between two consecutive points in one ring
//   for(int height_index = 0; height_index < clouds_per_ring.size(); height_index++)
//   {
//      for (int width_index = 0; width_index < clouds_per_ring[height_index].size() - 1; width_index++)
//      {
//         Eigen::Vector3f first_point(clouds_per_ring[height_index].points[width_index].getVector3fMap());
//         Eigen::Vector3f second_point(clouds_per_ring[height_index].points[width_index+1].getVector3fMap());
//         Eigen::Vector3f cross_prod = first_point.cross(second_point);
//         float temp1 = cross_prod.norm();
//         float temp2 = first_point.dot(second_point);
//         double angle = atan2(temp1, temp2);
//
//         PointVelodyne point_velodyne = clouds_per_ring[height_index].points[width_index];
//         point_velodyne.obstacle = angle;
//         modifiedCloud.push_back(point_velodyne);
//      }
//   }

   m_pub.publish(cloud);
}

}

int main(int argc, char **argv)
{
   ros::init(argc, argv, "velodyne_object_detector");

   velodyne_object_detector::VelodyneObjectDetector object_detector;

   ros::spin();

   return 0;
}