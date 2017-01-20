/** @file

    This class converts a 3D point cloud with height and detection information into a 2D height map

*/

#include "mapper.h"

namespace detection_height_mapper
{
/** @brief Constructor. */
Mapper::Mapper(ros::NodeHandle node, ros::NodeHandle private_nh)
   : m_height_image_size_x("obstacle_image_size_x", 0, 16, 256, 32)
   , m_height_image_size_y("obstacle_image_size_y", 0, 16, 256, 32)
   , m_height_image_resolution("obstacle_image_resolution", 0.01, 0.01, 0.5, 0.05)
   , m_height_image_min_z("obstacle_image_min_z", -5.f, 0.1f, 5.f, -1.0)
   , m_height_image_max_z("obstacle_image_max_z", -5.f, 0.1f, 5.f, 2.0)
   , m_height_image_min_obstacle_points("obstacle_min_points", 0, 1, 100, 10)
   , m_height_image_obstacle_thresh("obstacle_thresh", 0.0, 0.05, 1.0, 0.9)
   , m_height_image_obstacle_odds_hit("obstacle_odds_hit", 0.0, 0.05, 1.0, 0.7)
   , m_height_image_obstacle_odds_miss("obstacle_odds_miss", 0.0, 0.05, 1.0, 0.35)
   , m_height_image_obstacle_clamp_thresh_min("obstacle_clamp_min", -10.0, 0.1, 10.0, -2.8f)
   , m_height_image_obstacle_clamp_thresh_max("obstacle_clamp_max", -10.0, 0.1, 10.0, 5.8f)
   , m_input_topic("/mrs_laser_mapping/pointcloud")
{
   ROS_INFO_STREAM("detection_height_mapper nodelet init");
   private_nh.getParam("input_topic", m_input_topic);

   // TODO: change topic
   m_cloud_sub = node.subscribe(m_input_topic, 10, &Mapper::callback, this,
                  ros::TransportHints().tcpNoDelay(true));

   m_pub_height_image = node.advertise<sensor_msgs::Image>("height", 1);
   m_pub_height_image_grid = node.advertise<nav_msgs::OccupancyGrid>("obstacle_grid", 1);
}

/** @brief Callback for point clouds. */
void Mapper::callback(const InputPointCloud::ConstPtr &input_cloud)
{
   if(m_pub_height_image.getNumSubscribers() == 0 && m_pub_height_image_grid.getNumSubscribers() == 0)
      return;

   momaro_heightmap::HeightImage height_image;
   height_image.setSize(m_height_image_size_x(), m_height_image_size_y());
   height_image.setResolution(m_height_image_resolution(), m_height_image_resolution());

   InputPointCloud::Ptr obstacle_points(new InputPointCloud());

   // TODO evtl raus
   for(auto& point : input_cloud->points)
   {
      if(point.z > m_height_image_min_z() && point.z < m_height_image_max_z())
      {
         obstacle_points->points.push_back(point);
      }
   }

   Eigen::Affine3f transform = Eigen::Affine3f::Identity();
   transform.translate(Eigen::Vector3f(m_height_image_size_x()/2, m_height_image_size_y()/2, 0.f));
   height_image.processPointcloud(*obstacle_points, transform, m_height_image_obstacle_thresh(), m_height_image_obstacle_odds_hit(), m_height_image_obstacle_odds_miss(), m_height_image_obstacle_clamp_thresh_min(), m_height_image_obstacle_clamp_thresh_max());

   // TODO ab hier weiter
   height_image.detectObstacles(0.2, m_height_image_min_obstacle_points(), m_height_image_obstacle_thresh());

   sensor_msgs::ImagePtr img(new sensor_msgs::Image);
   height_image.fillObstacleColorImage(img.get(), m_height_image_min_z(), m_height_image_max_z(), 0.0, 1.0, 0.2, m_height_image_min_obstacle_points(), m_height_image_obstacle_thresh());

   img->header.frame_id = input_cloud->header.frame_id;
   img->header.stamp = pcl_conversions::fromPCL(input_cloud->header.stamp);
   m_pub_height_image.publish(img);

   nav_msgs::OccupancyGridPtr grid(new nav_msgs::OccupancyGrid);

   height_image.fillObstacleMap(grid.get(), m_height_image_min_z(), m_height_image_max_z(), 100, 0.2, m_height_image_min_obstacle_points(), m_height_image_obstacle_thresh());
   grid->header.frame_id = input_cloud->header.frame_id;
   grid->header.stamp = pcl_conversions::fromPCL(input_cloud->header.stamp);
   m_pub_height_image_grid.publish(grid);
}

} // namespace detection_height_mapper
