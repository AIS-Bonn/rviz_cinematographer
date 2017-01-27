/** @file

    This class converts a 3D point cloud with height and detection information into a 2D height map

*/

#include "mapper.h"

namespace detection_height_mapper
{
/** @brief Constructor. */
Mapper::Mapper(ros::NodeHandle node, ros::NodeHandle private_nh)
   : m_height_image_size_x("detection_height_mapper/object_image_size_x", 0, 16, 256, 32)
   , m_height_image_size_y("detection_height_mapper/object_image_size_y", 0, 16, 256, 32)
   , m_height_image_resolution("detection_height_mapper/object_image_resolution", 0.01, 0.01, 0.5, 0.05)
   , m_height_image_min_z("detection_height_mapper/object_image_min_z", -5.f, 0.1f, 5.f, -1.0)
   , m_height_image_max_z("detection_height_mapper/object_image_max_z", -5.f, 0.1f, 5.f, 2.0)
   , m_min_object_points_per_cell("detection_height_mapper/object_min_points", 0, 1, 100, 10)
   , m_object_detection_threshold("detection_height_mapper/object_thresh", 0.0, 0.05, 1.0, 0.9)
   , m_object_odds_hit("detection_height_mapper/object_odds_hit", 0.0, 0.05, 1.0, 0.7)
   , m_object_odds_miss("detection_height_mapper/object_odds_miss", 0.0, 0.05, 1.0, 0.35)
   , m_object_clamp_thresh_min("detection_height_mapper/object_clamp_min", -10.0, 0.1, 10.0, -2.8f)
   , m_object_clamp_thresh_max("detection_height_mapper/object_clamp_max", -10.0, 0.1, 10.0, 5.8f)
   , m_object_min_height_param("detection_height_mapper/object_min_height", 0.0, 0.01, 0.5, 0.1)
   , m_object_max_height_param("detection_height_mapper/object_max_height", 0.0, 0.01, 1.0, 0.3)
   , m_object_min_footprint_param("detection_height_mapper/object_min_footprint_in_sqm", 0.0, 0.01, 0.1, 0.05)
   , m_object_max_footprint_param("detection_height_mapper/object_max_footprint_in_sqm", 0.0, 0.01, 1.0, 0.25)
   , m_object_max_altitude_param("detection_height_mapper/object_max_altitude", 0.0, 0.1, 1.0, 0.5)
   , m_inflate_objects("detection_height_mapper/inflate_objects", true)
   , m_object_min_height(0.05f)
   , m_object_max_height(0.25f)
   , m_object_max_altitude(0.05f)
   , m_input_topic("/mrs_laser_mapping/pointcloud")
{
   ROS_INFO_STREAM("detection_height_mapper nodelet init");
   private_nh.getParam("input_topic", m_input_topic);
   private_nh.getParam("object_min_height", m_object_min_height);
   private_nh.getParam("object_max_height", m_object_max_height);
   private_nh.getParam("object_min_footprint_in_sqm", m_object_min_footprint_in_sqm);
   private_nh.getParam("object_max_footprint_in_sqm", m_object_max_footprint_in_sqm);
   private_nh.getParam("object_max_altitude", m_object_max_altitude);
   m_object_min_height_param.set(m_object_min_height);
   m_object_max_height_param.set(m_object_max_height);
   m_object_min_footprint_param.set(m_object_min_footprint_in_sqm);
   m_object_max_footprint_param.set(m_object_max_footprint_in_sqm);
   m_object_max_altitude_param.set(m_object_max_altitude);

   m_cloud_sub = node.subscribe(m_input_topic, 10, &Mapper::callback, this,
                  ros::TransportHints().tcpNoDelay(true));

   m_pub_height_image = node.advertise<sensor_msgs::Image>("height_and_detections", 1);
   m_pub_height_image_grid = node.advertise<nav_msgs::OccupancyGrid>("object_grid", 1);
}

/** @brief Callback for point clouds. */
void Mapper::callback(const InputPointCloud::ConstPtr &input_cloud)
{
   if(m_pub_height_image.getNumSubscribers() == 0 && m_pub_height_image_grid.getNumSubscribers() == 0)
      return;

   momaro_heightmap::HeightImage height_image;
   height_image.setSize(m_height_image_size_x(), m_height_image_size_y());
   height_image.setResolution(m_height_image_resolution(), m_height_image_resolution());
   height_image.setMinHeight(m_height_image_min_z());
   height_image.setMaxHeight(m_height_image_max_z());
   height_image.setMinObjectHeight(m_object_min_height_param());
   height_image.setMaxObjectHeight(m_object_max_height_param());
   height_image.setMinObjectFootprint(m_object_min_footprint_param());
   height_image.setMaxObjectFootprint(m_object_max_footprint_param());
   height_image.setMaxObjectAltitude(m_object_max_altitude_param());
   height_image.setDetectionThreshold(m_object_detection_threshold());

   Eigen::Affine3f transform = Eigen::Affine3f::Identity();
   transform.translate(Eigen::Vector3f(m_height_image_size_x()/2, m_height_image_size_y()/2, 0.f));
   height_image.processPointcloud(*input_cloud, transform, m_object_odds_hit(), m_object_odds_miss(), m_object_clamp_thresh_min(), m_object_clamp_thresh_max());

   height_image.detectObjects(m_min_object_points_per_cell(), m_inflate_objects());

   sensor_msgs::ImagePtr img(new sensor_msgs::Image);
   height_image.fillObjectColorImage(img.get());
   img->header.frame_id = input_cloud->header.frame_id;
   img->header.stamp = pcl_conversions::fromPCL(input_cloud->header.stamp);
   m_pub_height_image.publish(img);

   nav_msgs::OccupancyGridPtr grid(new nav_msgs::OccupancyGrid);
   height_image.fillObjectMap(grid.get());
   grid->header.frame_id = input_cloud->header.frame_id;
   grid->header.stamp = pcl_conversions::fromPCL(input_cloud->header.stamp);
   m_pub_height_image_grid.publish(grid);
}

} // namespace detection_height_mapper
