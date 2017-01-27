/* -*- mode: C++ -*- */
/** @file

    This class converts a 3D point cloud with height and detection information into a 2D height map

*/

#ifndef _MAPPER_H_
#define _MAPPER_H_ 1

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <pcl/filters/passthrough.h>

#include <pcl_conversions/pcl_conversions.h>

#include <visualization_msgs/Marker.h>

#include <config_server/parameter.h>

#include <nav_msgs/OccupancyGrid.h>

#include <velodyne_object_detector/point_type.h>

#include "height_image.h"

namespace detection_height_mapper
{
   class Mapper
   {
   public:
      typedef velodyne_pointcloud::PointXYZDetection  PointWithDetection;

      typedef PointWithDetection                      InputPoint;
      typedef pcl::PointCloud<InputPoint>             InputPointCloud;

      Mapper(ros::NodeHandle node, ros::NodeHandle private_nh);
      ~Mapper() {}

   private:
      void callback(const InputPointCloud::ConstPtr &input_cloud);

      ros::Subscriber m_cloud_sub;

      ros::Publisher m_pub_height_image;
      ros::Publisher m_pub_height_image_grid;

      config_server::Parameter<int> m_height_image_size_x;
      config_server::Parameter<int> m_height_image_size_y;
      config_server::Parameter<float> m_height_image_resolution;
      config_server::Parameter<float> m_height_image_min_z;
      config_server::Parameter<float> m_height_image_max_z;
      config_server::Parameter<int> m_min_object_points_per_cell;
      config_server::Parameter<float> m_object_detection_threshold;
      config_server::Parameter<float> m_object_odds_hit;
      config_server::Parameter<float> m_object_odds_miss;
      config_server::Parameter<float> m_object_clamp_thresh_min;
      config_server::Parameter<float> m_object_clamp_thresh_max;
      config_server::Parameter<float> m_object_min_height_param;
      config_server::Parameter<float> m_object_max_height_param;
      config_server::Parameter<float> m_object_min_footprint_param;
      config_server::Parameter<float> m_object_max_footprint_param;
      config_server::Parameter<float> m_object_max_altitude_param;
      config_server::Parameter<bool> m_inflate_objects;

      float m_object_min_height;
      float m_object_max_height;
      float m_object_min_footprint_in_sqm;
      float m_object_max_footprint_in_sqm;
      float m_object_max_altitude;

      std::string m_input_topic;
   };

} // namespace detection_height_mapper

#endif // _MAPPER_H_
