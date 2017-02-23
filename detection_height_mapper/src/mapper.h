/* -*- mode: C++ -*- */
/** @file

    This class converts a 3D point cloud with height and detection information into a 2D height map

*/

#ifndef _MAPPER_H_
#define _MAPPER_H_ 1

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>

#include <tf_conversions/tf_eigen.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <pcl/filters/passthrough.h>

#include <pcl_conversions/pcl_conversions.h>

#include <visualization_msgs/Marker.h>

#include <config_server/parameter.h>

#include <nav_msgs/OccupancyGrid.h>

#include <velodyne_object_detector/point_type.h>

#include <mrs_laser_maps/map_point_types.h>

#include <detection_height_mapper/ObjectPosition.h>

#include "height_image.h"

namespace detection_height_mapper
{
   class Mapper
   {
   public:
      Mapper(ros::NodeHandle node, ros::NodeHandle private_nh);
      ~Mapper() {}

   private:
      void callback(const detection_height_image::HeightImage::InputPointCloud::ConstPtr &input_cloud);

      bool geofencing(const detection_height_image::HeightImage::InputPointCloud::ConstPtr input_cloud,
                      detection_height_image::HeightImage::InputPointCloud::Ptr output_cloud,
                      std::string target_frame,
                      float min_x, float max_x, float min_y, float max_y);

      void publishTransformedObjectPositions(std::vector<detection_height_mapper::ObjectPosition>& object_positions,
                                             std::string target_frame,
                                             std::string source_frame,
                                             ros::Time stamp);

      void filterObjects(detection_height_mapper::ObjectPosition& new_object_position);

      bool transformObjectPositions(std::vector<detection_height_mapper::ObjectPosition>& object_positions,
                                    std::string target_frame,
                                    std::string source_frame,
                                    ros::Time stamp);

      ros::Subscriber m_cloud_sub;

      ros::Publisher m_pub_height_image;
      ros::Publisher m_pub_height_image_grid;
      ros::Publisher m_pub_object_positions;
      ros::Publisher m_pub_object_positions_with_info;

      tf::TransformListener m_tf_listener;

      config_server::Parameter<int> m_height_image_size_x;
      config_server::Parameter<int> m_height_image_size_y;
      config_server::Parameter<float> m_height_image_resolution;
      config_server::Parameter<float> m_geofencing_min_x;
      config_server::Parameter<float> m_geofencing_max_x;
      config_server::Parameter<float> m_geofencing_min_y;
      config_server::Parameter<float> m_geofencing_max_y;
      config_server::Parameter<float> m_height_image_min_z;
      config_server::Parameter<float> m_height_image_max_z;
      config_server::Parameter<int> m_min_object_points_per_cell;
      config_server::Parameter<int> m_min_object_scans_per_cell;
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
      config_server::Parameter<float> m_object_max_neighborhood_height_param;
      config_server::Parameter<float> m_object_inflation_radius_param;
      config_server::Parameter<float> m_object_robot_radius_param;
      config_server::Parameter<float> m_max_position_noise_param;
      config_server::Parameter<bool> m_inflate_objects;

      std::vector<detection_height_mapper::ObjectPosition> m_object_positions_accumulated;

      std::string m_input_topic;
      std::string m_fixed_frame;
   };

} // namespace detection_height_mapper

#endif // _MAPPER_H_
