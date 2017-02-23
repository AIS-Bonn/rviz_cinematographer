/** @file

    This class converts a 3D point cloud with height and detection information into a 2D height map

*/

#include "mapper.h"

namespace detection_height_mapper
{
/** @brief Constructor. */
Mapper::Mapper(ros::NodeHandle node, ros::NodeHandle private_nh)
   : m_height_image_size_x("detection_height_mapper/image_size_x", 0, 16, 256, 100)
   , m_height_image_size_y("detection_height_mapper/image_size_y", 0, 16, 256, 100)
   , m_height_image_resolution("detection_height_mapper/image_resolution", 0.01, 0.01, 0.5, 0.1)
   , m_geofencing_min_x("detection_height_mapper/geofencing_min_x", -100.0, 0.5, 100.0, -100.0)
   , m_geofencing_max_x("detection_height_mapper/geofencing_max_x", -100.0, 0.5, 100.0, 100.0)
   , m_geofencing_min_y("detection_height_mapper/geofencing_min_y", -100.0, 0.5, 100.0, -100.0)
   , m_geofencing_max_y("detection_height_mapper/geofencing_max_y", -100.0, 0.5, 100.0, 100.0)
   , m_height_image_min_z("detection_height_mapper/image_min_z", -20.f, 0.1f, 20.f, -20.0)
   , m_height_image_max_z("detection_height_mapper/image_max_z", -20.f, 0.1f, 20.f, 20.0)
   , m_min_object_points_per_cell("detection_height_mapper/object_min_points", 0, 1, 100, 10)
   , m_min_object_scans_per_cell("detection_height_mapper/object_min_scans", 0, 1, 100, 3)
   , m_object_detection_threshold("detection_height_mapper/object_thresh", 0.0, 0.05, 1.0, 0.5)
   , m_object_odds_hit("detection_height_mapper/object_odds_hit", 0.0, 0.05, 1.0, 0.7)
   , m_object_odds_miss("detection_height_mapper/object_odds_miss", 0.0, 0.05, 1.0, 0.35)
   , m_object_clamp_thresh_min("detection_height_mapper/object_clamp_min", -10.0, 0.1, 10.0, -2.8f)
   , m_object_clamp_thresh_max("detection_height_mapper/object_clamp_max", -10.0, 0.1, 10.0, 5.8f)
   , m_object_min_height_param("detection_height_mapper/object_min_height", 0.0, 0.01, 0.5, 0.15)
   , m_object_max_height_param("detection_height_mapper/object_max_height", 0.0, 0.01, 1.0, 0.3)
   , m_object_min_footprint_param("detection_height_mapper/object_min_footprint_in_sqm", 0.0, 0.01, 0.1, 0.05)
   , m_object_max_footprint_param("detection_height_mapper/object_max_footprint_in_sqm", 0.0, 0.01, 1.0, 0.25)
   , m_object_max_altitude_param("detection_height_mapper/object_max_altitude", 0.0, 0.1, 1.0, 0.5)
   , m_object_max_neighborhood_height_param("detection_height_mapper/object_max_neighborhood_height", 0.0, 0.1, 10.0, 0.4)
   , m_object_inflation_radius_param("detection_height_mapper/object_inflation_radius", 0.0, 0.05, 2.0, 0.25)
   , m_object_robot_radius_param("detection_height_mapper/object_robot_radius", 0.0, 0.1, 10.0, 1.0)
   , m_max_position_noise_param("detection_height_mapper/max_position_noise", 0.0, 0.1, 10.0, 1.0)
   , m_inflate_objects("detection_height_mapper/inflate_objects", true)
   , m_debug_mode("detection_height_mapper/debug_mode", 0, 1, 8, 0)
   , m_input_topic("/mrs_laser_mapping/pointcloud")
   , m_fixed_frame("/world_corrected")
{
   ROS_INFO_STREAM("detection_height_mapper nodelet init");
   private_nh.getParam("input_topic", m_input_topic);
   private_nh.getParam("fixed_frame", m_fixed_frame);

   float geofencing_min_x;
   if(private_nh.getParam("geofencing_min_x", geofencing_min_x))
      m_geofencing_min_x.set(geofencing_min_x);

   float geofencing_max_x;
   if(private_nh.getParam("geofencing_max_x", geofencing_max_x))
      m_geofencing_max_x.set(geofencing_max_x);

   float geofencing_min_y;
   if(private_nh.getParam("geofencing_min_y", geofencing_min_y))
      m_geofencing_min_y.set(geofencing_min_y);

   float geofencing_max_y;
   if(private_nh.getParam("geofencing_max_y", geofencing_max_y))
      m_geofencing_max_y.set(geofencing_max_y);

   float object_min_height;
   if(private_nh.getParam("object_min_height", object_min_height))
      m_object_min_height_param.set(object_min_height);

   float object_max_height;
   if(private_nh.getParam("object_max_height", object_max_height))
      m_object_max_height_param.set(object_max_height);

   float object_min_footprint_in_sqm;
   if(private_nh.getParam("object_min_footprint_in_sqm", object_min_footprint_in_sqm))
      m_object_min_footprint_param.set(object_min_footprint_in_sqm);

   float object_max_footprint_in_sqm;
   if(private_nh.getParam("object_max_footprint_in_sqm", object_max_footprint_in_sqm))
      m_object_max_footprint_param.set(object_max_footprint_in_sqm);

   float object_max_altitude;
   if(private_nh.getParam("object_max_altitude", object_max_altitude))
      m_object_max_altitude_param.set(object_max_altitude);

   float object_max_neighborhood_height;
   if(private_nh.getParam("object_max_neighborhood_height", object_max_neighborhood_height))
      m_object_max_neighborhood_height_param.set(object_max_neighborhood_height);

   float object_inflation_radius;
   if(private_nh.getParam("object_inflation_radius", object_inflation_radius))
      m_object_inflation_radius_param.set(object_inflation_radius);

   float object_robot_radius;
   if(private_nh.getParam("object_robot_radius", object_robot_radius))
      m_object_robot_radius_param.set(object_robot_radius);

   m_cloud_sub = node.subscribe(m_input_topic, 10, &Mapper::callback, this,
                  ros::TransportHints().tcpNoDelay(true));

   m_pub_height_image = node.advertise<sensor_msgs::Image>("height_and_detections", 1);
   m_pub_height_image_grid = node.advertise<nav_msgs::OccupancyGrid>("object_grid", 1);
   m_pub_object_positions = node.advertise<detection_height_mapper::ObjectPosition::_position_type>("object_positions", 1);
   m_pub_object_positions_with_info = node.advertise<detection_height_mapper::ObjectPosition>("object_positions_with_info", 1);

   m_height_image_size_x.setCallback(boost::bind(&Mapper::adaptFenceToImage, this));
   m_height_image_size_y.setCallback(boost::bind(&Mapper::adaptFenceToImage, this));
}

void Mapper::adaptFenceToImage()
{
   if(m_height_image_size_x() / 2.f != fabsf(m_geofencing_min_x()) || m_height_image_size_x() / 2.f != fabsf(m_geofencing_max_x()))
   {
      m_geofencing_min_x.set(-m_height_image_size_x() / 2.f);
      m_geofencing_max_x.set(m_height_image_size_x() / 2.f);
   }

   if(m_height_image_size_y() / 2.f != fabsf(m_geofencing_min_y()) || m_height_image_size_y() / 2.f != fabsf(m_geofencing_max_y()))
   {
      m_geofencing_min_y.set(-m_height_image_size_y() / 2.f);
      m_geofencing_max_y.set(m_height_image_size_y() / 2.f);
   }
}

/** @brief Callback for point clouds. */
void Mapper::callback(const detection_height_image::HeightImage::InputPointCloud::ConstPtr &input_cloud)
{
   if(m_pub_height_image.getNumSubscribers() == 0 && m_pub_height_image_grid.getNumSubscribers() == 0)
      return;

   ros::Time start = ros::Time::now();

   detection_height_image::HeightImage::InputPointCloud::Ptr cloud_filtered(new detection_height_image::HeightImage::InputPointCloud);

   if(!geofencing(input_cloud, cloud_filtered, m_fixed_frame, m_geofencing_min_x(), m_geofencing_max_x(), m_geofencing_min_y(), m_geofencing_max_y()))
      return;

   detection_height_image::HeightImage height_image;
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
   height_image.setMaxNeighborhoodHeight(m_object_max_neighborhood_height_param());
   height_image.setInflationRadius(m_object_inflation_radius_param());
   height_image.setRobotRadius(m_object_robot_radius_param());
   height_image.setDebug(m_debug_mode());

   Eigen::Affine3f transform = Eigen::Affine3f::Identity();
   transform.translate(Eigen::Vector3f(m_height_image_size_x()/2, m_height_image_size_y()/2, 0.f));
   height_image.processPointcloud(*cloud_filtered, transform, m_object_odds_hit(), m_object_odds_miss(), m_object_clamp_thresh_min(), m_object_clamp_thresh_max());

   height_image.detectObjects(m_min_object_points_per_cell(), m_min_object_scans_per_cell(), m_inflate_objects());

   sensor_msgs::ImagePtr img(new sensor_msgs::Image);
   height_image.fillObjectColorImage(img);
   img->header.frame_id = cloud_filtered->header.frame_id;
   img->header.stamp = pcl_conversions::fromPCL(cloud_filtered->header.stamp);
   m_pub_height_image.publish(img);

   nav_msgs::OccupancyGridPtr grid(new nav_msgs::OccupancyGrid);
   height_image.fillObjectMap(grid.get());
   grid->header.frame_id = cloud_filtered->header.frame_id;
   grid->header.stamp = pcl_conversions::fromPCL(cloud_filtered->header.stamp);
   m_pub_height_image_grid.publish(grid);

   // get object positions from height image and filter those that are already known
   std::vector<detection_height_mapper::ObjectPosition> object_positions;
   height_image.getObjectPositions(object_positions, transform.inverse());
   std::string source_frame = cloud_filtered->header.frame_id;
   std::string target_frame = m_fixed_frame;
   publishTransformedObjectPositions(object_positions, target_frame, source_frame, pcl_conversions::fromPCL(cloud_filtered->header.stamp));

   ros::Duration exec_time = ros::Time::now() - start;
   ROS_DEBUG_STREAM("Detection Height Image: Handling one pointcloud took " << exec_time.toSec() << " seconds.");
}

bool Mapper::geofencing(const detection_height_image::HeightImage::InputPointCloud::ConstPtr input_cloud,
                        detection_height_image::HeightImage::InputPointCloud::Ptr output_cloud,
                        std::string target_frame,
                        float min_x, float max_x, float min_y, float max_y)
{
   tf::StampedTransform transform;
   try
   {
      m_tf_listener.lookupTransform(target_frame, input_cloud->header.frame_id, pcl_conversions::fromPCL(input_cloud->header.stamp), transform);
   }
   catch(tf::TransformException& ex)
   {
      ROS_ERROR("Mapper::geofencing: Transform unavailable %s", ex.what());
      return false;
   }

   Eigen::Affine3d transform_eigen;
   tf::transformTFToEigen(transform, transform_eigen);

   detection_height_image::HeightImage::InputPointCloud::Ptr cloud_transformed(new detection_height_image::HeightImage::InputPointCloud);
   pcl::transformPointCloud(*input_cloud, *cloud_transformed, transform_eigen);

   for(const auto& point : cloud_transformed->points)
   {
      if(point.x > min_x && point.x < max_x &&
         point.y > min_y && point.y < max_y)
      {
         output_cloud->push_back(point);
      }
   }

   output_cloud->header.frame_id = target_frame;
   output_cloud->header.stamp = input_cloud->header.stamp;

   return true;
}

void Mapper::publishTransformedObjectPositions(std::vector<detection_height_mapper::ObjectPosition>& object_positions,
                                               std::string target_frame,
                                               std::string source_frame,
                                               ros::Time stamp)
{
   if(!transformObjectPositions(object_positions, target_frame, source_frame, stamp))
      return;

   for(auto& object_position: object_positions)
   {
      filterObjects(object_position);
   }

   for(auto& object_position: m_object_positions_accumulated)
   {

      ROS_DEBUG_STREAM("object " << object_position.id << " x= " << object_position.position.point.x
                                 << " y= " << object_position.position.point.y
                                 << " z= " << object_position.position.point.z);

      object_position.position.header.frame_id = target_frame;
      object_position.position.header.stamp = stamp;
      m_pub_object_positions.publish(object_position.position);
      m_pub_object_positions_with_info.publish(object_position);
   }
}

bool Mapper::transformObjectPositions(std::vector<detection_height_mapper::ObjectPosition>& object_positions,
                                      std::string target_frame,
                                      std::string source_frame,
                                      ros::Time stamp)
{
   tf::StampedTransform transform;
   try
   {
      m_tf_listener.lookupTransform(target_frame, source_frame, stamp, transform);
   }
   catch(tf::TransformException& ex)
   {
      ROS_ERROR("Mapper::transformObjectPositions: Transform unavailable %s", ex.what());
      return false;
   }

   Eigen::Affine3d transform_eigen;
   tf::transformTFToEigen(transform, transform_eigen);

   for(auto& object_position: object_positions)
   {
      Eigen::Vector3d pos;
      pos.x() = object_position.position.point.x;
      pos.y() = object_position.position.point.y;
      pos.z() = object_position.position.point.z;
      pos = transform_eigen * pos;
      object_position.position.point.x = pos.x();
      object_position.position.point.y = pos.y();
      object_position.position.point.z = pos.z();
   }

   return true;
}

void Mapper::filterObjects(detection_height_mapper::ObjectPosition& new_object_position)
{
   bool new_object = true;
   // compare distances of new object to old objects
   for(unsigned int i = 0; i < m_object_positions_accumulated.size(); i++)
   {
      float distance = static_cast<float>(hypot(new_object_position.position.point.x - m_object_positions_accumulated[i].position.point.x,
                                                new_object_position.position.point.y - m_object_positions_accumulated[i].position.point.y));

      if(distance < m_max_position_noise_param())
      {
         new_object = false;
         break;
      }
   }

   // add new object if its not close to any old object
   if(new_object)
   {
      new_object_position.id = m_object_positions_accumulated.size();
      m_object_positions_accumulated.push_back(new_object_position);
   }
}

} // namespace detection_height_mapper
