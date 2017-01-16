/* -*- mode: C++ -*- */
/** @file

    This class converts a 3D point cloud with height and detection information into a 2D height map

*/

#ifndef _MAPPER_H_
#define _MAPPER_H_ 1

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>


namespace detection_height_mapper
{
   class Mapper
   {
   public:

      Mapper(ros::NodeHandle node, ros::NodeHandle private_nh);
      ~Mapper() {}

      private:

      void callback(uint32_t level);
      void processScan(const sensor_msgs::PointCloud2::ConstPtr &scanMsg);

      //boost::shared_ptr<velodyne_rawdata::RawData> data_;
      ros::Subscriber velodyne_scan_;
      ros::Publisher output_;

      /// configuration parameters
      typedef struct {
         int npackets;                    ///< number of packets to combine
      } Config;
      Config config_;
   };

} // namespace detection_height_mapper

#endif // _MAPPER_H_
