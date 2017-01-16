/** @file

    This class converts a 3D point cloud with height and detection information into a 2D height map

*/

#include "mapper.h"

#include <pcl_conversions/pcl_conversions.h>

namespace detection_height_mapper
{
  /** @brief Constructor. */
  Mapper::Mapper(ros::NodeHandle node, ros::NodeHandle private_nh)//:
    //data_(new velodyne_rawdata::RawData())
  {
    //data_->setup(private_nh);


    // advertise output point cloud (before subscribing to input data)
    output_ =
      node.advertise<sensor_msgs::PointCloud2>("velodyne_points", 10);
      
    //srv_ = boost::make_shared <dynamic_reconfigure::Server<velodyne_pointcloud::
    //  CloudNodeConfig> > (private_nh);
    //dynamic_reconfigure::Server<velodyne_pointcloud::CloudNodeConfig>::
    //  CallbackType f;
    //f = boost::bind (&Convert::callback, this, _1, _2);
    //srv_->setCallback (f);

    // subscribe to VelodyneScan packets
    velodyne_scan_ =
      node.subscribe("velodyne_packets", 10,
                     &Mapper::processScan, (Mapper *) this,
                     ros::TransportHints().tcpNoDelay(true));
  }
  
  void Mapper::callback(//velodyne_pointcloud::CloudNodeConfig &config,
                uint32_t level)
  {
  ROS_INFO("Reconfigure Request");
  //data_->setParameters(config.min_range, config.max_range, config.view_direction,
  //                     config.view_width);
  }

  /** @brief Callback for raw scan messages. */
  void Mapper::processScan(const sensor_msgs::PointCloud2::ConstPtr &scanMsg)
  {
    if (output_.getNumSubscribers() == 0)         // no one listening?
      return;                                     // avoid much work

    // allocate a point cloud with same time and frame ID as raw data
    //velodyne_rawdata::VPointCloud::Ptr
    //  outMsg(new velodyne_rawdata::VPointCloud());
    // outMsg's header is a pcl::PCLHeader, convert it before stamp assignment
    //outMsg->header.stamp = pcl_conversions::toPCL(scanMsg->header).stamp;
    //outMsg->header.frame_id = scanMsg->header.frame_id;
    //outMsg->height = 1;

    // process each packet provided by the driver
    //for (size_t i = 0; i < scanMsg->packets.size(); ++i)
    //  {
    //    data_->unpack(scanMsg->packets[i], *outMsg);
    //  }

    // publish the accumulated cloud message
    //ROS_DEBUG_STREAM("Publishing " << outMsg->height * outMsg->width
    //                 << " Velodyne points, time: " << outMsg->header.stamp);
    //output_.publish(outMsg);
  }

} // namespace detection_height_mapper
