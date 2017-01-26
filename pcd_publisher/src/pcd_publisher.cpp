
#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/io/pcd_io.h>

using namespace std;

/**
\author Jan Razlaw
@b pcd_publisher is a simple node that loads a PCD (Point Cloud Data) file and publishes it repeatedly.
**/
class PCDPublisher
{
protected:
   ros::NodeHandle nh_;

private:
   string path_to_file_;

public:
   string output_topic_;
   string output_frame_id_;

   int publishing_rate_;

   ros::Publisher pub_;

   ////////////////////////////////////////////////////////////////////////////////
   // Callback
   void
   publish_cloud ()
   {
      // Read in the cloud data
      pcl::PCDReader reader;
      pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2);
      if(reader.read (path_to_file_, *cloud) < 0)
         exit(-1);

      cloud->header.frame_id = output_frame_id_;
      // Publish cloud at specified rate (in hz) with current timestamp
      ros::Rate r(publishing_rate_);
      while(ros::ok())
      {
         cloud->header.stamp = pcl_conversions::toPCL(ros::Time::now());
         pub_.publish(cloud);
         r.sleep();
      }
   }

   ////////////////////////////////////////////////////////////////////////////////
   PCDPublisher () :
     output_topic_("pcd_point_cloud")
     , output_frame_id_("velodyne")
     , publishing_rate_(10)
   {
      // Check if a path_to_file parameter is defined for input file name.
      ros::NodeHandle priv_nh("~");
      if (priv_nh.getParam ("path_to_file", path_to_file_))
      {
         ROS_INFO_STREAM ("Path to file is: " << path_to_file_);
      }
      else if (nh_.getParam ("path_to_file", path_to_file_))
      {
         ROS_WARN_STREAM ("Non-private path_to_file parameter is DEPRECATED: " << path_to_file_);
      }

      priv_nh.getParam ("output_topic", output_topic_);
      priv_nh.getParam ("output_frame_id", output_frame_id_);
      priv_nh.getParam ("publishing_rate", publishing_rate_);

      pub_ = nh_.advertise<pcl::PCLPointCloud2 >(output_topic_, 1);
      ROS_INFO ("Publishing clouds on topic %s",
                nh_.resolveName (output_topic_).c_str ());

      publish_cloud ();
   }
};

/* ---[ */
int
main (int argc, char** argv)
{
   ros::init (argc, argv, "pcd_publisher");

   PCDPublisher b;
   ros::spin ();

   return (0);
}
/* ]--- */
