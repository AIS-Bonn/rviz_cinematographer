// Visualizes the Entropy and Plane Variance
// Author: Jan Razlaw <jan.razlaw@gmx.de>

#ifndef COST_VISUALIZATION_H
#define COST_VISUALIZATION_H

#include <iostream>
#include <stdlib.h>
#include <time.h>
#include <random>

#include <ros/publisher.h>

#include <pcl_ros/point_cloud.h>

#include <visualization_msgs/Marker.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/common/geometry.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/search/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>

#include <pcl_conversions/pcl_conversions.h>

#include <config_server/parameter.h>

class CostVis
{
public:
   typedef pcl::PointXYZ Point;
   typedef pcl::PointCloud<Point> PointCloud;
   typedef pcl::KdTreeFLANN<Point> KdTree;


   CostVis();
	~CostVis();
   void load();
	void update();
private:
	float costs(const Eigen::Vector3f& mean_1,
				  const Eigen::Vector3f& mean_2,
				  const Eigen::Matrix3f& cov_1,
				  const Eigen::Matrix3f& cov_2);

   double bhattacharyya(const Eigen::Vector3f& dist,
                        const Eigen::Matrix3f& cov1,
                        const Eigen::Matrix3f& cov2);


   ros::NodeHandle nh_;
	ros::Publisher pub_marker_;
   ros::Publisher pub_pointcloud_1_;
   ros::Publisher pub_pointcloud_2_;

   PointCloud::Ptr initial_cloud_1_;
   PointCloud::Ptr initial_cloud_2_;

   visualization_msgs::Marker gaus_1_marker_;
   visualization_msgs::Marker gaus_2_marker_;

	config_server::Parameter<float> mean_1_x_;
	config_server::Parameter<float> mean_1_y_;
	config_server::Parameter<float> mean_1_z_;
	config_server::Parameter<float> mean_2_x_;
	config_server::Parameter<float> mean_2_y_;
	config_server::Parameter<float> mean_2_z_;

	config_server::Parameter<float> cov_1_xx_;
	config_server::Parameter<float> cov_1_yy_;
	config_server::Parameter<float> cov_1_zz_;
	config_server::Parameter<float> cov_1_xy_;
	config_server::Parameter<float> cov_1_xz_;
	config_server::Parameter<float> cov_1_yz_;

   config_server::Parameter<float> cov_2_xx_;
   config_server::Parameter<float> cov_2_yy_;
   config_server::Parameter<float> cov_2_zz_;
   config_server::Parameter<float> cov_2_xy_;
   config_server::Parameter<float> cov_2_xz_;
   config_server::Parameter<float> cov_2_yz_;

   float prev_dist_;
   float prev_mahal_;
   float prev_other_;
   std::string mahal_sign;
   std::string other_sign;
};

#endif
