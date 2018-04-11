// Visualizes the Bhattacharyya distance between two (Gaussian) distributions
// Author: Jan Razlaw <jan.razlaw@gmx.de>

#ifndef COST_VISUALIZATION_H
#define COST_VISUALIZATION_H

#include <iostream>
#include <stdlib.h>
#include <time.h>
#include <random>

#include <ros/publisher.h>

#include <pcl_ros/point_cloud.h>

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

class BhattacharyyaDistanceVisualizer
{
public:
	typedef pcl::PointXYZ Point;
	typedef pcl::PointCloud<Point> PointCloud;

	BhattacharyyaDistanceVisualizer();
	~BhattacharyyaDistanceVisualizer();

	void load();
	void update();
private:
	float bhattacharyya(const Eigen::Vector3f &mean_1,
											const Eigen::Vector3f &mean_2,
											const Eigen::Matrix3f &cov_1,
											const Eigen::Matrix3f &cov_2);

	double bhattacharyya(const Eigen::Vector3f& dist,
											 const Eigen::Matrix3f& cov1,
											 const Eigen::Matrix3f& cov2);


	ros::NodeHandle nh_;
	ros::Publisher pub_pointcloud_1_;
	ros::Publisher pub_pointcloud_2_;

	PointCloud::Ptr initial_cloud_1_;
	PointCloud::Ptr initial_cloud_2_;

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
};

#endif
