// Omnidirectional height image
// Original author: Max Schwarz <max.schwarz@uni-bonn.de>
// Adapted by Jan Razlaw <s6jarazl@uni-bonn.de>

#ifndef HEIGHT_IMAGE_H
#define HEIGHT_IMAGE_H


#include <ros/console.h>

#include <limits>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <nav_msgs/OccupancyGrid.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <velodyne_object_detector/point_type.h>

namespace momaro_heightmap
{

class HeightImage
{
public:
	typedef velodyne_pointcloud::PointXYZDetection  PointWithDetection;

	typedef PointWithDetection                      InputPoint;
	typedef pcl::PointCloud<InputPoint>             InputPointCloud;

	HeightImage();
	~HeightImage();

	void setResolution(double res_x, double res_y);
	void setSize(double size_x, double size_y);

	void processPointcloud(const InputPointCloud& cloud,
                          const Eigen::Affine3f& transform,
                          float obstacle_thresh,
                          float odds_hit,
                          float odds_miss,
                          float clamp_thresh_min,
                          float clamp_thresh_max);

	void fillObstacleColorImage(sensor_msgs::Image* img, double min_height, double max_height, double min_diff, double max_diff, double height_diff_thresh, int num_min_count, float obstacle_thresh);
	void fillObstacleMap(nav_msgs::OccupancyGrid* map, double min_value, double max_value, uint8_t max_map_value, double height_diff_thresh, int num_min_count, float obstacle_thresh);
	void detectObstacles(double height_diff_thresh, int num_min_count, float obstacle_thresh);
	void filterObstaclesBySize(const cv::Mat& prob_mat, int min_size_of_valid_obstacle, int max_size_of_valid_obstacle);

private:
	void resizeStorage();

	double m_res_x;
	double m_res_y;
	double m_length_x;
	double m_length_y;
	int m_buckets_x;
	int m_buckets_y;
	cv::Mat_<float> m_median_height;
	cv::Mat_<float> m_min_height;
	cv::Mat_<float> m_max_height;
	cv::Mat_<float> m_obstacle_min_height;
	cv::Mat_<float> m_obstacle_max_height;
	cv::Mat_<float> m_obstacle_detection;
	cv::Mat_<float> m_obstacles_inflated;
	cv::Mat_<int> m_obstacle_count;
	cv::Mat_<int> m_obstacle_last_scan_id;
	cv::Mat_<int> m_obstacle_scans_count;
	cv::Mat_<float> m_relative;
	cv::Mat_<float> m_mask;
	cv::Mat_<unsigned char> m_source;
	cv::Mat_<int> m_small_obstacles;
	double m_robotRadius;
};

}

#endif
