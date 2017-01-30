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

	void setResolution(float res_x, float res_y);
   void setSize(float size_x, float size_y);
   void setRobotRadius(float robot_radius);
   void setMinHeight(float min_height);
   void setMaxHeight(float max_height);
   void setMinObjectHeight(float min_height);
   void setMaxObjectHeight(float max_height);
   void setMinObjectFootprint(float min_footprint_size);
   void setMaxObjectFootprint(float max_footprint_size);
   void setMaxObjectAltitude(float max_altitude);
   void setDetectionThreshold(float detection_threshold);
   void setMaxNeighborhoodHeight(float max_neighborhood_height);
   void setHardInflationRadius(float hard_inflation_radius);
   void setSoftInflationRadius(float soft_inflation_radius);

	void processPointcloud(const InputPointCloud& cloud,
                          const Eigen::Affine3f& transform,
                          float odds_hit,
                          float odds_miss,
                          float clamp_thresh_min,
                          float clamp_thresh_max);

   void detectObjects(int num_min_count,
                      bool inflate_objects);

	void fillObjectColorImage(sensor_msgs::Image* img);

	void fillObjectMap(nav_msgs::OccupancyGrid* map);

private:
	void resizeStorage();

   void filterObjectsBySize(const cv::Mat& prob_mat,
                            cv::Mat& result_mat,
                            int min_size_of_valid_object,
                            int max_size_of_valid_object);

   void filterObjectsByNeighborHeight(cv::Mat& prob_mat,
                                      float robot_radius,
                                      float height_threshold);

   void inflateObjects(cv::Mat& prob_mat,
                       float hard_radius,
                       float soft_radius);

   float m_res_x;
   float m_res_y;
   float m_length_x;
   float m_length_y;
   float m_robot_radius;
   int m_buckets_x;
	int m_buckets_y;
   float m_min_height_threshold;
   float m_max_height_threshold;
   float m_min_object_height_threshold;
   float m_max_object_height_threshold;
   float m_min_footprint_size;
   float m_max_footprint_size;
   float m_max_object_altitude_threshold;
   float m_object_detection_threshold;
   float m_max_neighborhood_height_threshold;
   float m_hard_inflation_radius;
   float m_soft_inflation_radius;

	cv::Mat_<float> m_median_height;
	cv::Mat_<float> m_min_height;
	cv::Mat_<float> m_max_height;
	cv::Mat_<float> m_object_min_height;
	cv::Mat_<float> m_object_max_height;
	cv::Mat_<float> m_object_detection;
	cv::Mat_<float> m_objects_inflated;
	cv::Mat_<int> m_object_count;
	cv::Mat_<int> m_object_last_scan_id;
	cv::Mat_<int> m_object_scans_count;
	cv::Mat_<int> m_small_objects;
};

}

#endif
