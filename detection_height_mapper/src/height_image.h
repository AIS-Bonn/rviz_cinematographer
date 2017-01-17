// Omnidirectional height image
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef HEIGHT_IMAGE_H
#define HEIGHT_IMAGE_H

#include <sensor_msgs/Image.h>
#include <nav_msgs/OccupancyGrid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cv_bridge/cv_bridge.h>

#include <velodyne_object_detector/point_type.h>

//#include <mrs_laser_maps/map_point_types.h>
// #include <momaro_heightmap/HeightMap.h>

namespace momaro_heightmap
{

class HeightImage
{
public:
	HeightImage();
	~HeightImage();

	void setResolution(double res_x, double res_y);
	void setSize(double size_x, double size_y);
	void setSensorCenter(double x, double y);
	void setOrigin(double x, double y);

	void setNoiseSuppressionParameters(float minDiff, float minDiffSlope);

	template<class Point>
	void process(const pcl::PointCloud<Point>& cloud, unsigned char source = 0);

	void process(const cv::Mat& hight_image, double min_height, double max_height, unsigned char source = 0);

	template<class Point>
	void processWithTransform(const pcl::PointCloud<Point>& cloud, const Eigen::Affine3f& transform);

	template<class Point>
	void processMedianFiltered(const pcl::PointCloud<Point>& cloud, const Eigen::Affine3f& transform);

	void processObstaclesWithTransform(const pcl::PointCloud<velodyne_pointcloud::PointXYZIRDetection>& cloud, const Eigen::Affine3f& transform, float obstacle_thresh, float odds_hit, float odds_miss, float clamp_thresh_min, float clamp_thresh_max);
	
// 	void fillHeightMap(momaro_heightmap::HeightMap* map);

	void fillImage(sensor_msgs::Image* img, double min_value, double max_value);
	void fillMap(nav_msgs::OccupancyGrid* map, double min_value, double max_value, uint8_t max_map_value);
	void fillAbsoluteImage(sensor_msgs::Image* img, double min_value, double max_value);
	cv::Mat_<float> GetAbsoluteHeightData();

	void fillColorImage(sensor_msgs::Image* img);
	void fillHeightColorImage(sensor_msgs::Image* img, double min_height, double max_height, double min_diff, double max_diff);
	void fillObstacleColorImage(sensor_msgs::Image* img, double min_height, double max_height, double min_diff, double max_diff, double height_diff_thresh, int num_min_count, float obstacle_thresh);
	void fillObstacleMap(nav_msgs::OccupancyGrid* map, double min_value, double max_value, uint8_t max_map_value, double height_diff_thresh, int num_min_count, float obstacle_thresh);
	void detectObstacles(double height_diff_thresh, int num_min_count, float obstacle_thresh);
	void filterObstaclesBySize(const cv::Mat& prob_mat, int min_size_of_valid_obstacle, int max_size_of_valid_obstacle);

	void fillGaps(float fillSlope);
	void fillGapsWithMinimum();
	void calculateDifferences(int neighborHood = 1, cv::Mat_<float>* dest = 0, bool requireDense = false);
	void calculateDrivability(double maxFineDiff, double maxCoarseDiff, double maxVeryCoarseDiff, double exponent);

	void suppressNoise(double thresholdSlope);
	void suppressCameraEdges();

	void inflate(float hardRadius, float softRadius);

	void clearCircle( double x, double y, double radius );
	void clearRobot(float value = 0.0);

	void medianFilter(float apertureSize);
	
	std::vector<float> getCostsStorage() const;
	
	inline int cellsX() const
	{ return m_buckets_x; }
	
	inline int cellsY() const
	{ return m_buckets_y; }
private:
	void resizeStorage();

	double m_res_x;
	double m_res_y;
	double m_length_x;
	double m_length_y;
	int m_mid_x;
	int m_mid_y;
	int m_buckets_x;
	int m_buckets_y;
	cv::Mat_<float> m_absolute;
	cv::Mat_<float> m_absolute_min;
	cv::Mat_<float> m_absolute_max;
	cv::Mat_<float> m_obstacle_min;
	cv::Mat_<float> m_obstacle_max;
	cv::Mat_<float> m_obstacle;
	cv::Mat_<float> m_obstacles_inflated;
	cv::Mat_<int> m_obstacle_count;
	cv::Mat_<int> m_obstacle_last_scan_id;
	cv::Mat_<int> m_obstacle_scans_count;
	cv::Mat_<float> m_relative;
	cv::Mat_<float> m_mask;
	cv::Mat_<unsigned char> m_source;
	cv::Mat_<int> m_small_obstacles;
	cv::Point2i m_sensorCenter;
	double m_robotRadius;

	// Parameters
	float m_noiseSupMin;
	float m_noiseSupMinSlope;
};

// IMPLEMENTATION

template<class Point>
void HeightImage::process(const pcl::PointCloud<Point>& cloud, unsigned char source)
{
	for(typename pcl::PointCloud<Point>::const_iterator it = cloud.begin(); it != cloud.end(); ++it)
	{
		const Point& point = *it;

		int bin_x = point.x / m_res_x;
		int bin_y = -point.y / m_res_y;

		bin_x += m_mid_x;
		bin_y += m_mid_y;

		if(bin_x < 0 || bin_x >= m_buckets_x || bin_y < 0 || bin_y >= m_buckets_y)
			continue;

		float* binval = &m_absolute(bin_y, bin_x);

		if(std::isnan(*binval) || point.z > *binval)
		{
			*binval = point.z;
			m_source(bin_y,bin_x) = source;
		}
	}
}

template<class Point>
void HeightImage::processWithTransform(const pcl::PointCloud<Point>& cloud, const Eigen::Affine3f& transform)
{
	for(typename pcl::PointCloud<Point>::const_iterator it = cloud.begin(); it != cloud.end(); ++it)
	{
		const Point& point = *it;

		Eigen::Vector3f pos(point.x, point.y, point.z);
		pos = transform * pos;
		int bin_x = pos.x() / m_res_x;
		int bin_y = m_buckets_y - pos.y() / m_res_y;

		if(bin_x < 0 || bin_x >= m_buckets_x || bin_y < 0 || bin_y >= m_buckets_y)
			continue;

		float* binval = &m_absolute(bin_y, bin_x);

		if(std::isnan(*binval) || point.z > *binval)
		{
			*binval = point.z;
			m_source(bin_y,bin_x) = 0;
		}
	}
}

// use transformed point cloud to fill m_absolute_min and -max with min and max height (z-coordinate) 
// of the points in each bin + fill m_absolute with median height for each bin 
template<class Point>
void HeightImage::processMedianFiltered(const pcl::PointCloud<Point>& cloud, const Eigen::Affine3f& transform)
{
	std::vector<std::vector<float> > storage(m_buckets_x * m_buckets_y);

	for(typename pcl::PointCloud<Point>::const_iterator it = cloud.begin(); it != cloud.end(); ++it)
	{
		const Point& point = *it;

        // transform point and compute corresponding bin 
		Eigen::Vector3f pos(point.x, point.y, point.z);
		pos = transform * pos;
		int bin_x = pos.x() / m_res_x;
		int bin_y = m_buckets_y - pos.y() / m_res_y;

		if(bin_x < 0 || bin_x >= m_buckets_x || bin_y < 0 || bin_y >= m_buckets_y)
			continue;

		unsigned int idx = bin_y * m_buckets_x + bin_x;

		storage[idx].push_back(point.z);
		
		// compute min and max for current bin 
		float* binval_min = &m_absolute_min(bin_y, bin_x);
		float* binval_max = &m_absolute_max(bin_y, bin_x);
		if(std::isnan(*binval_min) || point.z < *binval_min)
		{
			*binval_min = point.z;
		}
		if(std::isnan(*binval_max) || point.z > *binval_max)
		{
			*binval_max = point.z;
		}
	}
	
	// compute median height for each bin 
	unsigned int idx = 0;
	for(unsigned int bin_y = 0; bin_y < m_buckets_y; ++bin_y)
	{
		for(unsigned int bin_x = 0; bin_x < m_buckets_x; ++bin_x, ++idx)
		{
			std::vector<float>& points = storage[idx];

			if(points.empty())
				continue;

            // set median to n-th element
			size_t n = (points.size()-1) / 2;
			std::nth_element(points.begin(), points.begin()+n, points.end());

            // if there is not exactly 1 median element
			if(points.size() % 2 == 0)
			{
			    // get element that is the next bigger one after the "median" 
				std::vector<float>::iterator it = std::min_element(points.begin()+n+1, points.end());
                // check for invalid iterator position 
				if(it == points.end())
				{
					fprintf(stderr, "invalid: %d %d %d\n", (int)points.size(), (int)n, (int)(it - points.begin()));
					std::abort();
				}

                // compute the mean of both values and use this as the median 
				m_absolute(bin_y, bin_x) = (points[n]+(*it))/2;
				// TODO: necessary?
				m_source(bin_y, bin_x) = 0;
			}
			else
			{
				m_absolute(bin_y, bin_x) = points[n];
				// TODO: necessary?
				m_source(bin_y, bin_x) = 0;
			}
		}
	}
}

}

#endif
