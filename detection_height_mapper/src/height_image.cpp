// Omnidirectional height image
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "height_image.h"

#include <sensor_msgs/image_encodings.h>

#include <opencv2/imgproc/imgproc.hpp>

#include <ros/console.h>

#include <limits>

//#include "median_filter.h"

#define DEBUG 0

// #if DEBUG
#include <opencv2/highgui/highgui.hpp>
// #endif

namespace
{
	float robustMaximum(float a, float b)
	{
		if(std::isnan(b))
			return a;

		if(std::isnan(a))
			return b;

		return std::max(a, b);
	}
}

namespace momaro_heightmap
{

HeightImage::HeightImage()
 : m_res_x(0.05)
 , m_res_y(0.05)
 , m_length_x(8.0)
 , m_length_y(8.0)
 , m_robotRadius(0.42)
{
	resizeStorage();
}

HeightImage::~HeightImage()
{
}

void HeightImage::resizeStorage()
{
	m_buckets_x = std::ceil(m_length_x / m_res_x);
	m_buckets_y = std::ceil(m_length_y / m_res_y);
	m_mid_x = m_buckets_x / 2;
	m_mid_y = m_buckets_y / 2;

	m_absolute.create(m_buckets_y, m_buckets_x);
	m_absolute_min.create(m_buckets_y, m_buckets_x);
	m_absolute_max.create(m_buckets_y, m_buckets_x);
	m_obstacle.create(m_buckets_y, m_buckets_x);
	m_obstacle_min.create(m_buckets_y, m_buckets_x);
	m_obstacle_max.create(m_buckets_y, m_buckets_x);
	m_obstacle_count.create(m_buckets_y, m_buckets_x);
	m_obstacle_last_scan_id.create(m_buckets_y, m_buckets_x);
	m_obstacle_scans_count.create(m_buckets_y, m_buckets_x);
	m_obstacles_inflated.create(m_buckets_y, m_buckets_x);
	m_relative.create(m_buckets_y, m_buckets_x);
	m_mask.create(m_buckets_y, m_buckets_x);
	m_source.create(m_buckets_y, m_buckets_x);
	m_small_obstacles.create(m_buckets_y, m_buckets_x);

	// Setup robot mask
	m_mask.setTo(cv::Scalar(0.0f));

	m_absolute = NAN;
	m_absolute_min = NAN;
	m_absolute_max = NAN;
	m_obstacle = NAN;
	m_obstacle_min = NAN;
	m_obstacle_max = NAN;
	m_obstacles_inflated = NAN;
	m_obstacle_count = 0;
	m_obstacle_scans_count = 0;
	m_obstacle_last_scan_id = 0;
	m_small_obstacles = 0;
// 	// Big footprint rectangle
// 	{
// 		cv::Point2i p1(m_mid_x - (m_robotRadius + 0.3) / m_res_x, m_mid_y - (m_robotRadius + 0.05) / m_res_y);
// 		cv::Point2i p2(m_mid_x + (m_robotRadius + 0.1) / m_res_x, m_mid_y + (m_robotRadius + 0.05)/ m_res_y);
// 
// 		cv::rectangle(m_mask, p1, p2, cv::Scalar(1), -1);
// 	}
}

void HeightImage::setResolution(double res_x, double res_y)
{
	m_res_x = res_x;
	m_res_y = res_y;
	resizeStorage();
}

void HeightImage::setSize(double size_x, double size_y)
{
	m_length_x = size_x;
	m_length_y = size_y;
	resizeStorage();
}

void HeightImage::setOrigin(double x, double y)
{
	m_mid_x = x / m_res_x;
	m_mid_y = m_buckets_y - y / m_res_y;
	resizeStorage();
}

void HeightImage::fillImage(sensor_msgs::Image* img, double min_value, double max_value)
{
	img->width = m_buckets_x;
	img->height = m_buckets_y;
	img->step = m_buckets_x;
	img->encoding = sensor_msgs::image_encodings::MONO8;
	img->data.resize(m_buckets_x * m_buckets_y);

	int ridx = 0;
	uint8_t* wptr = img->data.data();
	for(int y = 0; y < m_buckets_y; ++y)
	{
		for(int x = 0; x < m_buckets_x; ++x)
		{
			double v = (m_relative(y, x) - min_value) / (max_value - min_value);

			uint8_t val = 255;

			if(!std::isnan(v))
			{
				int64_t unbounded = v * 254;

				if(v > 1.0 || unbounded > 254)
					val = 254;
				else if(unbounded < 0)
					val = 0;
				else
					val = unbounded;
			}

			*wptr = val;

			wptr++;
			ridx++;
		}
	}
}

void HeightImage::fillMap(nav_msgs::OccupancyGrid* map, double min_value, double max_value, uint8_t max_map_value)
{
	map->info.height = m_buckets_y;
	map->info.width = m_buckets_x;

	map->info.resolution = m_res_x;
	//map->info.origin.position.x = -m_res_x * (m_buckets_x/2);
	//map->info.origin.position.y = -m_res_y * (m_buckets_y/2);
	map->info.origin.position.x = 0;
	map->info.origin.position.y = 0;
	map->info.origin.orientation.w = 1;

	map->data.resize(m_buckets_x * m_buckets_y);

	int8_t* wptr = map->data.data();
	for(unsigned int y = 0; y < map->info.height; ++y)
	{
		for(unsigned int x = 0; x < map->info.width; ++x)
		{
			double v = (m_relative(m_buckets_y - y - 1, x) - min_value) / (max_value - min_value);

			if(std::isnan(v))
				*wptr = 255; // Unknown
			else
			{
// 				int64_t unbounded = v * 254;
				if(v >= 1.0)
					*wptr = max_map_value;
				else if(v < 0)
					*wptr = 0;
				else
					*wptr = v * max_map_value;
				//else
				//	*wptr = unbounded;
			}

			wptr++;
		}
	}
}

void HeightImage::fillAbsoluteImage(sensor_msgs::Image* img, double min_value, double max_value)
{
	img->width = m_buckets_x;
	img->height = m_buckets_y;
	img->step = m_buckets_x;
	img->encoding = sensor_msgs::image_encodings::MONO8;
	img->data.resize(m_buckets_x * m_buckets_y);

	int ridx = 0;
	uint8_t* wptr = img->data.data();
	for(int y = 0; y < m_buckets_y; ++y)
	{
		for(int x = 0; x < m_buckets_x; ++x)
		{
			double v = (m_absolute(y, x) - min_value) / (max_value - min_value);

			uint8_t val = 255;

			if(!std::isnan(v))
			{
				int64_t unbounded = v * 254;

				if(v > 1.0 || unbounded > 254)
					val = 254;
				else if(unbounded < 0)
					val = 0;
				else
					val = unbounded;
			}

			*wptr = val;

			wptr++;
			ridx++;
		}
	}
}

cv::Mat_<float> HeightImage::GetAbsoluteHeightData()
{
	std::cout << "returnig m_absolute with size: " << m_absolute.size() << std::endl;
	return m_absolute;
}

typedef uint16_t NeighborDist;

void HeightImage::fillGaps(float distSlope)
{
	cv::Mat_<float>& target = m_absolute;

	cv::Mat_<NeighborDist> leftNeighbours(m_absolute.size());
	cv::Mat_<NeighborDist> rightNeighbours(m_absolute.size());
	cv::Mat_<NeighborDist> topNeighbours(m_absolute.size());
	cv::Mat_<NeighborDist> bottomNeighbours(m_absolute.size());

	leftNeighbours = 0;
	rightNeighbours = 0;
	topNeighbours = 0;
	bottomNeighbours = 0;

#if DEBUG
	cv::Mat_<uint8_t> debug(m_absolute.size());
	for(int y = 0; y < m_buckets_y; ++y)
		for(int x = 0; x < m_buckets_x; ++x)
			debug(y,x) = (m_absolute(y,x) + 1.0)/3.5 * 255;
	cv::imwrite("/tmp/prefill.ppm", debug);
#endif

	// Row-wise neighbors
	for(int y = 0; y < m_buckets_y; ++y)
	{
		float* row = target[y];
		NeighborDist* lRow = leftNeighbours[y];
		NeighborDist* rRow = rightNeighbours[y];

		NeighborDist distance = 1;

		for(int x = 0; x < m_buckets_x; ++x, ++distance)
		{
			if(!std::isnan(row[x]))
				distance = 0;
			else
				lRow[x] = distance;
		}

		distance = 1;
		for(int x = m_buckets_x - 1; x != -1; --x, ++distance)
		{
			if(!std::isnan(row[x]))
				distance = 0;
			else
				rRow[x] = distance;
		}
	}

	// Col-wise neighbors
	for(int x = 0; x < m_buckets_x; ++x)
	{
		NeighborDist distance = 1;

		for(int y = 0; y < m_buckets_y; ++y, ++distance)
		{
			if(!std::isnan(target(y, x)))
				distance = 0;
			else
				topNeighbours(y,x) = distance;
		}

		distance = 1;
		for(int y = m_buckets_y-1; y != -1; --y, ++distance)
		{
			if(!std::isnan(target(y,x)))
				distance = 0;
			else
				bottomNeighbours(y,x) = distance;
		}
	}

	// And fill in the data
	for(int y = 0; y < m_buckets_y; ++y)
	{
		float* row = target[y];
		NeighborDist* lRow = leftNeighbours[y];
		NeighborDist* rRow = rightNeighbours[y];
		NeighborDist* tRow = topNeighbours[y];
		NeighborDist* bRow = bottomNeighbours[y];

		for(int x = 0; x < m_buckets_x; ++x)
		{
			if(!std::isnan(row[x]))
				continue;

			cv::Point2i p(x, y);

			int distances[] = {
				lRow[x],
				rRow[x],
				tRow[x],
				bRow[x]
			};
			cv::Point2i neighbors[4] = {
				p + cv::Point2i(-lRow[x], 0),
				p + cv::Point2i(rRow[x], 0),
				p + cv::Point2i(0, -tRow[x]),
				p + cv::Point2i(0, bRow[x])
			};

			double totalWeight = 0;
			double totalValue = 0;
			int smallest_distance = INT_MAX;
			int validCount = 0;

			m_source(y,x) = 9;

			for(int i = 0; i < 4; ++i)
			{
				const cv::Point2i& pos = neighbors[i];

				float value = 0.0;
				float weight = 1.0 / distances[i];

				if(pos.x >= 0 && pos.x < m_buckets_x && pos.y >= 0 && pos.y < m_buckets_y)
				{
					value = target(pos.y, pos.x);
					
					if(!std::isfinite(value))
						std::abort();
					
					if(distances[i] <= 0.2 / m_res_x)
						validCount++;

					if(distances[i] < smallest_distance)
					{
						smallest_distance = distances[i];
						m_source(y,x) = m_source(pos.y, pos.x);
					}

					totalValue += weight * value;
					totalWeight += weight;
				}
			}

			if(validCount >= 2 && totalWeight != 0.0)
			{
				totalValue /= totalWeight;
				row[x] = totalValue;
			}
		}
	}

#if DEBUG
	for(int y = 0; y < m_buckets_y; ++y)
		for(int x = 0; x < m_buckets_x; ++x)
			debug(y,x) = (m_absolute(y,x) + 1.0)/3.5 * 255;
	cv::imwrite("/tmp/filled.ppm", debug);
	for(int y = 0; y < m_buckets_y; ++y)
		for(int x = 0; x < m_buckets_x; ++x)
			debug(y,x) = leftNeighbours(y,x);
	cv::imwrite("/tmp/leftNeighbors.ppm", debug);
	for(int y = 0; y < m_buckets_y; ++y)
		for(int x = 0; x < m_buckets_x; ++x)
			debug(y,x) = bottomNeighbours(y,x);
	cv::imwrite("/tmp/bottomNeighbors.ppm", debug);
	for(int y = 0; y < m_buckets_y; ++y)
		for(int x = 0; x < m_buckets_x; ++x)
			debug(y,x) = topNeighbours(y,x);
	cv::imwrite("/tmp/topNeighbors.ppm", debug);
	for(int y = 0; y < m_buckets_y; ++y)
		for(int x = 0; x < m_buckets_x; ++x)
			debug(y,x) = rightNeighbours(y,x);
	cv::imwrite("/tmp/rightNeighbors.ppm", debug);
#endif
}

void HeightImage::fillGapsWithMinimum()
{
	cv::Mat_<float>& target = m_absolute;

	cv::Mat_<NeighborDist> leftNeighbours(m_absolute.size());
	cv::Mat_<NeighborDist> rightNeighbours(m_absolute.size());
	cv::Mat_<NeighborDist> topNeighbours(m_absolute.size());
	cv::Mat_<NeighborDist> bottomNeighbours(m_absolute.size());

	leftNeighbours = 0;
	rightNeighbours = 0;
	topNeighbours = 0;
	bottomNeighbours = 0;

#if DEBUG
	cv::Mat_<uint8_t> debug(m_absolute.size());
	for(int y = 0; y < m_buckets_y; ++y)
		for(int x = 0; x < m_buckets_x; ++x)
			debug(y,x) = (m_absolute(y,x) - 1.0)/2.0 * 255;
	cv::imwrite("/tmp/prefill.ppm", debug);
#endif

	// Row-wise neighbors
	for(int y = 0; y < m_buckets_y; ++y)
	{
		float* row = target[y];
		NeighborDist* lRow = leftNeighbours[y];
		NeighborDist* rRow = rightNeighbours[y];

		NeighborDist distance = 1;

		for(int x = 0; x < m_buckets_x; ++x, ++distance)
		{
			if(!std::isnan(row[x]))
				distance = 0;
			else
				lRow[x] = distance;
		}

		distance = 1;
		for(int x = m_buckets_x - 1; x != -1; --x, ++distance)
		{
			if(!std::isnan(row[x]))
				distance = 0;
			else
				rRow[x] = distance;
		}
	}

	// Col-wise neighbors
	for(int x = 0; x < m_buckets_x; ++x)
	{
		NeighborDist distance = 1;

		for(int y = 0; y < m_buckets_y; ++y, ++distance)
		{
			if(!std::isnan(target(y, x)))
				distance = 0;
			else
				topNeighbours(y,x) = distance;
		}

		distance = 1;
		for(int y = m_buckets_y-1; y != -1; --y, ++distance)
		{
			if(!std::isnan(target(y,x)))
				distance = 0;
			else
				bottomNeighbours(y,x) = distance;
		}
	}

	// And fill in the data
	for(int y = 0; y < m_buckets_y; ++y)
	{
		float* row = target[y];
		NeighborDist* lRow = leftNeighbours[y];
		NeighborDist* rRow = rightNeighbours[y];
		NeighborDist* tRow = topNeighbours[y];
		NeighborDist* bRow = bottomNeighbours[y];

		for(int x = 0; x < m_buckets_x; ++x)
		{
			if(!std::isnan(row[x]))
				continue;

			cv::Point2i p(x, y);

			int distances[] = {
				lRow[x],
				rRow[x],
				tRow[x],
				bRow[x]
			};
			cv::Point2i neighbors[4] = {
				p + cv::Point2i(-lRow[x], 0),
				p + cv::Point2i(rRow[x], 0),
				p + cv::Point2i(0, -tRow[x]),
				p + cv::Point2i(0, bRow[x])
			};

			float totalValue = std::numeric_limits<float>::infinity();
			int smallest_distance = INT_MAX;
			int validCount = 0;

			m_source(y,x) = 9;

			for(int i = 0; i < 4; ++i)
			{
				const cv::Point2i& pos = neighbors[i];

				float value = 0.0;

				if(pos.x >= 0 && pos.x < m_buckets_x && pos.y >= 0 && pos.y < m_buckets_y)
				{
					value = target(pos.y, pos.x);
					if(distances[i] <= 10)
						validCount++;

					if(distances[i] < smallest_distance)
					{
						smallest_distance = distances[i];
						m_source(y,x) = m_source(pos.y, pos.x);
					}

					totalValue = std::min(totalValue, value);
				}
			}

			if(validCount >= 2)
				row[x] = totalValue;
		}
	}

#if DEBUG
	for(int y = 0; y < m_buckets_y; ++y)
		for(int x = 0; x < m_buckets_x; ++x)
			debug(y,x) = (m_absolute(y,x) - 1.0)/2.0 * 255;
	cv::imwrite("/tmp/filled.ppm", debug);
	for(int y = 0; y < m_buckets_y; ++y)
		for(int x = 0; x < m_buckets_x; ++x)
			debug(y,x) = leftNeighbours(y,x);
	cv::imwrite("/tmp/leftNeighbors.ppm", debug);
	for(int y = 0; y < m_buckets_y; ++y)
		for(int x = 0; x < m_buckets_x; ++x)
			debug(y,x) = bottomNeighbours(y,x);
	cv::imwrite("/tmp/bottomNeighbors.ppm", debug);
	for(int y = 0; y < m_buckets_y; ++y)
		for(int x = 0; x < m_buckets_x; ++x)
			debug(y,x) = topNeighbours(y,x);
	cv::imwrite("/tmp/topNeighbors.ppm", debug);
	for(int y = 0; y < m_buckets_y; ++y)
		for(int x = 0; x < m_buckets_x; ++x)
			debug(y,x) = rightNeighbours(y,x);
	cv::imwrite("/tmp/rightNeighbors.ppm", debug);
#endif
}

void HeightImage::calculateDifferences(int neighborHood, cv::Mat_<float>* dest, bool requireDense)
{
	if(!dest)
		dest = &m_relative;

	for(int y = 0; y < m_absolute.rows; ++y)
	{
		for(int x = 0; x < m_absolute.cols; ++x)
		{
			float max_delta = NAN;

			float center_val = m_absolute(y, x);

// 			unsigned char source = m_source(y, x);

			bool source_ok = false;
			bool dense = true;

			for(int _y = std::max(0, y - neighborHood); _y < std::min(m_absolute.rows, y + neighborHood+1); ++_y)
			{
				for(int _x = std::max(0, x - neighborHood); _x < std::min(m_absolute.cols, x + neighborHood+1); ++_x)
				{
					if(m_mask(_y, _x))
						continue;

// 					if(m_source(_y, _x) != source)
// 						continue;

					source_ok = true;
					float delta = fabs(center_val - m_absolute(_y, _x));
					max_delta = robustMaximum(max_delta, delta);
					
					if(!std::isfinite(delta))
						dense = false;
				}
			}

			if(!source_ok || (requireDense && !dense))
			{
				// We are surrounded by pixels from another camera.
				(*dest)(y,x) = NAN;
			}
			else
			{
				int mx = x - m_absolute.cols/2;
				int my = y - m_absolute.rows/2;
				double dist = sqrt(mx*mx + my*my);
				double min = m_noiseSupMin + dist * m_noiseSupMinSlope;

				if(max_delta < min)
					(*dest)(y,x) = 0.0f;
				else
					(*dest)(y,x) = max_delta;
			}
		}
	}
}

void HeightImage::calculateDrivability(double maxFineDiff, double maxCoarseDiff, double maxVeryCoarseDiff, double exponent)
{
	cv::Mat_<float> d1;
	cv::Mat_<float> d3;
	cv::Mat_<float> d6;

	d1.create(cv::Size(m_relative.cols, m_relative.rows));
	d3.create(cv::Size(m_relative.cols, m_relative.rows));
	d6.create(cv::Size(m_relative.cols, m_relative.rows));

	calculateDifferences(1, &d1, true);
	calculateDifferences(3, &d3);
	calculateDifferences(6, &d6);

#if DEBUG
	printf("============================\n");
#endif

	for(int y = 0; y < m_relative.rows; ++y)
	{
		for(int x = 0; x < m_relative.cols; ++x)
		{
			if(!std::isfinite(d1(y,x)))
			{
#if DEBUG
				printf("\033[31m%d\033[0m", m_source(y,x));
#endif
				m_relative(y,x) = NAN;
				continue;
			}

#if DEBUG
			printf("%d", m_source(y,x));
#endif

			float norm1 = 0.5f * d1(y,x) / maxFineDiff;
			float norm3 = std::min(0.5f, 0.5f * d3(y,x) / (float)maxCoarseDiff);
			float norm6 = std::min(0.5f, 0.5f * d6(y,x) / (float)maxVeryCoarseDiff);

			float cost = std::max(0.0f, std::min(1.0f, norm1 + norm3 + norm6));
			
			m_relative(y,x) = pow(cost, exponent);//*cost;

/*			if(m_relative(y,x) < 0.999f)
				m_relative(y,x) /= 2.0f;*/
		}
#if DEBUG
		printf("\n");
#endif
	}

// 	clearRobot();
}


void HeightImage::setNoiseSuppressionParameters(float minDiff, float minDiffSlope)
{
	m_noiseSupMin = minDiff;
	m_noiseSupMinSlope = minDiffSlope;
}

void HeightImage::suppressNoise(double thresholdSlope)
{
	for(int y = 0; y < m_relative.rows; ++y)
	{
		float my = y - 0.5f * m_relative.rows;

		for(int x = 0; x < m_relative.cols; ++x)
		{
			float mx = x - 0.5f * m_relative.cols;

			float threshold = sqrt(mx*mx + my*my) * thresholdSlope;

			if(m_relative(y,x) < threshold)
				m_relative(y,x) = 0.0f;
		}
	}
}

void HeightImage::suppressCameraEdges()
{



	for(int y = 1; y < m_relative.rows-1; ++y)
	{
		for(int x = 1; x < m_relative.cols-1; ++x)
		{
			int source = m_source(y,x);
			bool mask = false;

			for(int dy = -1; dy <= 1; ++dy)
			{
				for(int dx = -1; dx <= 1; ++dx)
				{
					if(m_source(y+dy,x+dx) != source)
					{
						mask = true;
						break;
					}
				}

				if(mask)
					break;
			}

			if(mask)
			{
				const int MASK_SIZE = 1;
				for(int dy = -MASK_SIZE; dy <= MASK_SIZE; ++dy)
				{
					if(y+dy < 0 || y+dy >= m_relative.rows)
						continue;

					for(int dx = -MASK_SIZE; dx <= MASK_SIZE; ++dx)
					{
						if(x+dx < 0 || x+dx >= m_relative.cols)
							continue;

						float* value = &m_relative(y+dy,x+dx);
						if(!std::isnan(*value))
							*value = 0.0;
					}
				}
			}
		}
	}
}

void HeightImage::inflate(float hardRadius, float softRadius)
{
	cv::Mat_<float> uninflated;
	uninflated.create(cv::Size(m_relative.cols, m_relative.rows));

	hardRadius /= m_res_x;
	softRadius /= m_res_x;

	const int INFLATION_RADIUS = ceil(hardRadius);
	const int INFLATION_RADIUS_SOFT = ceil(softRadius);
//
	// 1st pass: inflate lethal obstacles

	for(int y = 0; y < m_relative.rows; ++y)
	{
		for(int x = 0; x < m_relative.cols; ++x)
		{
			bool cellFree = true;

			for(int dy = -INFLATION_RADIUS; dy <= INFLATION_RADIUS; ++dy)
			{
				int _y = y+dy;

				if(_y < 0 || _y >= m_relative.rows)
					continue;

				for(int dx = -INFLATION_RADIUS; dx <= INFLATION_RADIUS; ++dx)
				{
					int _x = x+dx;

					if(_x < 0 || _x >= m_relative.cols)
						continue;
						
					if(dx*dx+dy*dy > hardRadius*hardRadius)
						continue;

					if(m_relative(_y,_x) > 0.999f)
					{
						cellFree = false;
						break;
					}
				}
			}

			if(cellFree)
				uninflated(y, x) = m_relative(y,x);
			else
				uninflated(y, x) = 1.0f;
		}
	}
	
	// 2nd pass: inflate

	for(int y = 0; y < m_relative.rows; ++y)
	{
		for(int x = 0; x < m_relative.cols; ++x)
		{
			if(uninflated(y,x) >= 1.0)
			{
				m_relative(y,x) = 1.0;
				continue; // Do not "eat" into absolute obstacles
			}
			
			float costSum = 0;
			float weightSum = 0;
			
			float max = 0.0;

			for(int dy = -INFLATION_RADIUS_SOFT; dy <= INFLATION_RADIUS_SOFT; ++dy)
			{
				int _y = y+dy;

				if(_y < 0 || _y >= m_relative.rows)
					continue;

				for(int dx = -INFLATION_RADIUS_SOFT; dx <= INFLATION_RADIUS_SOFT; ++dx)
				{
					int _x = x+dx;

					if(_x < 0 || _x >= m_relative.cols)
						continue;
						
					if(dx*dx+dy*dy > softRadius*softRadius)
						continue;

					float dist = hypotf(dx,dy);
					float response = (-(1.0 / softRadius) * dist + 1.0) * uninflated(_y,_x);
					if(response > max)
					{
						max = response;
					}
					
					float weight = 1.0;

					costSum += weight*uninflated(_y, _x);
					weightSum += weight;
				}
			}

			m_relative(y,x) = costSum / weightSum;
		}
	}
}



void HeightImage::fillColorImage(sensor_msgs::Image* img)
{
	img->width = m_buckets_x;
	img->height = m_buckets_y;
	img->encoding = sensor_msgs::image_encodings::RGBA8;
	img->step = m_buckets_x * 4;
	img->data.resize(img->step * img->height);

	int ridx = 0;
	uint8_t* wptr = img->data.data();
	for(int y = 0; y < m_buckets_y; ++y)
	{
		for(int x = 0; x < m_buckets_x; ++x)
		{
			double height = m_absolute(y, x);

			int32_t val = (height + 2.0) / 4.0 * 255;
			if(val < 0) val = 0;
			if(val > 255) val = 255;

			wptr[0] = val;  // R
			wptr[1] = val;  // G
			wptr[2] = val;  // B
			wptr[3] = 0x55; // Alpha

			wptr += 4;
			ridx++;
		}
	}
}

// void HeightImage::fillHeightMap(momaro_heightmap::HeightMap* map)
// {
// 	map->cells_x = m_buckets_x;
// 	map->cells_y = m_buckets_y;
// 	map->origin_x = m_mid_x;
// 	map->origin_y = m_mid_y;
// 	map->resolution = (m_res_x + m_res_y) / 2.0;
// 
// 	map->height.resize(m_buckets_x * m_buckets_y * sizeof(float));
// 	float* wrptr = reinterpret_cast<float*>(map->height.data());
// 
// 	for(int y = 0; y < m_buckets_y; ++y)
// 	{
// 		memcpy(wrptr, m_absolute.ptr(y), m_buckets_x * sizeof(float));
// 		wrptr += m_buckets_x;
// 	}
// }

template<class T>
T limited(T val)
{
	return std::max(std::numeric_limits<T>::min(), std::min(std::numeric_limits<T>::max(), val));
}

template<class T>
T limited(T min, T val, T max)
{
	return std::max(min, std::min(max, val));
}

void HeightImage::fillHeightColorImage(sensor_msgs::Image* img, double min_height, double max_height, double min_diff, double max_diff)
{
	img->width = m_buckets_x;
	img->height = m_buckets_y;
	img->encoding = sensor_msgs::image_encodings::RGBA8;
	img->step = m_buckets_x * 4;
	img->data.resize(img->step * img->height);

	int ridx = 0;
	uint8_t* wptr = img->data.data();
	for(int y = 0; y < m_buckets_y; ++y)
	{
		for(int x = 0; x < m_buckets_x; ++x)
		{
			double gradient = m_relative(y, x);
			double height = m_absolute(y, x);
			double diff = fabs(m_obstacle_max(y, x)-m_obstacle_min(y, x));

			if(!std::isfinite(gradient) || !std::isfinite(height))
			{
				wptr[0] = wptr[1] = wptr[2] = 0;
				wptr[3] = 0x55;
				wptr += 4;
				ridx++;
				continue;
			}

			// use normalized height as brightness value
			double brightness = limited(0.0, (height - min_height) / (max_height - min_height), 1.0);
			double redshift = limited(0.0, (gradient -min_diff) / (max_diff-min_diff), 1.0);

// 			double y = std::max(0.0, std::min(1.0, (height - min_height) / (max_height - min_height)));
// 			double u = limited(-1.0, 2.0 * gradient / max_diff - 1.0, 1.0);
// 			double v = 0;
//
// 			double b = y + u / 0.493;
// 			double r = y + v / 0.877;
// 			double g = 1.704 * y - 0.509 * r - 0.194 * b;

			double r = brightness * redshift;
			double g = brightness * (1.0-redshift);
			double b = 0;

			wptr[0] = limited<uint8_t>(r * 255);  // R
			wptr[1] = limited<uint8_t>(g * 255);  // G
			wptr[2] = limited<uint8_t>(b * 255);  // B
			wptr[3] = 0x55; // Alpha

			wptr += 4;
			ridx++;
		}
	}
}


void HeightImage::detectObstacles(double height_diff_thresh, int num_min_count, float obstacle_thresh)
{
   // if bins meet some criteria, set those bins to 1 in m_obstacles_inflated
	// criteria: detection "probability" above threshold
	//    obstacle height between thresholds
	//    distance of obstacle top to ground under threshold
	//    number of scans that this bin was seen in above threshold
	for(int y = 0; y < m_buckets_y; ++y)
	{
		for(int x = 0; x < m_buckets_x; ++x)
		{
			double detection = m_obstacle(y, x);
			int num_detections = m_obstacle_scans_count(y, x);
			double diff = fabs(m_obstacle_max(y, x) - m_obstacle_min(y, x));
			double diff_to_abs_min = fabs(m_obstacle_max(y, x) - m_absolute_min(y, x));
			
			if(std::isfinite(detection))
			{
				if (detection > obstacle_thresh
			     && diff < height_diff_thresh
			     && diff > 0.05f
			     && diff_to_abs_min < 0.2 
			     && num_detections > num_min_count)
			   {
					m_obstacles_inflated(y, x) = 1.f;
			   }
			}
			
		}
	}
  
	cv::Mat_<float> uninflated;
	uninflated.create(cv::Size(m_relative.cols, m_relative.rows));

	float hardRadius = 0.25f;
	float softRadius = 0.1f;

	// radius in number of bins
	hardRadius /= m_res_x;
	softRadius /= m_res_x;
	const int INFLATION_RADIUS = ceil(hardRadius);
	const int INFLATION_RADIUS_SOFT = ceil(softRadius);

	// TODO: make more efficient
	// if at least one neighbor of current bin is set to 1, set current bin to 1
	// 1st pass: inflate lethal obstacles
	for(int y = 0; y < m_obstacles_inflated.rows; ++y)
	{
		for(int x = 0; x < m_obstacles_inflated.cols; ++x)
		{
			bool cellFree = true;

			for(int dy = -INFLATION_RADIUS; dy <= INFLATION_RADIUS; ++dy)
			{
				int _y = y+dy;

				if(_y < 0 || _y >= m_obstacles_inflated.rows)
					continue;

				for(int dx = -INFLATION_RADIUS; dx <= INFLATION_RADIUS; ++dx)
				{
					int _x = x+dx;

					if(_x < 0 || _x >= m_obstacles_inflated.cols)
						continue;
						
					if(dx*dx+dy*dy > hardRadius*hardRadius)
						continue;

					if(m_obstacles_inflated(_y,_x) > 0.999f)
					{
						cellFree = false;
						break;
					}
				}
			}

			// TODO: test effects of setting uninflated to zero instead of nan
			if(cellFree)
				uninflated(y, x) = m_obstacles_inflated(y,x);
			else
				uninflated(y, x) = 1.0f;
		}
	}
	
	// 2nd pass: inflate

	for(int y = 0; y < m_obstacles_inflated.rows; ++y)
	{
		for(int x = 0; x < m_obstacles_inflated.cols; ++x)
		{
			if(uninflated(y,x) >= 1.0)
			{
				m_obstacles_inflated(y,x) = 1.0;
				continue; // Do not "eat" into absolute obstacles
			}
			
			float costSum = 0;
			float weightSum = 0;
			
//			float max = 0.0;

			for(int dy = -INFLATION_RADIUS_SOFT; dy <= INFLATION_RADIUS_SOFT; ++dy)
			{
				int _y = y+dy;

				if(_y < 0 || _y >= m_obstacles_inflated.rows)
					continue;

				for(int dx = -INFLATION_RADIUS_SOFT; dx <= INFLATION_RADIUS_SOFT; ++dx)
				{
					int _x = x+dx;

					if(_x < 0 || _x >= m_obstacles_inflated.cols)
						continue;
						
					if(dx*dx+dy*dy > softRadius*softRadius)
						continue;

					float dist = hypotf(dx,dy);
					// the bigger the distance, the smaller the response
					// the fraction can be between 0 and 1
					// TODO: uninflated is probably not necessary because its either 1 or nan
					float response = (-(dist / softRadius) + 1.0) * uninflated(_y,_x);
//					if(response > max)
//					{
//						max = response;
//					}
					
					float weight = 1.0;
					// TODO: probably uninflated has to be switched with response
					costSum += weight*uninflated(_y, _x);
					weightSum += weight;
				}
			}

			m_obstacles_inflated(y,x) = costSum / weightSum;
		}
	}
	
	
	// filter objects which have a size between min and max size and save those in a binary mask named m_small_obstacles
	filterObstaclesBySize(m_obstacles_inflated, 0, 30);

	// set bins to zero if height difference in neighborhood exceeds a threshold
	// should prevent false positives that are too near to a wall or big obstacle
	float robot_radius = 2.5f;
	robot_radius /= m_res_x;
	const int ROBOT_RADIUS_INT = ceil(robot_radius);

	for(int y = 0; y < m_small_obstacles.rows; ++y)
	{
		for(int x = 0; x < m_small_obstacles.cols; ++x)
		{
			if(m_small_obstacles(y, x) == 0)
				continue;

			bool cell_canceled = false;
			double local_min_height = 9999.9;
			double local_max_height = -9999.9;

			// compute local min and max height
			for(int dy = -ROBOT_RADIUS_INT; dy <= ROBOT_RADIUS_INT; ++dy)
			{
				int _y = y+dy;

				if(_y < 0 || _y >= m_small_obstacles.rows)
					continue;

				for(int dx = -ROBOT_RADIUS_INT; dx <= ROBOT_RADIUS_INT; ++dx)
				{
					int _x = x+dx;

					if(_x < 0 || _x >= m_small_obstacles.cols)
						continue;

					if(dx*dx+dy*dy > robot_radius*robot_radius)
						continue;

					if(m_absolute_min(_y, _x) < local_min_height)
					{
						local_min_height = m_absolute_min(_y, _x);
					}
					if(m_absolute_max(_y, _x) > local_max_height)
					{
						local_max_height = m_absolute_max(_y, _x);
					}
				}
			}

			// filter
			double diff  = fabs(local_max_height - local_min_height);
			if(diff > 0.4)
			{
				m_small_obstacles(y, x) = 0;
			}
		}
	}

	m_obstacles_inflated =  m_small_obstacles.clone();
}

void HeightImage::fillObstacleMap(nav_msgs::OccupancyGrid* map, double min_value, double max_value, uint8_t max_map_value, double height_diff_thresh, int num_min_count, float obstacle_thresh)
{
	map->info.height = m_buckets_y;
	map->info.width = m_buckets_x;

	map->info.resolution = m_res_x;
	map->info.origin.position.x = -m_res_x * (m_buckets_x/2);
	map->info.origin.position.y = -m_res_y * (m_buckets_y/2);
	map->info.origin.position.z = -1.f;
// 	map->info.origin.position.x = 0;
// 	map->info.origin.position.y = 0;
	map->info.origin.orientation.w = 1;

	map->data.resize(m_buckets_x * m_buckets_y);

	int8_t* wptr = map->data.data();
	for(unsigned int y = 0; y < map->info.height; ++y)
	{
		for(unsigned int x = 0; x < map->info.width; ++x)
		{
			double v = m_obstacles_inflated(m_buckets_y - y - 1, x) ;

			double detection = m_obstacles_inflated(m_buckets_y - y - 1, x);
			int num_detections = m_obstacle_scans_count(m_buckets_y - y - 1, x);
			double height = m_absolute(m_buckets_y - y - 1, x);
			double diff = fabs(m_obstacle_max(m_buckets_y - y - 1, x)-m_obstacle_min(m_buckets_y - y - 1, x));
			double diff_to_abs_min = fabs(m_obstacle_max(m_buckets_y - y - 1, x)-m_absolute_min(m_buckets_y - y - 1, x));
			
			if(std::isnan(v))
				*wptr = (int8_t)0 ;//-1; // Unknown
			else
			{ 
			  
			  
// 			  if (isnan(current_wheel_cost))
//                                 wheel_cost_map_data[i] = (int8_t)(-1);
//                         else if (current_wheel_cost > 1000.0)
//                                 wheel_cost_map_data[i] = (int8_t)(-30);
//                         else
//                                 wheel_cost_map_data[i] = (int8_t)((current_wheel_cost - min_wheel_cost) / (max_wheel_cost - min_wheel_cost) * 100);

			  
			  
			  
  			   if (v > 0.5f)
  			   {
			     
			     *wptr = (int8_t)(129);
// 			     if(v >= 0.9)
// 					*wptr = (int8_t)(-30);
// // 					*wptr = (int8_t)(100);
// 			      else if(v < 0.6)
// // 					*wptr = (int8_t)(-30);
// 					*wptr = (int8_t)(0);
// 			      else

				
  			   }
//   			   else if (std::isfinite(height))
// 			     *wptr =  limited(0.0, (height - min_value) / (max_value - min_value), 1.0) * 100;
			   else
  			      *wptr = (int8_t)(0);
			     
			}
			
			


			wptr++;
		}
	}
}

void HeightImage::fillObstacleColorImage(sensor_msgs::Image* img, double min_height, double max_height, double min_diff, double max_diff, double height_diff_thresh, int num_min_count, float obstacle_thresh)
{
	img->width = m_buckets_x;
	img->height = m_buckets_y;
	img->encoding = sensor_msgs::image_encodings::RGBA8;
	img->step = m_buckets_x * 4;
	img->data.resize(img->step * img->height);
	int ridx = 0;
	uint8_t* wptr = img->data.data();

	// TODO: delete hack
	cv::flip(m_obstacles_inflated, m_obstacles_inflated, -1);
	cv::flip(m_absolute, m_absolute,-1);

	for(int y = 0; y < m_buckets_y; ++y)
	{
		for(int x = 0; x < m_buckets_x; ++x)
		{
			double detection = m_obstacles_inflated(y, x);
			int num_detections = m_obstacle_scans_count(y, x);
			double height = m_absolute(y, x);
			double diff = fabs(m_obstacle_max(y, x)-m_obstacle_min(y, x));
			double diff_to_abs_min = fabs(m_obstacle_max(y, x)-m_absolute_min(y, x));

			// background color if height in bin is not valid
			if(!std::isfinite(height))
			{
				wptr[0] = wptr[1] = wptr[2] = 255;
				wptr[3] = 255;
				wptr += 4;
				ridx++;
				continue;
			}

			// for non obstacle points
			// scale height and cap to zero to one, the higher the mean the brighter
			double brightness = limited(0.0, (height - min_height) / (max_height - min_height), 1.0);
			double redshift = 0.0;

			ROS_DEBUG_STREAM_THROTTLE(0.1, "diff: " << diff << " num: " << num_detections);
			if (detection > 0.5f)
			{
				redshift = 1.0; //limited(0.0, (detection - min_diff) / (max_diff - min_diff), 1.0);
				brightness = limited(0.0, detection, 1.0);
			}

			double r = brightness * redshift;
			double g = brightness * (1.0-redshift);
			double b = 0;
			wptr[0] = limited<uint8_t>(r * 255);  // R
			wptr[1] = limited<uint8_t>(g * 255);  // G
			wptr[2] = limited<uint8_t>(b * 255);  // B
			wptr[3] = 255; // Alpha
			wptr += 4;
			ridx++;
		}
	}
}

// filter those detections that are too big or small
void HeightImage::filterObstaclesBySize(const cv::Mat& prob_mat, int min_size_of_valid_obstacle, int max_size_of_valid_obstacle)
{
	// binarize prob_mat
	m_small_obstacles = 0;
	cv::Mat_<uint8_t > binarized_obstacle;
	binarized_obstacle.create(m_buckets_y, m_buckets_x);
	prob_mat.convertTo(binarized_obstacle, CV_8U, 255);
	cv::threshold(binarized_obstacle, binarized_obstacle, 0, 1, cv::THRESH_BINARY);

	// perform morphological opening to get rid of single pixel detections
	int morph_size = 1;
	cv::Mat element = cv::getStructuringElement( cv::MORPH_RECT, cv::Size( 2*morph_size + 1, 2*morph_size+1 ), cv::Point( morph_size, morph_size ) );
	cv::morphologyEx(binarized_obstacle, binarized_obstacle, cv::MORPH_OPEN, element );

	// find contours of segments
	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::findContours( binarized_obstacle, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE );

	// iterate through all the top-level contours,
	// fill contours if small enough, discard rest
	cv::Scalar obstacle_color(1);
	cv::Scalar no_obstacle_color(0);
	if(hierarchy.size() > 0)
	{
		for(int idx = 0; idx >= 0; idx = hierarchy[idx][0])
		{
			if(cv::contourArea(contours[idx]) < max_size_of_valid_obstacle &&
				cv::contourArea(contours[idx]) > min_size_of_valid_obstacle)
			{
				drawContours(m_small_obstacles, contours, idx, obstacle_color, CV_FILLED, 8, hierarchy);
			}
		}
	}
}
/*
void HeightImage::fillObstacleColorImage(sensor_msgs::Image* img, double min_height, double max_height, double min_diff, double max_diff, double height_diff_thresh)
{
	img->width = m_buckets_x;
	img->height = m_buckets_y;
	img->encoding = sensor_msgs::image_encodings::RGBA8;
	img->step = m_buckets_x * 4;
	img->data.resize(img->step * img->height);

	// filter those detections that are too big or small
	cv::Mat_<uint8_t > binarized_obstacle;
	binarized_obstacle.create(m_buckets_y, m_buckets_x);
	m_obstacle.convertTo(binarized_obstacle, CV_8U, 255);
	cv::threshold(binarized_obstacle, binarized_obstacle, 0, 1, cv::THRESH_BINARY);

	// perform morphological opening to get rid of single pixel detections
	int morph_size = 1;
	cv::Mat element = cv::getStructuringElement( cv::MORPH_RECT, cv::Size( 2*morph_size + 1, 2*morph_size+1 ), cv::Point( morph_size, morph_size ) );
	cv::morphologyEx(binarized_obstacle, binarized_obstacle, cv::MORPH_OPEN, element );

	// find contours of segments
	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::findContours( binarized_obstacle, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE );

	// iterate through all the top-level contours,
	// fill contours if small enough, discard rest
	int magic_small_obstacle_number = 244;
	int max_size_of_valid_obstacle = 20;
	int idx = 0;
	for( ; idx >= 0; idx = hierarchy[idx][0] )
	{
		//std::cout << "contour area with id " << idx << " " << cv::contourArea(contours[idx]) << std::endl;

		cv::Scalar obstacle_color(magic_small_obstacle_number);
		cv::Scalar no_obstacle_color(0);
		if(cv::contourArea(contours[idx]) < max_size_of_valid_obstacle && cv::contourArea(contours[idx]) > 0)
		{
			//std::cout << "    INSIDE contour area with id " << idx << " " << cv::contourArea(contours[idx]) << std::endl;
			drawContours( binarized_obstacle, contours, idx, obstacle_color, CV_FILLED, 8, hierarchy );
		}
		else
		{
			drawContours( binarized_obstacle, contours, idx, no_obstacle_color, CV_FILLED, 8, hierarchy );
		}
	}

	// colorize contour points of obstacles
//	for(int i = 0; i < contours.size(); i++)
//	{
//		for(int counter_point_counter = 0; counter_point_counter < contours[i].size(); counter_point_counter++)
//		{
//			cv::Point temp = contours[i][counter_point_counter];
//			binarized_obstacle(temp) = 244;
//		}
//	}

	int ridx = 0;
	uint8_t* wptr = img->data.data();
	for(int y = 0; y < m_buckets_y; ++y)
	{
		for(int x = 0; x < m_buckets_x; ++x)
		{
			double r = 0;
			double g = 0;
			double b = 0;

			double detection = m_obstacle(y, x);
			if(binarized_obstacle(y, x) == 0) detection = 0.0;
			double height = m_absolute(y, x);
			double diff = fabs(m_obstacle_max(y, x)-m_obstacle_min(y, x));


			if(!std::isfinite(height))
			{
				wptr[0] = wptr[1] = wptr[2] = 255;
//				wptr[3] = 0x55;
				wptr[3] = 255;
				wptr += 4;
				ridx++;
				continue;
			}

			// use normalized height as brightness value
			double relative_height = limited(0.0, (height - min_height) / (max_height - min_height), 1.0);

			
			double redshift = 0.0;

			// pixels without detection are colorized in shades of green
			if(!std::isfinite(detection) || detection == 0.0)
			{
				detection = 0.0;
				g = relative_height;
			}
			else
			{

				// TODO: filter detections that are not on the ground (median of local neighborhood or so)

				// TODO: filter detections with a low probability

			   if (diff < height_diff_thresh && binarized_obstacle(y, x) == magic_small_obstacle_number)
			   {
					redshift = limited(0.0, detection, 1.0);
					r = redshift;
			   }
			}


// 			double y = std::max(0.0, std::min(1.0, (height - min_height) / (max_height - min_height)));
// 			double u = limited(-1.0, 2.0 * detection / max_diff - 1.0, 1.0);
// 			double v = 0;
//
// 			double b = y + u / 0.493;
// 			double r = y + v / 0.877;
// 			double g = 1.704 * y - 0.509 * r - 0.194 * b;


			wptr[0] = limited<uint8_t>(r * 255);  // R
			wptr[1] = limited<uint8_t>(g * 255);  // G
			wptr[2] = limited<uint8_t>(b * 255);  // B
			wptr[3] = 0x55; // Alpha

			wptr += 4;
			ridx++;
		}
	}
}*/

void HeightImage::setSensorCenter(double x, double y)
{
	m_sensorCenter = cv::Point(
		m_mid_x + x / m_res_x,
		m_mid_y + y / m_res_y
	);
	resizeStorage();
}

void HeightImage::clearCircle( double x, double y, double radius ) {

	int bin_x = x / m_res_x;
	int bin_y = m_buckets_y - y / m_res_y;

	cv::circle( m_relative, cv::Point( bin_x, bin_y ), radius / m_res_x, cv::Scalar(0,0,0), -1 );

}

void HeightImage::clearRobot(float value)
{
	for(int y = 0; y < m_relative.rows; ++y)
	{
		for(int x = 0; x < m_relative.cols; ++x)
		{
			if(m_mask(y,x) != 0.0)
			{
				m_relative(y,x) = value;
				m_absolute(y,x) = NAN;
			}
		}
	}
}

void HeightImage::process(const cv::Mat& height_image, double min_height, double max_height, unsigned char source)
{
	for (int row = 0; row < height_image.rows; row++)
	{
		const unsigned char *p = height_image.ptr(row);
		int bin_y = static_cast<float>(row) / static_cast<float>(height_image.rows) * m_buckets_y;

		for (int col = 0; col < height_image.cols; col++, p++)
		{
			int bin_x = static_cast<float>(col) / static_cast<float>(height_image.cols) * m_buckets_x;

			if (bin_x < 0 || bin_x >= m_buckets_x || bin_y < 0 || bin_y >= m_buckets_y)
				continue;

			float* binval = &m_absolute(bin_y, bin_x);
			float pt_value = min_height + (static_cast<float>(*p) / 255.f) * (max_height - min_height);

			if (std::isnan(*binval) || pt_value > *binval)
			{
				*binval = pt_value;
				m_source(bin_y, bin_x) = source;
			}
		}
	}
}

//void HeightImage::medianFilter(float apertureSize)
//{
//	cv::Mat_<float> temp;
//	temp = ::hybridMedianFilter(m_absolute, apertureSize / m_res_x);
//// 	temp = ::medianFilter(m_absolute, apertureSize / m_res_x);
//
//	m_absolute = temp;
//}

std::vector<float> HeightImage::getCostsStorage() const
{
	std::vector<float> ret(m_relative.rows*m_relative.cols);

	for(int row = 0; row < m_relative.rows; ++row)
	{
		const float* ptr = m_relative[row];
		memcpy(ret.data() + m_relative.cols * (m_relative.rows - row - 1), ptr, sizeof(float)*m_relative.cols);
	}

	return ret;
}

// fill m_obstacle with some kind of detection probablity, which is the clamped sum of all detection probabilities of all points in one bin 
// fill m_obstacle_min with the min height of each bin TODO: check if its usefull to take only the points with a high detection value 
// fill m_obstacle_max with the max height of each bin TODO: check if its usefull to take only the points with a high detection value 
// fill m_obstacle_count with the number of points that were in one bin ever // TODO same as above + really ever? or should it be per scan?  
// fill m_obstacle_scans_count with the number of different scans that correspond to a bin // TODO same as above
void HeightImage::processObstaclesWithTransform(const pcl::PointCloud<velodyne_pointcloud::PointXYZIRDetection>& cloud, const Eigen::Affine3f& transform, float obstacle_thresh, float odds_hit, float odds_miss, float clamp_thresh_min, float clamp_thresh_max)
{
	for(typename pcl::PointCloud<velodyne_pointcloud::PointXYZIRDetection>::const_iterator it = cloud.begin(); it != cloud.end(); ++it)
	{
		const velodyne_pointcloud::PointXYZIRDetection& point = *it;

      // get corresponding bin
		Eigen::Vector3f pos(point.x, point.y, point.z);
		pos = transform * pos;
		int bin_x = pos.x() / m_res_x;
		int bin_y = m_buckets_y - pos.y() / m_res_y;

		if(bin_x < 0 || bin_x >= m_buckets_x || bin_y < 0 || bin_y >= m_buckets_y)
			continue;

		float* binval = &m_obstacle(bin_y, bin_x);
		float* binval_min = &m_obstacle_min(bin_y, bin_x);
		float* binval_max = &m_obstacle_max(bin_y, bin_x);
		int* binval_count = &m_obstacle_count(bin_y, bin_x);

      // update detection "probability" for this bin
 		if(std::isnan(*binval) )
 		{
		    *binval = point.detection;
		}
		else 
		{
		    if (point.detection > obstacle_thresh)
		    {
		      *binval += odds_hit;
		    }
		    else 
		      *binval -= odds_miss;
		    
		    std::min<float>(*binval, clamp_thresh_max);
		    std::max<float>(*binval, clamp_thresh_min);
		}

		// TODO: replace scanNr by something we have, maybe save timestamp and compare it to the current one
		// TODO: access elements as above 
		// check if this bin was seen in this scan 
//		if ( m_obstacle_last_scan_id(bin_y, bin_x) != point.scanNr )
//		{
//		    // update current scan number
//		    m_obstacle_last_scan_id(bin_y, bin_x) = point.scanNr;
//		    // increment number of scans that this bin was seen in
//		    m_obstacle_scans_count(bin_y, bin_x)++;
//		}
		
		// increment number of points that were assigned to this bin ever 
		binval_count++;
		
		//TODO: probably useless
		m_source(bin_y,bin_x) = 4;
		
		// update min and max height for this bin 
		if(std::isnan(*binval_min) || point.z < *binval_min)
		{
			*binval_min = point.z;
			m_source(bin_y,bin_x) = 4;
		}
		if(std::isnan(*binval_max) || point.z > *binval_max)
		{
			*binval_max = point.z;
			m_source(bin_y,bin_x) = 4;
		}
	}
}


}
