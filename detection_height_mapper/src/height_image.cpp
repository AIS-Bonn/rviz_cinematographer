// Omnidirectional height image
// Original author: Max Schwarz <max.schwarz@uni-bonn.de>
// Adapted by Jan Razlaw <s6jarazl@uni-bonn.de>

#include "height_image.h"

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

// fill m_obstacle with some kind of detection probablity, which is the clamped sum of all detection probabilities of all points in one bin 
// fill m_obstacle_min with the min height of each bin TODO: check if its usefull to take only the points with a high detection value 
// fill m_obstacle_max with the max height of each bin TODO: check if its usefull to take only the points with a high detection value 
// fill m_obstacle_count with the number of points that were in one bin ever // TODO same as above + really ever? or should it be per scan?  
// fill m_obstacle_scans_count with the number of different scans that correspond to a bin // TODO same as above
void HeightImage::processObstaclesWithTransform(const InputPointCloud& cloud,
                                                const Eigen::Affine3f& transform,
                                                float obstacle_thresh,
                                                float odds_hit,
                                                float odds_miss,
                                                float clamp_thresh_min,
                                                float clamp_thresh_max)
{
	for(typename InputPointCloud::const_iterator it = cloud.begin(); it != cloud.end(); ++it)
	{
		const InputPoint& point = *it;

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

// use transformed point cloud to fill m_absolute_min and -max with min and max height (z-coordinate)
// of the points in each bin + fill m_absolute with median height for each bin
void HeightImage::processMedianFiltered(const InputPointCloud& cloud, const Eigen::Affine3f& transform)
{
   std::vector<std::vector<float> > storage(m_buckets_x * m_buckets_y);

   for(typename InputPointCloud::const_iterator it = cloud.begin(); it != cloud.end(); ++it)
   {
      const InputPoint& point = *it;

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
