// Omnidirectional height image
// Original author: Max Schwarz <max.schwarz@uni-bonn.de>
// Adapted by Jan Razlaw <s6jarazl@uni-bonn.de>

#include "height_image.h"

namespace momaro_heightmap
{

HeightImage::HeightImage()
 : m_res_x(0.05f)
 , m_res_y(0.05f)
 , m_length_x(8.0f)
 , m_length_y(8.0f)
 , m_robotRadius(0.42f)
 , m_min_height_threshold(std::numeric_limits<float>::lowest())
 , m_max_height_threshold(std::numeric_limits<float>::max())
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

	m_median_height.create(m_buckets_y, m_buckets_x);
	m_min_height.create(m_buckets_y, m_buckets_x);
	m_max_height.create(m_buckets_y, m_buckets_x);
	m_object_detection.create(m_buckets_y, m_buckets_x);
	m_object_min_height.create(m_buckets_y, m_buckets_x);
	m_object_max_height.create(m_buckets_y, m_buckets_x);
	m_object_count.create(m_buckets_y, m_buckets_x);
	m_object_last_scan_id.create(m_buckets_y, m_buckets_x);
	m_object_scans_count.create(m_buckets_y, m_buckets_x);
	m_objects_inflated.create(m_buckets_y, m_buckets_x);
	m_relative.create(m_buckets_y, m_buckets_x);
	m_mask.create(m_buckets_y, m_buckets_x);
	m_source.create(m_buckets_y, m_buckets_x);
	m_small_objects.create(m_buckets_y, m_buckets_x);

	// Setup robot mask
	m_mask.setTo(cv::Scalar(0.0f));

	m_median_height = NAN;
	m_min_height = NAN;
	m_max_height = NAN;
	m_object_detection = NAN;
	m_object_min_height = NAN;
	m_object_max_height = NAN;
	m_objects_inflated = NAN;
	m_object_count = 0;
	m_object_scans_count = 0;
	m_object_last_scan_id = 0;
	m_small_objects = 0;
}

void HeightImage::setResolution(float res_x, float res_y)
{
	m_res_x = res_x;
	m_res_y = res_y;
	resizeStorage();
}

void HeightImage::setSize(float size_x, float size_y)
{
	m_length_x = size_x;
	m_length_y = size_y;
	resizeStorage();
}

void HeightImage::setMinHeight(float min_height)
{
   m_min_height_threshold = min_height;
}

void HeightImage::setMaxHeight(float max_height)
{
   m_max_height_threshold = max_height;
}

void HeightImage::setMinObjectHeight(float min_height)
{
   m_min_object_height_threshold = min_height;
}

void HeightImage::setMaxObjectHeight(float max_height)
{
   m_max_object_height_threshold = max_height;
}

void HeightImage::setMinObjectFootprint(float min_footprint_size)
{
   m_min_footprint_size = min_footprint_size;
}

void HeightImage::setMaxObjectFootprint(float max_footprint_size)
{
   m_max_footprint_size = max_footprint_size;
}

void HeightImage::setMaxObjectAltitude(float max_altitude)
{
   m_max_object_altitude_threshold = max_altitude;
}

void HeightImage::setDetectionThreshold(float detection_threshold)
{
   m_object_detection_threshold = detection_threshold;
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

// use transformed point cloud to fill m_min_height and -max with min and max height (z-coordinate)
// of the points in each bin + fill m_median_height with median height for each bin
void HeightImage::processPointcloud(const InputPointCloud& cloud,
                                    const Eigen::Affine3f& transform,
                                    float odds_hit,
                                    float odds_miss,
                                    float clamp_thresh_min,
                                    float clamp_thresh_max)
{
   std::vector<std::vector<float> > storage(m_buckets_x * m_buckets_y);

   for(const auto& point : cloud.points)
   {
      // transform point and compute corresponding bin
      Eigen::Vector3f pos(point.x, point.y, point.z);
      pos = transform * pos;
      int bin_x = pos.x() / m_res_x;
      int bin_y = m_buckets_y - pos.y() / m_res_y;

      if(bin_x < 0 || bin_x >= m_buckets_x || bin_y < 0 || bin_y >= m_buckets_y)
         continue;

      unsigned int idx = bin_y * m_buckets_x + bin_x;

      storage[idx].push_back(point.z);

      // compute min and max height for current bin
      float* binval_min_height = &m_min_height(bin_y, bin_x);
      float* binval_max_height = &m_max_height(bin_y, bin_x);
      if((std::isnan(*binval_min_height) || point.z < *binval_min_height) && point.z > m_min_height_threshold)
      {
         *binval_min_height = point.z;
      }
      if((std::isnan(*binval_max_height) || point.z > *binval_max_height) && point.z < m_max_height_threshold)
      {
         *binval_max_height = point.z;
      }

      float* binval_object_min_height = &m_object_min_height(bin_y, bin_x);
      float* binval_object_max_height = &m_object_max_height(bin_y, bin_x);
      int* binval_object_count = &m_object_count(bin_y, bin_x);
      float* binval_detection = &m_object_detection(bin_y, bin_x);

      // TODO: implement in todo below
//      int* binval_object_last_scan_id = &m_object_last_scan_id(bin_y, bin_x);
//      int* binval_object_scans_count = &m_object_scans_count(bin_y, bin_x);

      //if first point for bin and probably an object, copy values
      if(std::isnan(*binval_detection) )
      {
         if(point.detection > m_object_detection_threshold)
         {
            *binval_detection = point.detection;
            *binval_object_min_height = point.z;
            *binval_object_max_height = point.z;
            *binval_object_count = 1;

            // TODO: implement in todo below
//            *binval_object_last_scan_id = point.scanNr;
//            *binval_object_scans_count = 1;
            continue;
         }
      }

      // compute min and max height for objects
      if(point.detection > m_object_detection_threshold)
      {
         *binval_detection += odds_hit;

         // update min and max height of object for this bin
         if(point.z < *binval_object_min_height && point.z > m_min_height_threshold)
         {
            *binval_object_min_height = point.z;
         }
         if(point.z > *binval_object_max_height && point.z < m_max_height_threshold)
         {
            *binval_object_max_height = point.z;
         }

         *binval_object_count += 1;

         // TODO access values as above + replace with a more accurate way to check to how many full scans
         //       this bins corresponds to + evtl if they are all current scans or very old ones
         //check if this bin was seen in this scan
//         if(*binval_object_last_scan_id != point.scanNr)
//         {
//             // update current scan number
//             *binval_object_last_scan_id = point.scanNr;
//             // increment number of scans that this bin was seen in
//             *binval_object_scans_count += 1;
//         }
      }
      else
         *binval_detection -= odds_miss;

      std::min<float>(*binval_detection, clamp_thresh_max);
      std::max<float>(*binval_detection, clamp_thresh_min);
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
            m_median_height(bin_y, bin_x) = (points[n]+(*it))/2;
         }
         else
         {
            m_median_height(bin_y, bin_x) = points[n];
         }
      }
   }
}

void HeightImage::detectObjects(int num_min_count,
                                bool inflate_objects)
{
   // if bins meet some criteria, set those bins to 1 in m_objects_inflated
	// criteria: detection "probability" above threshold
	//    object height between thresholds
	//    distance of object bottom to ground under threshold
	//    number of scans that this bin was seen in above threshold
	for(int y = 0; y < m_buckets_y; ++y)
	{
		for(int x = 0; x < m_buckets_x; ++x)
		{
			float detection = m_object_detection(y, x);
         // TODO currently not implemented, fix in the ProcessPointCloud function first
			//int num_detections = m_object_scans_count(y, x);

         float object_height = m_object_max_height(y, x) - m_object_min_height(y, x);
			float object_altitude = m_object_min_height(y, x) - m_min_height(y, x);
			
			if(std::isfinite(detection))
			{
				if(detection > m_object_detection_threshold
			     && object_height < m_max_object_height_threshold
			     && object_height > m_min_object_height_threshold
			     && object_altitude < m_max_object_altitude_threshold
			     /*&& num_detections > num_min_count*/)
			   {
					m_objects_inflated(y, x) = 1.f;
			   }
			}
			
		}
	}

   // TODO: test effects of inflation
   if(inflate_objects)
   {
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
      // 1st pass: inflate lethal objects
      for(int y = 0; y < m_objects_inflated.rows; ++y){
         for(int x = 0; x < m_objects_inflated.cols; ++x){
            bool cellFree = true;

            for(int dy = -INFLATION_RADIUS; dy <= INFLATION_RADIUS; ++dy){
               int _y = y + dy;

               if(_y < 0 || _y >= m_objects_inflated.rows)
                  continue;

               for(int dx = -INFLATION_RADIUS; dx <= INFLATION_RADIUS; ++dx){
                  int _x = x + dx;

                  if(_x < 0 || _x >= m_objects_inflated.cols)
                     continue;

                  if(dx * dx + dy * dy > hardRadius * hardRadius)
                     continue;

                  if(m_objects_inflated(_y, _x) > 0.999f){
                     cellFree = false;
                     break;
                  }
               }
            }

            // TODO: test effects of setting uninflated to zero instead of nan
            if(cellFree)
               uninflated(y, x) = m_objects_inflated(y, x);
            else
               uninflated(y, x) = 1.0f;
         }
      }

      // 2nd pass: inflate

      for(int y = 0; y < m_objects_inflated.rows; ++y){
         for(int x = 0; x < m_objects_inflated.cols; ++x){
            if(uninflated(y, x) >= 1.0){
               m_objects_inflated(y, x) = 1.0;
               continue; // Do not "eat" into absolute objects
            }

            float costSum = 0;
            float weightSum = 0;

//			float max = 0.0;

            for(int dy = -INFLATION_RADIUS_SOFT; dy <= INFLATION_RADIUS_SOFT; ++dy){
               int _y = y + dy;

               if(_y < 0 || _y >= m_objects_inflated.rows)
                  continue;

               for(int dx = -INFLATION_RADIUS_SOFT; dx <= INFLATION_RADIUS_SOFT; ++dx){
                  int _x = x + dx;

                  if(_x < 0 || _x >= m_objects_inflated.cols)
                     continue;

                  if(dx * dx + dy * dy > softRadius * softRadius)
                     continue;

                  float dist = hypotf(dx, dy);
                  // the bigger the distance, the smaller the response
                  // the fraction can be between 0 and 1
                  // TODO: uninflated is probably not necessary because its either 1 or nan
                  float response = (-(dist / softRadius) + 1.0) * uninflated(_y, _x);
//					if(response > max)
//					{
//						max = response;
//					}

                  float weight = 1.0;
                  // TODO: probably uninflated has to be switched with response
                  costSum += weight * uninflated(_y, _x);
                  weightSum += weight;
               }
            }

            m_objects_inflated(y, x) = costSum / weightSum;
         }
      }
   }
	
	// filter objects which have a size between min and max size and
   // save those in a binary mask named m_small_objects
   float area_of_one_cell = m_res_x * m_res_y;
   int min_number_of_cells = (int)std::ceil(m_min_footprint_size/area_of_one_cell);
   int max_number_of_cells = (int)std::ceil(m_max_footprint_size/area_of_one_cell);
	filterObjectsBySize(m_objects_inflated, min_number_of_cells, max_number_of_cells);

	// set bins to zero if height difference in neighborhood exceeds a threshold
	// should prevent false positives that are too near to a wall or big object
	float robot_radius = 2.5f;
	robot_radius /= m_res_x;
	const int ROBOT_RADIUS_INT = ceil(robot_radius);

	for(int y = 0; y < m_small_objects.rows; ++y)
	{
		for(int x = 0; x < m_small_objects.cols; ++x)
		{
			if(m_small_objects(y, x) == 0)
				continue;

			bool cell_canceled = false;
			float local_min_height = std::numeric_limits<float>::max();
         float local_max_height = std::numeric_limits<float>::lowest();

			// compute local min and max height
			for(int dy = -ROBOT_RADIUS_INT; dy <= ROBOT_RADIUS_INT; ++dy)
			{
				int _y = y+dy;

				if(_y < 0 || _y >= m_small_objects.rows)
					continue;

				for(int dx = -ROBOT_RADIUS_INT; dx <= ROBOT_RADIUS_INT; ++dx)
				{
					int _x = x+dx;

					if(_x < 0 || _x >= m_small_objects.cols)
						continue;

					if(dx*dx+dy*dy > robot_radius*robot_radius)
						continue;

					if(m_min_height(_y, _x) < local_min_height)
					{
						local_min_height = m_min_height(_y, _x);
					}
					if(m_max_height(_y, _x) > local_max_height)
					{
						local_max_height = m_max_height(_y, _x);
					}
				}
			}

			// filter
			double object_height  = fabs(local_max_height - local_min_height);
			if(object_height > 0.4)
			{
				m_small_objects(y, x) = 0;
			}
		}
	}

	m_objects_inflated =  m_small_objects.clone();
}

void HeightImage::fillObjectMap(nav_msgs::OccupancyGrid* map)
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
			double v = m_objects_inflated(m_buckets_y - y - 1, x) ;

			double detection = m_objects_inflated(m_buckets_y - y - 1, x);
			int num_detections = m_object_scans_count(m_buckets_y - y - 1, x);
			double height = m_median_height(m_buckets_y - y - 1, x);
			double object_height = fabs(m_object_max_height(m_buckets_y - y - 1, x)-m_object_min_height(m_buckets_y - y - 1, x));
			double object_altitude = fabs(m_object_max_height(m_buckets_y - y - 1, x)-m_min_height(m_buckets_y - y - 1, x));
			
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

void HeightImage::fillObjectColorImage(sensor_msgs::Image* img)
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
			double detection = m_objects_inflated(y, x);
			double height = m_median_height(y, x);

			// background color if height in bin is not valid
			if(!std::isfinite(height))
			{
				wptr[0] = wptr[1] = wptr[2] = 255;
				wptr[3] = 255;
				wptr += 4;
				ridx++;
				continue;
			}

			// for non object pixels
			// scale height and cap to zero to one, the higher the mean the brighter
			double brightness = limited(0.0, (height - m_min_height_threshold) / (m_max_height_threshold - m_min_height_threshold), 1.0);
			double redshift = 0.0;

			if (detection >= m_object_detection_threshold)
			{
            // TODO more sophisticated colorization
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
void HeightImage::filterObjectsBySize(const cv::Mat& prob_mat,
                                      int min_size_of_valid_object,
                                      int max_size_of_valid_object)
{
	// binarize prob_mat
	m_small_objects = 0;
	cv::Mat_<uint8_t> binarized_object;
	binarized_object.create(m_buckets_y, m_buckets_x);
	prob_mat.convertTo(binarized_object, CV_8U, 255);
	cv::threshold(binarized_object, binarized_object, 0, 1, cv::THRESH_BINARY);

	// perform morphological opening to get rid of single pixel detections
	int morph_size = 1;
	cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2*morph_size + 1, 2*morph_size+1), cv::Point(morph_size, morph_size));
	cv::morphologyEx(binarized_object, binarized_object, cv::MORPH_OPEN, element);

	// find contours of segments
	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::findContours(binarized_object, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);

	// iterate through all the top-level contours,
	// fill contours if small enough, discard rest
	cv::Scalar object_color(1);
	cv::Scalar no_object_color(0);
	if(hierarchy.size() > 0)
	{
		for(int idx = 0; idx >= 0; idx = hierarchy[idx][0])
		{
			if(cv::contourArea(contours[idx]) < max_size_of_valid_object &&
				cv::contourArea(contours[idx]) > min_size_of_valid_object)
			{
				drawContours(m_small_objects, contours, idx, object_color, CV_FILLED, 8, hierarchy);
			}
		}
	}
}

}
