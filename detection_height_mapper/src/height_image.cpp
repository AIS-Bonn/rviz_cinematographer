// Omnidirectional height image
// Original author: Max Schwarz <max.schwarz@uni-bonn.de>
// Adapted by Jan Razlaw <s6jarazl@uni-bonn.de>

#include "height_image.h"

namespace detection_height_image
{

HeightImage::HeightImage()
 : m_res_x(0.05f)
 , m_res_y(0.05f)
 , m_length_x(8.0f)
 , m_length_y(8.0f)
 , m_robot_radius(0.42f)
 , m_min_height_threshold(std::numeric_limits<float>::lowest())
 , m_max_height_threshold(std::numeric_limits<float>::max())
 , m_min_object_height_threshold(std::numeric_limits<float>::lowest())
 , m_max_object_height_threshold(std::numeric_limits<float>::max())
 , m_min_footprint_size(0.f)
 , m_max_footprint_size(std::numeric_limits<float>::max())
 , m_max_object_altitude_threshold(std::numeric_limits<float>::max())
 , m_object_detection_threshold(0.9f)
 , m_max_neighborhood_height_threshold(std::numeric_limits<float>::max())
 , m_inflation_radius(0.25f)
 , m_debug_mode(false)
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

	m_object_median_height.create(m_buckets_y, m_buckets_x);
	m_min_height.create(m_buckets_y, m_buckets_x);
	m_max_height.create(m_buckets_y, m_buckets_x);
	m_object_detection.create(m_buckets_y, m_buckets_x);
	m_object_min_height.create(m_buckets_y, m_buckets_x);
	m_object_count.create(m_buckets_y, m_buckets_x);
	m_object_scans_count.create(m_buckets_y, m_buckets_x);
	m_objects_inflated.create(m_buckets_y, m_buckets_x);

	m_object_median_height = NAN;
	m_min_height = NAN;
	m_max_height = NAN;
	m_object_detection = NAN;
	m_object_min_height = NAN;
	m_objects_inflated = 0;
	m_object_count = 0;
	m_object_scans_count = 0;
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

void HeightImage::setRobotRadius(float robot_radius)
{
   m_robot_radius = robot_radius;
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

void HeightImage::setMaxNeighborhoodHeight(float max_neighborhood_height)
{
   m_max_neighborhood_height_threshold = max_neighborhood_height;
}

void HeightImage::setInflationRadius(float inflation_radius)
{
   m_inflation_radius = inflation_radius;
}

void HeightImage::setDebug(int debug_mode)
{
   m_debug_mode = debug_mode;
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
// of the points in each bin + fill m_object_median_height with median height for each bin
void HeightImage::processPointcloud(const InputPointCloud& cloud,
                                    const Eigen::Affine3f& transform,
                                    float odds_hit,
                                    float odds_miss,
                                    float clamp_thresh_min,
                                    float clamp_thresh_max)
{
   std::vector<std::vector<float> > storage_z(m_buckets_x * m_buckets_y);
   std::vector<std::set<uint16_t> > storage_scan_ids(m_buckets_x * m_buckets_y);

   for(const auto& point : cloud.points)
   {
      if(point.z < m_min_height_threshold || point.z > m_max_height_threshold)
         continue;

      // transform point and compute corresponding bin
      Eigen::Vector3f pos(point.x, point.y, point.z);
      pos = transform * pos;
      int bin_x = pos.x() / m_res_x;
      int bin_y = m_buckets_y - pos.y() / m_res_y;

      if(bin_x < 0 || bin_x >= m_buckets_x || bin_y < 0 || bin_y >= m_buckets_y)
         continue;
      
      unsigned int idx = bin_y * m_buckets_x + bin_x;

      // compute min and max height for current bin
      float* binval_min_height = &m_min_height(bin_y, bin_x);
      float* binval_max_height = &m_max_height(bin_y, bin_x);
      if(std::isnan(*binval_min_height) || point.z < *binval_min_height)
      {
         *binval_min_height = point.z;
      }
      if(std::isnan(*binval_max_height) || point.z > *binval_max_height)
      {
         *binval_max_height = point.z;
      }

      float* binval_object_min_height = &m_object_min_height(bin_y, bin_x);
      int* binval_object_count = &m_object_count(bin_y, bin_x);
      float* binval_detection = &m_object_detection(bin_y, bin_x);

      //if first point for bin and probably an object, copy values
      if(std::isnan(*binval_detection) )
      {
         if(point.detection > m_object_detection_threshold)
         {
            *binval_detection = point.detection;
            *binval_object_min_height = point.z;
            *binval_object_count = 1;

            storage_z[idx].push_back(point.z);
            storage_scan_ids[idx].insert(point.scan_id);
            continue;
         }
      }

      // compute min and max height for objects
      if(point.detection > m_object_detection_threshold)
      {
         *binval_detection += odds_hit;

         // update min and max height of object for this bin
         if(point.z < *binval_object_min_height)
         {
            *binval_object_min_height = point.z;
         }

         *binval_object_count += 1;

         storage_scan_ids[idx].insert(point.scan_id);
      }
      else // TODO if resolution is too high and cells are too big objects are not detected due to fixed odds_miss
         *binval_detection -= odds_miss;

      // TODO maybe clamp afterwards to not depend on the order of detections vs non-detections
      *binval_detection = std::min<float>(*binval_detection, clamp_thresh_max);
      *binval_detection = std::max<float>(*binval_detection, clamp_thresh_min);
   }

   // compute median height for each bin
   unsigned int idx = 0;
   for(int bin_y = 0; bin_y < m_buckets_y; ++bin_y)
   {
      for(int bin_x = 0; bin_x < m_buckets_x; ++bin_x, ++idx)
      {
         // TODO evtl check if they are all current scans or very old ones
         // update number of scans this object was seen in 
         m_object_scans_count(bin_y, bin_x) = storage_scan_ids[idx].size();

         std::vector<float>& points = storage_z[idx];

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
            m_object_median_height(bin_y, bin_x) = (points[n]+(*it))/2;
         }
         else
         {
            m_object_median_height(bin_y, bin_x) = points[n];
         }
      }
   }
}

void HeightImage::detectObjects(int min_number_of_object_points_per_cell,
                                int min_number_of_object_scans_per_cell,
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
			float *detection = &m_object_detection(y, x);
			int* scans_count = &m_object_scans_count(y, x);
         int* object_count = &m_object_count(y, x);

         // compute object height in a more robust way
         float object_height = (m_object_median_height(y, x) - m_object_min_height(y, x)) * 2.f;
			float object_altitude = m_object_min_height(y, x) - m_min_height(y, x);
			
			if(std::isfinite(*detection))
			{
            if(m_debug_mode)
            {
               if(*detection > m_object_detection_threshold)
                  m_objects_inflated(y, x) = m_objects_inflated(y, x) | 0b00000001;
               // TODO: check for bugs
               if(object_height < m_max_object_height_threshold)
                  m_objects_inflated(y, x) = m_objects_inflated(y, x) | 0b00000010;
               if(object_height > m_min_object_height_threshold)
                  m_objects_inflated(y, x) = m_objects_inflated(y, x) | 0b00000100;
               if(object_altitude < m_max_object_altitude_threshold)
                  m_objects_inflated(y, x) = m_objects_inflated(y, x) | 0b00001000;
               if(*object_count > min_number_of_object_points_per_cell)
                  m_objects_inflated(y, x) = m_objects_inflated(y, x) | 0b00010000;
               if(*scans_count > min_number_of_object_scans_per_cell)
                  m_objects_inflated(y, x) = m_objects_inflated(y, x) | 0b00100000;
            }
            else
            {
               if(*detection > m_object_detection_threshold
                  && object_height < m_max_object_height_threshold
                  && object_height > m_min_object_height_threshold
                  && object_altitude < m_max_object_altitude_threshold
                  && *object_count > min_number_of_object_points_per_cell
                  && *scans_count > min_number_of_object_scans_per_cell)
               {
                  m_objects_inflated(y, x) = 1;
               }
            }
			}
		}
	}

   // set bins to zero if height difference in neighborhood exceeds a threshold
   // should prevent false positives that are too near to a wall or big object
   filterObjectsByNeighborHeight(m_objects_inflated, m_robot_radius, m_max_neighborhood_height_threshold);

   if(inflate_objects)
      inflateObjects(m_objects_inflated, m_inflation_radius);
	
	// filter objects which have a size between min and max size and
   float area_of_one_cell = m_res_x * m_res_y;
   int min_number_of_cells = (int)std::round(m_min_footprint_size/area_of_one_cell);
   int max_number_of_cells = (int)std::ceil(m_max_footprint_size/area_of_one_cell);
   if(m_debug_mode)
      filterObjectsBySize(m_objects_debug, min_number_of_cells, max_number_of_cells);
   else
      filterObjectsBySize(m_objects_inflated, min_number_of_cells, max_number_of_cells);
}

void HeightImage::fillObjectMap(nav_msgs::OccupancyGrid* map)
{
	map->info.height = m_buckets_y;
	map->info.width = m_buckets_x;

	map->info.resolution = m_res_x;
	map->info.origin.position.x = -m_res_x * (m_buckets_x/2);
	map->info.origin.position.y = -m_res_y * (m_buckets_y/2);
	map->info.origin.position.z = -1.f;
	map->info.origin.orientation.w = 1;

	map->data.resize(m_buckets_x * m_buckets_y);

	int8_t* wptr = map->data.data();
	for(unsigned int y = 0; y < map->info.height; ++y)
	{
		for(unsigned int x = 0; x < map->info.width; ++x)
		{
         int* detection_binary = &m_objects_inflated(m_buckets_y - y - 1, x);
         float* detection = &m_object_detection(m_buckets_y - y - 1, x);

			if(*detection_binary == 0)
				*wptr = (int8_t)0 ;//-1; // Unknown
			else
			{
  			   if (*detection >= m_object_detection_threshold)
  			   {
			     *wptr = (int8_t)(129);
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

void HeightImage::fillObjectColorImage(sensor_msgs::ImagePtr img)
{
   cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage());
   cv_ptr->encoding = sensor_msgs::image_encodings::RGBA8;
   cv_ptr->image.create(cv::Size(m_buckets_x, m_buckets_y), CV_8UC4);
   cv_ptr->image.setTo(255);

   // create simple height image
	for(int y = 0; y < m_buckets_y; ++y)
	{
		for(int x = 0; x < m_buckets_x; ++x)
		{
			float* height = &m_max_height(y, x);

			// background color if height in bin is not valid
			if(!std::isfinite(*height))
			{
				continue;
			}

			// for non object pixels
			// scale height and cap to zero to one, the higher the mean the brighter
			float brightness = limited(0.f, (*height - m_min_height_threshold) / (m_max_height_threshold - m_min_height_threshold), 1.f);

         cv_ptr->image.at<cv::Vec4b>(y, x) = cv::Scalar(0,limited<uint8_t>(brightness * 255), 0, 255);

         if(m_debug_mode && m_objects_debug(y, x) > 0)
         {
            uint8_t filter = 0b00000001 << (m_debug_mode - 1);
            if(!(m_objects_debug(y, x) & filter))
               cv_ptr->image.at<cv::Vec4b>(y, x) = cv::Scalar(255, 0, 0, 255);
         }
      }
	}

   if(m_debug_mode)
   {
      for(const auto& mean_pixel: m_mean_object_pixels)
      {
         cv::circle(cv_ptr->image, mean_pixel, (int)(1.f / m_res_x) , cv::Scalar(255, 128, 0, 255), 3, 8, 0);
      }
   }
   else
   {
      cv::RNG rng( 0xFFFFFFFF );
      for(const auto& mean_pixel: m_mean_object_pixels)
      {
         // colorize objects in height image with different colors
         cv::circle(cv_ptr->image, mean_pixel, (int)(1.f / m_res_x) , cv::Scalar(rng.uniform(0, 255), 0, rng.uniform(0, 255), 255), -1, 8, 0);
      }
   }

   cv_ptr->toImageMsg(*img);
}

void HeightImage::getObjectPositions(std::vector<detection_height_mapper::ObjectPosition>& object_positions,
                                     const Eigen::Affine3f& transform)
{
   object_positions.clear();
   object_positions.reserve(m_mean_object_pixels.size());

   for(unsigned int id = 0; id < m_mean_object_pixels.size(); id++)
   {
      detection_height_mapper::ObjectPosition object_position;

      // retrieve position in point cloud frame and publish
      Eigen::Vector3f pos;
      pos.x() = m_mean_object_pixels[id].x * m_res_x;
      pos.y() = (m_buckets_y - m_mean_object_pixels[id].y) * m_res_y;
      pos.z() = m_max_height(m_mean_object_pixels[id].y, m_mean_object_pixels[id].x);
      pos = transform * pos;

      ROS_DEBUG_STREAM("position of point " << pos.x() << " " << pos.y() << " " << pos.z() << " " );

      object_position.position.point.x = pos.x();
      object_position.position.point.y = pos.y();
      object_position.position.point.z = pos.z();
      object_position.id = id;
      object_position.detection_certainty = m_object_detection(m_mean_object_pixels[id].y, m_mean_object_pixels[id].x);

      object_positions.push_back(object_position);
   }
}

// filter those detections that are too big or small
void HeightImage::filterObjectsBySize(cv::Mat& prob_mat,
                                      int min_size_of_valid_object,
                                      int max_size_of_valid_object)
{
   // binarize prob_mat
	cv::Mat_<uint8_t> binarized_object;
	prob_mat.convertTo(binarized_object, CV_8U);
   if(m_debug_mode)
   {
      cv::threshold(binarized_object, binarized_object, 126, 255, cv::THRESH_TOZERO);
      cv::threshold(binarized_object, binarized_object, 0, 1, cv::THRESH_BINARY);
   }
   else
      cv::threshold(binarized_object, binarized_object, 0, 1, cv::THRESH_BINARY);

	// perform morphological opening to get rid of single pixel detections
//	int morph_size = 1;
//	cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2*morph_size + 1, 2*morph_size+1), cv::Point(morph_size, morph_size));
//	cv::morphologyEx(binarized_object, binarized_object, cv::MORPH_OPEN, element);

	// find contours of segments
	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::findContours(binarized_object, contours, hierarchy, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);

	// iterate through all the top-level contours,
	// fill contours if size fits + save mean positions, discard rest
   cv::Scalar object_color(1);
   cv::Scalar scalar_one(1);
	cv::Scalar no_object_color(0);
	if(hierarchy.size() > 0)
	{
      for(int i = 0; i < (int)contours.size(); i++)
      {
         if(cv::contourArea(contours[i]) <= max_size_of_valid_object &&
            cv::contourArea(contours[i]) >= min_size_of_valid_object)
         {
            if(!m_debug_mode)
               drawContours(prob_mat, contours, i, object_color, CV_FILLED, 8, hierarchy);

            cv::Point2i mean_object_pixel(0, 0);
            for(int point_index = 0; point_index < (int)contours[i].size(); point_index++)
            {
               mean_object_pixel.x += contours[i][point_index].x;
               mean_object_pixel.y += contours[i][point_index].y;

               if(m_debug_mode)
               {
                  int row = contours[i][point_index].y;
                  int col = contours[i][point_index].x;
                  prob_mat.at<int>(row, col) = prob_mat.at<int>(row, col) | 0b10000000;
               }
            }
            mean_object_pixel.x /= contours[i].size();
            mean_object_pixel.y /= contours[i].size();
            m_mean_object_pixels.push_back(mean_object_pixel);

            object_color += scalar_one;
         }
         else
         {
            if(!m_debug_mode)
               drawContours(prob_mat, contours, i, no_object_color, CV_FILLED, 8, hierarchy);
         }
      }

      ROS_DEBUG_STREAM("number of found objects: " << m_mean_object_pixels.size());
	}
}

// set bins to zero if height difference in neighborhood exceeds a threshold
// should prevent false positives that are too near to a wall or big object
void HeightImage::filterObjectsByNeighborHeight(cv::Mat& prob_mat,
                                                float robot_radius,
                                                float height_threshold)
{
   float robot_radius_in_bins = robot_radius/m_res_x;
   int ROBOT_RADIUS_INT = (int)ceil(robot_radius_in_bins);

   for(int row = 0; row < prob_mat.rows; ++row)
   {
      for(int col = 0; col < prob_mat.cols; ++col)
      {
         if(prob_mat.at<int>(row, col) < 1)
            continue;

         float local_min_height = std::numeric_limits<float>::max();
         float local_max_height = std::numeric_limits<float>::lowest();
         // compute local min and max height
         for(int local_row = std::max(0, row - ROBOT_RADIUS_INT); local_row < std::min(prob_mat.rows, row + ROBOT_RADIUS_INT); ++local_row)
         {
            for(int local_col = std::max(0, col - ROBOT_RADIUS_INT); local_col < std::min(prob_mat.cols, col + ROBOT_RADIUS_INT); ++local_col)
            {
               // check if bin is within the robot radius
               int col_diff = col - local_col;
               int row_diff = row - local_row;
               if(col_diff*col_diff + row_diff*row_diff > robot_radius_in_bins*robot_radius_in_bins)
                  continue;

               if((!std::isnan(m_min_height(local_row, local_col))) && m_min_height(local_row, local_col) < local_min_height)
               {
                  local_min_height = m_min_height(local_row, local_col);
               }
               if((!std::isnan(m_max_height(local_row, local_col))) && m_max_height(local_row, local_col) > local_max_height)
               {
                  local_max_height = m_max_height(local_row, local_col);
               }
            }
         }

         // filter
         double neighborhood_height = local_max_height - local_min_height;
         if(neighborhood_height > height_threshold)
         {
            if(!m_debug_mode)
               prob_mat.at<int>(row, col) = 0;
         }
         else
         {
            if(m_debug_mode)
               prob_mat.at<int>(row, col) = prob_mat.at<int>(row, col) | 0b01000000;
         }
      }
   }
}

void HeightImage::inflateObjects(cv::Mat& prob_mat,
                                 float inflation_radius)
{
   std::vector<cv::Point3i> points_to_inflate;

   // radius in number of bins
   inflation_radius /= m_res_x;

   for(int y = 0; y < m_objects_inflated.rows; ++y)
   {
      for(int x = 0; x < m_objects_inflated.cols; ++x)
      {
         int local_thresh = 1;
         if(m_debug_mode)
            local_thresh = 127;

         if(m_objects_inflated.at<int>(y, x) < local_thresh)
            continue;

         points_to_inflate.push_back(cv::Point3i(y, x, m_objects_inflated.at<int>(y, x)));
      }
   }

   if(m_debug_mode)
   {
      m_objects_debug = m_objects_inflated.clone();
      for(const auto &point : points_to_inflate)
         cv::circle(m_objects_debug, cv::Point(point.y, point.x), inflation_radius, cv::Scalar(point.z), -1, 8, 0);
   }
   else
   {
      for(const auto &point : points_to_inflate)
         cv::circle(m_objects_inflated, cv::Point(point.y, point.x), inflation_radius, cv::Scalar(1), -1, 8, 0);
   }
}

}
