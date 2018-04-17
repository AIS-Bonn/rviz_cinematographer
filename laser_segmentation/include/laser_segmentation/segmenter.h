/* -*- mode: C++ -*- */
/** @file
 *
 * This class segments objects of a specified width in laser point clouds
 *
 * @author Jan Razlaw
 */

#ifndef _SEGMENTER_H_
#define _SEGMENTER_H_ 1

#include <functional>

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <laser_segmentation/point_type.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <laser_geometry/laser_geometry.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h>
#include <pcl/conversions.h>
#include <pcl/visualization/histogram_visualizer.h>
#include <pcl/visualization/pcl_plotter.h>

#include <boost/circular_buffer.hpp>
#include <boost/thread/mutex.hpp>

#include <config_server/parameter.h>


namespace laser_segmentation
{

/**
 *  Class to segment objects of a specified width in laser point clouds.
 */
class Segmenter
{
public:
	typedef velodyne_pointcloud::PointXYZIDR              	InputPoint;
	typedef velodyne_pointcloud::PointXYZIRSegmentation   	DebugOutputPoint;
	typedef velodyne_pointcloud::PointXYZIdsSegment     		OutputPoint;

	typedef pcl::PointCloud<InputPoint>                			InputPointCloud;
	typedef pcl::PointCloud<DebugOutputPoint>          			DebugOutputPointCloud;
	typedef pcl::PointCloud<OutputPoint>               			OutputPointCloud;

	typedef typename boost::circular_buffer<InputPoint>::iterator 				buffer_iterator;
	typedef typename boost::circular_buffer<InputPoint>::const_iterator 	buffer_const_iterator;

	typedef boost::circular_buffer<InputPoint> BufferInputPoints;
	typedef std::shared_ptr<BufferInputPoints> BufferInputPointsPtr;

	/** Point and additional information from the filters. */
	struct MedianFiltered {
		MedianFiltered(InputPoint p):point(p){};
		MedianFiltered(){};
		InputPoint point; 						///< 3D point + intensity + distance + ring
		float dist_noise_kernel;			///< result of noise filter on distances
		float dist_object_kernel;			///< result of object filter on distances
		float intens_noise_kernel;		///< result of noise filter on intensities
		float intens_object_kernel;		///< result of object filter on intensities
	}; EIGEN_ALIGN16;

	typedef boost::circular_buffer<MedianFiltered> 	BufferMedians;
	typedef std::shared_ptr<BufferMedians> 					BufferMediansPtr;
	typedef typename BufferMedians::iterator 				median_iterator;
	typedef typename BufferMedians::const_iterator 	median_const_iterator;

	/**
	 * @brief Constructor.
	 *
	 * Parameters are needed to use this class within a ros node or nodelet.
	 */
	Segmenter(ros::NodeHandle node, ros::NodeHandle private_nh);
	/** @brief Destructor. */
	~Segmenter(){};

	/** @brief Waits for callbacks to finish before changing parameters.*/
	void changeParameterSavely();

	/** @brief Set up circular buffers and iterators. Number depends on sensor.*/
	void initBuffer(int number_of_rings);
	/** @brief Clear circular buffers and iterators. */
	void resetBuffer();

	/**
	 * @brief Process one scan from a Hokuyo laser range scanner.
	 *
	 * Transform one 2D scan line from the sensor's frame to a 3D point cloud
	 * in the base_link.
	 * Feeds those points to the circular buffer and starts the segmentation.
	 *
	 * @param[in] input_scan 	one scan line from the Hokuyo.
	 */
	void hokuyoCallback(const sensor_msgs::LaserScanConstPtr& input_scan);

	/**
	 * @brief Process one scan from a Velodyne laser range scanner.
	 *
	 * Feeds points from scan rings to the corresponding circular buffers and
	 * starts the segmentation.
	 *
	 * @param[in] input_cloud 	one scan from the Velodyne.
	 */
	void velodyneCallback(const InputPointCloud::ConstPtr &input_cloud);

	/**
	 * @brief Segment the current points in the circular buffers.
	 *
	 * Applies the noise and object filters to the points in the circular buffers
	 * and computes the segmentation probabilities depending on the results.
	 * Publishes the resulting cloud in the end.
	 *
	 * @param[in] header 	header of the currently processed point's cloud, needed for publishing.
	 */
	void processScan(pcl::PCLHeader header);

	/**
	 * @brief Apply the noise and object filters to the unfiltered points in the buffers.
	 *
	 * For each unfiltered point in the buffers, compute the kernel sizes wrt. the distance
	 * and apply the noise and the object median filters.
	 *
	 * @param[in,out] buffer_median_filtered 	circular buffer containing the filtered and unfiltered points.
	 * @param[in,out] iter 										iterator pointing to the first unfiltered point that should be filtered next.
	 */
	void filterRing(std::shared_ptr<boost::circular_buffer<MedianFiltered> > buffer_median_filtered,
								 median_iterator& iter);

	/**
	 * @brief Apply the noise and object filters to the current point.
	 *
	 * Fills a vector with the distance values of the neighbors of the current point.
	 * Vector has the size of the object median filter kernel.
	 * Computes noise median filter result first, as the vector is partly sorted during the process.
	 * Computes the object median filter result afterwards.
	 *
	 * @param[in] noise_filter_kernel_size 					kernel size of the noise filter.
	 * @param[in] object_filter_kernel_size 				kernel size of the object filter.
	 * @param[in] buffer 														pointer to circular buffer.
	 * @param[in] current_element 									iterator pointing to current point.
	 * @param[in] f 	 															function to get either the distance value or the intensity value of the point.
	 * @param[in] max_dist_for_median_computation 	threshold that can optionally be used to discard neighbors if their distance difference to the current point is too high.
	 * @param[out] noise_filter_result 	 						result of the noise median filter for the current point.
	 * @param[out] object_filter_result 	 					result of the object median filter for the current point.
	 */
	void calcMedianFromBuffer(const int noise_filter_kernel_size,
														const int object_filter_kernel_size,
														const BufferMediansPtr& buffer,
														const median_const_iterator& current_element,
														std::function<float(Segmenter::InputPoint)> f,
														float max_dist_for_median_computation,
														float& noise_filter_result,
														float& object_filter_result) const;

	/**
	 * @brief Converts the distance and intensity deltas to a segmentation probability.
	 *
	 * @param[in] distance_delta 		difference between the results of the object and noise filter applied on the distances.
	 * @param[in] intensity_delta 	difference between the results of the object and noise filter applied on the intensities.
	 *
	 * @return segmentation probability
	 */
	float computeSegmentationProbability(float distance_delta, float intensity_delta);

	/**
	 * @brief Computes the segmentation probabilities for the points in the buffers.
	 *
	 * @param[in,out] buffer_median_filtered 		circular buffer with filtered points.
	 * @param[in,out] median_it 								iterator pointing to the currently processed element.
	 * @param[in] 		end 	 										iterator to the last point in the buffer that was filtered.
	 * @param[out] 		obstacle_cloud 	 					segmented point cloud (from one scan).
	 * @param[out] 		debug_obstacle_cloud 	 		segmented point cloud with additional debug fields.
	 */
	void segmentRing(std::shared_ptr<boost::circular_buffer<MedianFiltered> > buffer_median_filtered,
										median_iterator& median_it,
										median_iterator& end,
										OutputPointCloud::Ptr obstacle_cloud,
										DebugOutputPointCloud::Ptr debug_obstacle_cloud);

	/**
	 * @brief Adjusts points positions to the distances after filtering with the noise kernel.
	 *
	 * For visualization and debug purposes.
	 * m_publish_debug_clouds has to be set to true.
	 *
	 * @param[in] 	cloud 					input cloud.
	 * @param[out] 	filtered_cloud 	output cloud - points positions adjusted to distances after filtering.
	 */
	void fillFilteredCloud(const DebugOutputPointCloud::ConstPtr &cloud,
												DebugOutputPointCloud::Ptr filtered_cloud);

	/**
	 * @brief Show the delta to probability conversion function defined by the thresholds.
	 */
	void plot();


private:
	 const int PUCK_NUM_RINGS;
	 const int HOKUYO_NUM_RINGS;

	 bool m_input_is_velodyne;

	 ros::Subscriber m_velodyne_sub;
	 ros::Subscriber m_hokuyo_sub;
   ros::Publisher m_pub_obstacle_cloud;
   ros::Publisher m_pub_debug_obstacle_cloud;
   ros::Publisher m_pub_filtered_cloud;

   tf::TransformListener m_tf_listener;

   pcl::visualization::PCLPlotter *m_plotter;

   int m_circular_buffer_capacity_launch;
   float m_angle_between_scanpoints_launch;
   float m_certainty_threshold_launch;
   float m_dist_weight_launch;
   float m_intensity_weight_launch;
   float m_object_size_launch;
   float m_distance_to_comparison_points_launch;
   float m_kernel_size_diff_factor_launch;
   float m_median_min_dist_launch;
   float m_median_thresh1_dist_launch;
   float m_median_thresh2_dist_launch;
   float m_median_max_dist_launch;
   float m_max_dist_for_median_computation_launch;
   int m_max_kernel_size;

   float m_max_prob_by_distance;
   float m_max_intensity_range;

   config_server::Parameter<float> m_certainty_threshold;
   config_server::Parameter<float> m_dist_weight;
   config_server::Parameter<float> m_intensity_weight;
   config_server::Parameter<float> m_weight_for_small_intensities;

   config_server::Parameter<float> m_object_size;
   config_server::Parameter<float> m_distance_to_comparison_points;
   config_server::Parameter<float> m_kernel_size_diff_factor;

   config_server::Parameter<float> m_median_min_dist;
   config_server::Parameter<float> m_median_thresh1_dist;
   config_server::Parameter<float> m_median_thresh2_dist;
   config_server::Parameter<float> m_median_max_dist;

   config_server::Parameter<float> m_max_dist_for_median_computation;

   std::string m_input_topic;

   bool m_publish_debug_clouds;

   boost::mutex m_parameter_change_lock;
   std::vector<BufferMediansPtr> m_median_filtered_circ_buffer_vector;

	 std::vector<boost::optional<median_iterator>> m_median_iters_by_ring;
   std::vector<boost::optional<median_iterator>> m_segmentation_iters_by_ring;

   std::vector<float> m_filtering_factors;

	 bool m_buffer_initialized;
   laser_geometry::LaserProjection m_scan_projector;
};

} // namespace laser_segmentation

#endif // _SEGMENTER_H_
