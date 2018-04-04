#ifndef __MULTI_OBJECT_TRACKER_H__
#define __MULTI_OBJECT_TRACKER_H__

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>

#include <object_detection/ObjectDetections.h>
// #include "multi_object_tracking/TrackReset.h"
#include <multi_object_tracking/DebugTracking.h>

#include <limits>

#include <multi_object_tracking/multiobjecttracker_algorithm.h>
#include <multi_object_tracking/multihypothesistracker.h>


namespace MultiObjectTracker
{

class Tracker
{
public:
  Tracker();
  ~Tracker(){};

  void update();

  /**
   * @brief Callback function for detections messages
   *
   * Converts messages to measurements.
   * Transforms measurements to #m_world_frame and passes those to the tracking algorithm.
   *
   * @param [in] msg    poses of the detections.
   */
  void detectionCallback(const geometry_msgs::PoseArray::ConstPtr& msg);

  void publish_hypotheses();
  void publish_hypotheses_object_msg();
  void publish_hypotheses_future_object_msg();
  void publish_hypotheses_future();
  void publish_hypothesis_covariance();
  void publish_static_hypotheses();  //publishes a vertical line indicating which hypothesis are static (non-moveable)
  void publish_dynamic_hypotheses();

  /** @brief Publish the data from the first hypothesis and it's latest measurement that it was corrected with. */
  void publish_debug();

private:
  std::vector<Measurement> m_picked_static_positions;
  int m_debug_counter;

  //Params
  double m_merge_close_hypotheses_distance;
  double m_max_mahalanobis_distance;
  std::string m_world_frame;
  double m_born_time_threshold;   //Time after which we start publishing the hypothesis. If the hypothesis is too young it may be unrealiable and therefore it will be removed by the isSpurious
  double m_future_time;

  ros::Publisher m_hypothesesPublisher;
  ros::Publisher m_measurement_marker_publisher;
  ros::Publisher m_measurementCovPublisher;
  ros::Publisher m_hypothesisCovPublisher;
  ros::Publisher m_track_linePublisher;
  ros::Publisher m_static_objectsPublisher;
  ros::Publisher m_dynamic_objectsPublisher;
  ros::Publisher m_hypothesis_object_msg_publisher;
  ros::Publisher m_hypothesis_future_object_msg_publisher;
  ros::Publisher m_debug_publisher;
  ros::Publisher m_hypotheses_future_publisher;

  ros::Subscriber m_laser_detection_subscriber;

  std::shared_ptr<tf::TransformListener> m_transformListener;

  std::shared_ptr<MultiObjectTrackerAlgorithm> m_algorithm;

  visualization_msgs::Marker full_track;

  /**
   * @brief Return a rather specific marker.
   *
   * @param[in] x   x position.
   * @param[in] y   y position.
   * @param[in] z   z position.
   * @param[in] r   red color value.
   * @param[in] g   green color value.
   * @param[in] b   blue color value.
   *
   * @return marker.
   */
  visualization_msgs::Marker createMarker(int x=0, int y=0, int z=0, float r=0.0, float g=1.0, float b=0.0);

  /**
   * @brief Transforms measurements to the target_frame.
   *
   * @param[in,out] measurements   measurements.
   * @param[in]     target_frame   frame the measurements are transformed to.
   *
   * @return false if at least one measurement couldn't be transformed, true otherwise
   */
  bool transform_to_frame(std::vector<Measurement>& measurements,
                          std::string target_frame);

  /**
   * @brief Publishes measurements as markers.
   *
   * @param[in] measurements   detections.
   */
  void publish_measurement_markers(std::vector<Measurement> measurements);

  /**
   * @brief Publishes measurements covariances as markers.
   *
   * @param[in] measurements   detections.
   */
  void publish_measurement_covariance(std::vector<Measurement> measurements);

  /**
   * @brief Converts the detections from the laser into the internal format
   *
   * @param[in] msg   poses of the detections.
   */
  std::vector<Measurement> laser_detections2measurements(const geometry_msgs::PoseArray::ConstPtr& msg);

};

}

#endif