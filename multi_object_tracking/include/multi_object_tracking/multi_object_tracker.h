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
#include <multi_object_tracking/mot_publisher.h>


namespace MultiHypothesisTracker
{

class Tracker
{
public:
  Tracker();
  ~Tracker(){};

  void update();
  void publish();

  /**
   * @brief Callback function for detections messages
   *
   * Converts messages to measurements.
   * Transforms measurements to #m_world_frame and passes those to the tracking algorithm.
   *
   * @param [in] msg    poses of the detections.
   */
  void detectionCallback(const geometry_msgs::PoseArray::ConstPtr& msg);

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
   * @brief Converts the detections from the laser into the internal format
   *
   * @param[in] msg   poses of the detections.
   */
  std::vector<Measurement> laser_detections2measurements(const geometry_msgs::PoseArray::ConstPtr& msg);

private:
  MOTPublisher m_mot_publisher;

  std::vector<Measurement> m_picked_static_positions;

  //Params
  double m_merge_close_hypotheses_distance;
  double m_max_mahalanobis_distance;
  std::string m_world_frame;

  ros::Subscriber m_laser_detection_subscriber;

  std::shared_ptr<tf::TransformListener> m_transformListener;

  std::shared_ptr<MultiObjectTrackerAlgorithm> m_algorithm;
};

}

#endif