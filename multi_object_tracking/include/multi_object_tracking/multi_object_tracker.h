#ifndef __MULTI_OBJECT_TRACKER_H__
#define __MULTI_OBJECT_TRACKER_H__

#include <ros/ros.h>
#include <ros/console.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>

#include <object_detection/ObjectDetections.h>

#include <multi_object_tracking/multihypothesistracker.h>
#include <multi_object_tracking/mot_publisher.h>


namespace MultiHypothesisTracker
{

class Tracker
{
public:
  Tracker();
  ~Tracker(){};

  /**
   * @brief Publishes the hypothesis in several versions.
   */
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
  bool transformToFrame(std::vector<Measurement>& measurements,
                        const std::string target_frame);

  /**
   * @brief Converts the detections from the laser into the internal format
   *
   * @param[in]     msg             poses of the detections.
   * @param[out]    measurements    detections in tracker format.
   */
  void convert(const geometry_msgs::PoseArray::ConstPtr &msg,
               std::vector<Measurement>& measurements);

  /**
   * @brief Calls prediction method of the hypotheses and deletes spurious hypothesis afterwards.
   */
  void predict(double prediction_time);

  /**
   * @brief Predicts and corrects the hypotheses based on the new measurements.
   *
   * // TODO: rename
   *
   * @param measurements    new detections
   */
  void objectDetectionDataReceived(const std::vector<Measurement>& measurements);

  const std::vector<std::shared_ptr<Hypothesis>>& getHypotheses();

  void setMergeDistance(double distance){ m_merge_distance = distance; }

private:
  /** @brief Subscribes to detections. */
  ros::Subscriber m_laser_detection_subscriber;
  /** @brief Publishes results. */
  MOTPublisher m_mot_publisher;

  /** @brief Provides transforms to world frame. */
  std::shared_ptr<tf::TransformListener> m_transform_listener;

  /** @brief The functionality. */
  MultiHypothesisTracker m_multi_hypothesis_tracker;


  //Params
  /** @brief Hypotheses that are closer than this distance to one another are merged. */
  double m_merge_close_hypotheses_distance;
  /** @brief Threshold for Mahalanobis distance. */
  double m_max_mahalanobis_distance;
  /** @brief Fixed frame the detections and tracks are in. */
  std::string m_world_frame;

  double m_last_prediction_time;
  double m_merge_distance;
};

}

#endif