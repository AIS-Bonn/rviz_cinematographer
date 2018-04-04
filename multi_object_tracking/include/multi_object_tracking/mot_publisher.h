#ifndef __MOT_PUBLISHER_H__
#define __MOT_PUBLISHER_H__

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
#include <multi_object_tracking/DebugTracking.h>

#include <limits>

#include <multi_object_tracking/multihypothesistracker.h>


namespace MultiHypothesisTracker
{

class MOTPublisher
{
public:
  MOTPublisher();
  ~MOTPublisher(){};

  void setFrame(const std::string& frame){ m_world_frame = frame; };
  void setBornTimeThreshold(const double threshold){ m_born_time_threshold = threshold; };
  void setFutureTime(const double time){ m_future_time = time; };

  void publishAll(const std::vector<std::shared_ptr<Hypothesis>>& hypotheses);
  void publish_hypotheses(const std::vector<std::shared_ptr<Hypothesis>>& hypotheses);
  void publish_hypotheses_object_msg(const std::vector<std::shared_ptr<Hypothesis>>& hypotheses);
  void publish_hypotheses_future_object_msg(const std::vector<std::shared_ptr<Hypothesis>>& hypotheses);
  void publish_hypotheses_future(const std::vector<std::shared_ptr<Hypothesis>>& hypotheses);
  void publish_hypothesis_covariance(const std::vector<std::shared_ptr<Hypothesis>>& hypotheses);
  void publish_static_hypotheses(const std::vector<std::shared_ptr<Hypothesis>>& hypotheses);  //publishes a vertical line indicating which hypothesis are static (non-moveable)
  void publish_dynamic_hypotheses(const std::vector<std::shared_ptr<Hypothesis>>& hypotheses);

  /** @brief Publish the data from the first hypothesis and it's latest measurement that it was corrected with. */
  void publish_debug(const std::vector<std::shared_ptr<Hypothesis>>& hypotheses);

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
   * @brief Publishes measurements as markers.
   *
   * @param[in] measurements   detections.
   */
  void publish_measurement_markers(const std::vector<Measurement>& measurements);

  /**
   * @brief Publishes measurements covariances as markers.
   *
   * @param[in] measurements   detections.
   */
  void publish_measurement_covariance(const std::vector<Measurement>& measurements);

private:
  int m_debug_counter;

  //Params
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

  visualization_msgs::Marker full_track;

};

}

#endif