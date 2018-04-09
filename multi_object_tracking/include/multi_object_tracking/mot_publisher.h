/** @file
 *
 * Class to publish data from the multi object tracker.
 *
 * @author Jan Razlaw
 */

#ifndef __MOT_PUBLISHER_H__
#define __MOT_PUBLISHER_H__

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>

#include <object_detection/ObjectDetections.h>
#include <multi_object_tracking/DebugTracking.h>

#include <multi_object_tracking/hypothesis_3D.h>


namespace MultiHypothesisTracker
{

class MOTPublisher
{
public:
  /** @brief Constructor */
  MOTPublisher();
  /** @brief Destructor */
  ~MOTPublisher(){};

  /** @brief Calls all publishers. */
  void publishAll(const std::vector<std::shared_ptr<Hypothesis>>& hypotheses);

  /** @brief Publish positions of hypotheses that are tracked longer than #m_born_time_threshold */
  void publishHypothesesPositions(const std::vector<std::shared_ptr<Hypothesis>> &hypotheses);
  /** @brief Publish hypotheses that are tracked longer than #m_born_time_threshold */
  void publishHypothesesFull(const std::vector<std::shared_ptr<Hypothesis>> &hypotheses);
  /** @brief Publish hypotheses' predictions that are tracked longer than #m_born_time_threshold */
  void publishHypothesesPredictions(const std::vector<std::shared_ptr<Hypothesis>> &hypotheses);
  /** @brief Publish predicted positions of hypotheses that are tracked longer than #m_born_time_threshold */
  void publishHypothesesPredictedPositions(const std::vector<std::shared_ptr<Hypothesis>> &hypotheses);
  /** @brief Publish covariances of hypotheses that are tracked longer than #m_born_time_threshold */
  void publishHypothesesCovariances(const std::vector<std::shared_ptr<Hypothesis>> &hypotheses);
  /** @brief Publish positions of static hypotheses that are tracked longer than #m_born_time_threshold */
  void publishStaticHypothesesPositions(const std::vector<std::shared_ptr<Hypothesis>> &hypotheses);  //publishes a vertical line indicating which hypothesis are static (non-moveable)
  /** @brief Publish positions of dynamic hypotheses that are tracked longer than #m_born_time_threshold */
  void publishDynamicHypothesesPositions(const std::vector<std::shared_ptr<Hypothesis>> &hypotheses);
  /** @brief Publish full tracks of dynamic hypotheses that are tracked longer than #m_born_time_threshold */
  void publishFullTracks(const std::vector<std::shared_ptr<Hypothesis>>& hypotheses);

  /**
   * @brief Return a rather specific marker.
   *
   * @param[in] r   red color value.
   * @param[in] g   green color value.
   * @param[in] b   blue color value.
   * @param[in] ns  namespace of marker.
   *
   * @return marker.
   */
  visualization_msgs::Marker createMarker(float r = 0.0,
                                          float g = 1.0,
                                          float b = 0.0,
                                          std::string ns = "multi_object_tracker");

  /**
   * @brief Publishes measurements as markers.
   *
   * @param[in] measurements   detections.
   */
  void publishMeasurementPositions(const std::vector<Measurement> &measurements);

  /**
   * @brief Publishes measurements covariances as markers.
   *
   * @param[in] measurements   detections.
   */
  void publishMeasurementsCovariances(const std::vector<Measurement> &measurements);

  /** @brief Publish the data from the first hypothesis in the memory. */
  void publishDebug(const std::vector<std::shared_ptr<Hypothesis>> &hypotheses);

private:
  // Parameters
  /** @brief A fixed frame in the world. */
  std::string m_world_frame;
  /** @brief Time after which we start publishing a new hypothesis.
   *
   * If the hypothesis is too young it may be unreliable and therefore it
   * will be removed by the isSpurious.
   */
  double m_born_time_threshold;
  /** @brief Time offset for predictions. */
  double m_future_time;

  int m_debug_counter;

  ros::Publisher m_hypotheses_positions_publisher;
  ros::Publisher m_measurement_positions_publisher;
  ros::Publisher m_measurements_covariances_publisher;
  ros::Publisher m_hypotheses_covariance_publisher;
  ros::Publisher m_track_line_publisher;
  ros::Publisher m_static_hypotheses_positions_publisher;
  ros::Publisher m_dynamic_hypotheses_positions_publisher;
  ros::Publisher m_hypotheses_full_publisher;
  ros::Publisher m_hypotheses_predictions_publisher;
  ros::Publisher m_debug_publisher;
  ros::Publisher m_hypotheses_predicted_positions_publisher;

  visualization_msgs::Marker full_track;
};

}

#endif