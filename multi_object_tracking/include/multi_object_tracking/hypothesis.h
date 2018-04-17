/** @file
 *
 * Hypothesis implementation.
 *
 * @author Jan Razlaw
 */

#ifndef __HYPOTHESIS_H__
#define __HYPOTHESIS_H__

#include <memory> // for std::shared_ptr
#include <iostream>

#include <multi_object_tracking/kalman_filter.h>

/**
 * @brief Description of detection.
 */
struct Measurement
{
  Eigen::VectorXf pos;      ///< position of detection.
  Eigen::MatrixXf cov;      ///< covariance of detection.
  std::string frame;        ///< frame_id of detection.
  double time;              ///< time_stamp of the detection.
};


namespace MultiHypothesisTracker
{

/**
 * @brief Hypothesis class used for tracking.
 */
class Hypothesis
{
public:
  /**
   * @brief Constructor.
   *
   * @param[in] measurement     initial state.
   * @param[in] id              id of hypothesis.
   */
  Hypothesis(const Measurement& measurement,
             const unsigned int id);
  /** @brief Destructor. */
  virtual ~Hypothesis() = default;

  /**
   * @brief Predicts the state after time difference without control using kalman filter.
   *
   * @param[in] dt  time difference to last state.
   * @see predict(float, Eigen::Vector3f&)
   */
  void predict(float dt);
  /**
   * @brief Predicts the state after time difference using kalman filter.
   *
   * @param[in] dt          time difference to last state.
   * @param[in] control     control vector used for prediction.
   * @see predict(float, Eigen::Vector3f&)
   */
  void predict(float dt,
               Eigen::Vector3f& control);

  /**
   * @brief Corrects the state using the measurement.
   *
   * @param measurement measurement used for correction.
   */
  void correct(const Measurement& measurement);

  /**
   * @brief Checks if covariance exceeds max_covariance.
   *
   * @param covariance      covariance matrix that is checked.
   * @param max_covariance  covariance threshold.
   *
   * @return true if exceeds, false otherwise
   */
  bool exceedsMaxCovariance(const Eigen::Matrix3f& covariance,
                            float max_covariance);

  /**
   * @brief Checks if hypothesis is spurious.
   *
   * //TODO: rename? + more description
   *
   * @param[in] current_time    current time.
   *
   * @return true if hypothesis is spurious, false otherwise.
   */
  bool isSpurious(double current_time);

  /**
   * @brief Updates the detection and misdetection rate after succesful detection.
   */
  void detected();
  /**
   * @brief Updates the detection and misdetection rate after misdetection.
   */
  void undetected();

  /** @brief Getter for hypothesis ID. */
  inline unsigned int getID(){ return m_id; }

  /** @brief Getter for current position. */
  inline Eigen::Vector3f getPosition(){ return m_kalman->getState().block<3,1>(0, 0); }
  /** @brief Getter for current velocity. */
  inline Eigen::Vector3f getVelocity(){ return m_kalman->getState().block<3,1>(3, 0); }
  /** @brief Getter for current position covariance matrix. */
  inline Eigen::Matrix3f getCovariance(){ return m_kalman->getErrorCovariance().block<3,3>(0,0); }

  /** @brief Getter for static property. */
  inline bool isStatic(){ return m_is_static; }

  /** @brief Getter for detection rate. */
  inline float getDetectionRate(){ return m_detection_rate; }
  /** @brief Getter for misdetection rate. */
  inline float getMisdetectionRate(){ return m_misdetection_rate; }

  /** @brief Getter time of hypothesis initialization. */
  inline double getBornTime(){ return m_born_time; }

protected:

  /** @brief Check if hypothesis is still static. */
  void verifyStatic();

  /** @brief Kalman filter for state estimation. */
  std::shared_ptr<KalmanFilter> m_kalman;

  /** @brief Hypothesis ID. */
  unsigned int m_id;

  /** @brief Initial position. */
  Eigen::Vector3f m_first_position_in_track;

  /** @brief Time of initialization. */
  double m_born_time;
  /** @brief time of last correction. */
  double m_last_correction_time;

  /** @brief Number of times the hypothesis got assigned to a measurement. */
  int m_times_measured;

  /** @brief Ratio of detections vs all scans taken. */
  float m_detection_rate;
  /** @brief Ratio of misdetections vs all scans taken. */
  float m_misdetection_rate;

  /** @brief Flag for immobility of hypothesis. */
  bool m_is_static;
  /** @brief Distance a hypothesis is allowed to move to still be considered static. */
  double m_static_distance_threshold;

  /** @brief Flag that specifies if velocity should be capped. */
  bool m_cap_velocity;
  /** @brief Bound for velocity. */
  double m_max_allowed_velocity;
  /** @brief Maximal velocity this hypothesis had. */
  double m_max_tracked_velocity;

  /** @brief Bound for covariance. */
  float m_max_covariance;
};

/**
 * @brief Hypothesis factory.
 */
class HypothesisFactory
{
public:
  /** @brief Constructor. */
  HypothesisFactory() = default;
  /** @brief Destructor. */
  virtual ~HypothesisFactory() = default;

  /** @brief Creates hypothesis.
   *
   * @param[in] measurement     inital state.
   * @param[in] id              ID of created hypothesis.
   *
   * @return pointer to created hypothesis.
   */
  std::shared_ptr<Hypothesis> createHypothesis(const Measurement& measurement,
                                               unsigned int id);
};

};

#endif //__HYPOTHESIS_H__
