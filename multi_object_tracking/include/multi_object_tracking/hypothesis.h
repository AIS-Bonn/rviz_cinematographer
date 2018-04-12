#ifndef __HYPOTHESIS_H__
#define __HYPOTHESIS_H__

#include <vector>
#include <string>
#include <algorithm>
#include <chrono>
#include <memory> // std::shared_ptr

#include <iostream>

#include <multi_object_tracking/kalman_filter.h>
#include <multi_object_tracking/utils.h>

/**
 * @brief Description of detection //TODO: rename
 */
struct Measurement
{
  Eigen::VectorXf pos;      ///< position of detection
  Eigen::MatrixXf cov;      ///< covariance of detection
  std::string frame;        ///< frame_id of detection
  double time;              ///< time_stamp of the detection - in time of day
};


namespace MultiHypothesisTracker
{

struct TrackerParameters
{
  double cov_x_per_sec;
  double cov_y_per_sec;
  double cov_z_per_sec;
  double cov_vx_per_sec;
  double cov_vy_per_sec;
  double cov_vz_per_sec;
  double alpha_vx_vx_per_sec;
  double alpha_vx_vy_per_sec;
  double alpha_vy_vy_per_sec;
  double alpha_vz_vz_per_sec;

  double init_cov;
  double max_cov;

  double measurementStd;

  double ambiguous_dist;
};


class Hypothesis
{
public:

  Hypothesis();
  virtual ~Hypothesis(){};

  virtual const TrackerParameters& getParameters();

  inline unsigned int getID(){ return m_id; }

  // TODO: delete? - never used, just set
  inline unsigned int getNumStateDimensions() { return m_num_state_dimensions; }

  inline Eigen::Vector3f getMean(){ return m_kalman.m_state.block<3,1>(0, 0); }
  inline Eigen::Vector3f getVelocity(){ return m_kalman.m_state.block<3,1>(3, 0); }
  inline Eigen::Matrix3f getCovariance(){ return m_kalman.getErrorCovariance().block<3,3>(0,0); }
  inline bool isStatic(){ return m_is_static; }

  virtual void initialize(const Measurement& measurement,
                          unsigned int id);

  virtual bool isSpurious(double current_time);

  virtual void detected();
  virtual void undetected();

  inline float getDetectionRate() { return m_detection_rate; }
  inline float getMisdetectionRate() { return m_misdetection_rate; }

  virtual void predict(float dt);
  virtual void predict(float dt, Eigen::Vector3f& control);
  virtual void correct( const Measurement& measurement );

  inline double get_born_time(){return m_born_time;}
  inline void detected_absolute(){m_times_measured++;}  //total number of times that hypothesis had a measurement

  KalmanFilter m_kalman;

protected:

  void verify_static();

  bool m_is_static;

  float m_detection_rate;
  float m_misdetection_rate;

  unsigned int m_id;

  size_t m_num_state_dimensions;

  Eigen::Vector3f m_first_position_in_track;
  double m_born_time;
  int m_times_measured;

  double m_static_distance_threshold;
  Eigen::Vector3f m_state_after_last_correction;
  double m_last_correction_time;

  bool m_cap_velocity;
  double m_max_allowed_velocity;

  double m_max_tracked_velocity;

};

class HypothesisFactory
{
public:
  HypothesisFactory(){}
  virtual ~HypothesisFactory(){}

  virtual std::shared_ptr<Hypothesis> createHypothesis();
};

};

#endif //__HYPOTHESIS_H__
