#ifndef __KALMAN_FILTER_H__
#define __KALMAN_FILTER_H__

#include <Eigen/Eigenvalues>
#include <Eigen/Cholesky>

#include <vector>
#include <algorithm>
#include <iostream>

#include <multi_object_tracking/utils.h>

// uncomment to disable assert()
// #define NDEBUG
#include <cassert>

namespace MultiHypothesisTracker
{

class KalmanFilter
{
public:

  KalmanFilter();
  virtual ~KalmanFilter(){};

  void initialize(const Eigen::VectorXf& state);

  void predict(float dt);
  void predict(float dt,
               const Eigen::VectorXf& control);

  void correct(const Eigen::VectorXf& measurement,
               const Eigen::MatrixXf& measurement_covariance);

  bool isSymmetric(const Eigen::MatrixXf& covariance);

  Eigen::MatrixXf getErrorCovariance(){ return m_error_covariance; };

  // TODO getter
  Eigen::VectorXf m_state;
  int m_state_dimensions;

protected:
  Eigen::MatrixXf m_state_transition_model;
  Eigen::MatrixXf m_control_input_model;
  Eigen::MatrixXf m_observation_model;

  Eigen::MatrixXf m_error_covariance;
  Eigen::MatrixXf m_process_noise_covariance;
  Eigen::MatrixXf m_observation_noise_covariance;

  size_t m_control_dimensions;
};

};

#endif //__KALMAN_FILTER_H__
