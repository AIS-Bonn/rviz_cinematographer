#ifndef __KALMAN_FILTER_H__
#define __KALMAN_FILTER_H__

#include <Eigen/Eigenvalues>
#include <Eigen/Cholesky>

#include <vector>
#include <algorithm>
#include <memory> // std::shared_ptr
#include <iostream>

#include <multi_object_tracking/utils.h>

// uncomment to disable assert()
// #define NDEBUG
#include <cassert>

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


class KalmanFilter
{
public:

  KalmanFilter(const Eigen::VectorXf& state);
  virtual ~KalmanFilter(){};

  void initialize();

  void predict(float dt);
  void predict(float dt,
               const Eigen::VectorXf& control);

  void correct(const Eigen::VectorXf& measurement,
               const Eigen::MatrixXf& measurement_covariance);

  bool isSymmetric(const Eigen::MatrixXf& covariance);

  Eigen::VectorXf m_state;

protected:
  Eigen::MatrixXf m_state_transition_model;
  Eigen::MatrixXf m_control_input_model;
  Eigen::MatrixXf m_observation_model;

  Eigen::MatrixXf m_error_covariance;
  Eigen::MatrixXf m_process_noise_covariance;
  Eigen::MatrixXf m_observation_noise_covariance;

  int m_state_dimensions;
  int m_control_dimensions;
  int m_measurement_dimensions;
};

};

#endif //__KALMAN_FILTER_H__
