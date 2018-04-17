/** @file
 *
 * Kalman filter implementation
 *
 * @author Jan Razlaw
 */

#include "multi_object_tracking/kalman_filter.h"

namespace MultiHypothesisTracker
{

KalmanFilter::KalmanFilter(const Eigen::VectorXf& state)
: m_measurement_dimensions(3)
 , m_control_dimensions(1)
{
  m_state_dimensions = state.size();
  m_state = state;


  // next_state = m_state_transition_model * m_state + m_control_input_model * control + process_noise  with process_noise ~ N(0,m_process_noise_covariance)
  m_state_transition_model.resize(m_state_dimensions, m_state_dimensions);
  m_state_transition_model.setIdentity();

  m_control_input_model = Eigen::MatrixXf(m_state_dimensions, m_control_dimensions);
  m_control_input_model.setZero();

  m_process_noise_covariance = Eigen::MatrixXf(m_state_dimensions, m_state_dimensions);
  m_process_noise_covariance.setIdentity();


  // measurement = m_observation_model * current_state + observation_noise  with observation_noise ~ N(0,m_observation_noise_covariance)
  m_observation_model = Eigen::MatrixXf(m_measurement_dimensions, m_state_dimensions);
  m_observation_model.setZero();

  m_observation_noise_covariance = Eigen::MatrixXf(m_measurement_dimensions, m_measurement_dimensions);
  m_observation_noise_covariance.setIdentity();


  m_error_covariance = Eigen::MatrixXf(m_state_dimensions, m_state_dimensions);
  m_error_covariance.setIdentity();
}

void KalmanFilter::predict(float dt)
{
  Eigen::VectorXf control(m_control_dimensions);
  control.setZero();
  predict(dt, control);
}

void KalmanFilter::predict(float dt,
                           const Eigen::VectorXf& control)
{
  // set up state transition model
  m_state_transition_model.setIdentity();
  m_state_transition_model(0, 3) = dt;
  m_state_transition_model(1, 4) = dt;
  m_state_transition_model(2, 5) = dt;

  // set up control input model - here not used
  m_control_input_model.setZero();

  // update state according to models
  m_state = m_state_transition_model * m_state + m_control_input_model * control;


  // TODO: check if approx correct
  // set up process_noise_covariance
  float covariance_per_second = 0.1;
  for(size_t i = 0; i < m_state_dimensions; i++)
    m_process_noise_covariance(i, i) = dt * covariance_per_second;

  // update error covariance
  m_error_covariance = m_state_transition_model * m_error_covariance * m_state_transition_model.transpose() + m_process_noise_covariance;

  // check if cov matrix is symmetric as is should be
  if(!isAlmostSymmetric(m_error_covariance))
    std::cout << "KalmanFilter::predict: m_error_covariance is not symmetric!!!!!" << std::endl;

  // TODO: check if matrix is positive definite ?!?
}

void KalmanFilter::correct(const Eigen::VectorXf& measurement,
                           const Eigen::MatrixXf& measurement_covariance)
{
  assert(measurement.size() == m_measurement_dimensions);

  // set up measurement model
  m_observation_model.setZero();
  for(size_t i = 0; i < m_measurement_dimensions; i++)
    m_observation_model(i, i) = 1.f;

  // set up observation noise covariance
  m_observation_noise_covariance = measurement_covariance;

  // compute kalman gain
  Eigen::MatrixXf temp = m_error_covariance * m_observation_model.transpose();
  Eigen::MatrixXf kalman_gain = temp * (m_observation_model * temp + m_observation_noise_covariance).inverse();


  // compute the expected measurement
  Eigen::VectorXf expected_measurement = m_observation_model * m_state;

  // correct state
  m_state = m_state + kalman_gain * (measurement - expected_measurement);


  // update error covariance
  Eigen::MatrixXf identity(kalman_gain.rows(), m_observation_model.cols());
  identity.setIdentity();
  m_error_covariance = (identity - kalman_gain * m_observation_model) * m_error_covariance;


  // TODO: check if calculations are responsible for need for epsilon
  // check if cov matrix is symmetric as it should be
  if(!isAlmostSymmetric(m_error_covariance))
    std::cout << "KalmanFilter::correct: m_error_covariance is not symmetric!!!!!\n" << m_error_covariance << std::endl;
}

bool KalmanFilter::isAlmostSymmetric(const Eigen::MatrixXf& matrix,
                                     float epsilon)
{
  bool is_symmetric = true;
  for(int i = 1; i < matrix.rows(); i++)
    for(int j = i; j < matrix.cols(); j++)
      if(fabs(matrix(i, j) - matrix(j, i)) > epsilon)
        is_symmetric = false;

  return is_symmetric;
}

};
