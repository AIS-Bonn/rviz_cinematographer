#include "multi_object_tracking/kalman_filter.h"

namespace MultiHypothesisTracker
{

KalmanFilter::KalmanFilter(const Eigen::VectorXf& state)
: m_state_dimensions(state.size())
  , m_control_dimensions(1) // TODO: parameter?
  , m_measurement_dimensions(3)
{
  // next_state = m_state_transition_model * m_state + m_control_input_model * control + process_noise  with process_noise ~ N(0,m_process_noise_covariance)
  m_state_transition_model = Eigen::MatrixXf(m_state_dimensions, m_state_dimensions);
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

  m_state = state;
}

const TrackerParameters& KalmanFilter::getParameters()
{
  static TrackerParameters params = {
    0.001,	// cov_x_per_sec
    0.001,	// cov_y_per_sec
    0.001,	// cov_z_per_sec

    0.001,	// cov_vx_per_sec (independent of |vx|)
    0.001,	// cov_vy_per_sec (independent of |vy|)
    0.001,	// cov_vz_per_sec (independent of |vz|)

    0,	// alpha_vx_vx_per_sec
    0,	// alpha_vx_vy_per_sec
    0,	// alpha_vy_vy_per_sec
    0,	// alpha_vz_vz_per_sec

    0.05*0.05, // init_cov
    0.05*0.05, // max_cov

    0.1, // measurementStd

    0.4, // ambiguous_dist

    };
  return params;
}

void KalmanFilter::initialize()
{
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
  float covariance_per_second = 0.001;
  for(int i = 0; i < m_state_dimensions; i++)
    m_process_noise_covariance(i, i) = dt * covariance_per_second;

  // update error covariance
  m_error_covariance = m_state_transition_model * m_error_covariance * m_state_transition_model.transpose() + m_process_noise_covariance;


  // check if cov matrix is symmetric as is should be
  if(!isSymmetric(m_error_covariance))
    std::cout << "KalmanFilter::predict: m_error_covariance is not symmetric!!!!!" << std::endl;

  // TODO: check if matrix is positive definite ?!?
}

void KalmanFilter::correct(const Eigen::VectorXf& measurement,
                           const Eigen::MatrixXf& measurement_covariance)
{
  assert(measurement.size() == m_measurement_dimensions);

  // set up measurement model //TODO: move set ups for fixed matrices to constructor?
  m_observation_model.setZero();
  for(int i = 0; i < m_measurement_dimensions; i++)
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

  Eigen::MatrixXf identity(kalman_gain.rows(), m_observation_model.cols());
  identity.setIdentity();
  m_error_covariance = (identity - kalman_gain * m_observation_model) * m_error_covariance;


  // check if cov matrix is symmetric as is should be
  if(!isSymmetric(m_error_covariance))
    std::cout << "KalmanFilter::correct: m_error_covariance is not symmetric!!!!!" << std::endl;
}

bool KalmanFilter::isSymmetric(const Eigen::MatrixXf& covariance)
{
  bool is_symmetric = true;
  for(int i = 0; i < (int)covariance.rows(); i++)
    for(int j = i; j < (int)covariance.cols(); j++)
      if(covariance(i, j) != covariance(j, i))
        is_symmetric = false;

  return is_symmetric;
}

};
