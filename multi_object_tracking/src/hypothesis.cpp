/** @file
 *
 * Hypothesis implementation.
 *
 * @author Jan Razlaw
 */

#include "multi_object_tracking/hypothesis.h"

#define DETECTION_RATE_INCREMENT 1.0f

namespace MultiHypothesisTracker
{

Hypothesis::Hypothesis(const Measurement& measurement,
                       unsigned int id)
: m_id(id)
  , m_born_time(measurement.time)
  , m_last_correction_time(measurement.time)
  , m_times_measured(1)
  , m_detection_rate(0.5f) // TODO: should those be 1?
  , m_misdetection_rate(0.5f)
  , m_is_static(true)
  , m_static_distance_threshold(1.f)
  , m_cap_velocity(true)
  , m_max_allowed_velocity(1.4) // 1.4m/s or 5km/h
  , m_max_tracked_velocity(0.0)
  , m_max_covariance(0.1f)
{
  int number_of_state_dimensions = 6;
  Eigen::VectorXf meas(number_of_state_dimensions);
  meas.setZero();
  for(int i = 0; i < 3; i++)
    meas(i) = measurement.pos(i);

  m_kalman = std::make_shared<KalmanFilter>(meas);

  m_first_position_in_track = getPosition();
}

void Hypothesis::predict(float dt)
{
  Eigen::Vector3f control;
  predict(dt, control);
}

void Hypothesis::predict(float dt,
                         Eigen::Vector3f& control)
{
  m_kalman->predict(dt);

  verifyStatic();
}

void Hypothesis::correct(const Measurement& measurement)
{
  m_kalman->correct(measurement.pos, measurement.cov);

  m_last_correction_time = measurement.time;


  // additional stuff and workarounds

  Eigen::Vector3f current_velocity = getVelocity();
  for(int i = 0; i < 3; i++)
    current_velocity(i) = m_kalman->getState()(3+i);

  // keep track of maximal recorded velocity
  double current_velocity_magnitude = current_velocity.norm();
  if(current_velocity_magnitude > m_max_tracked_velocity)
  {
    m_max_tracked_velocity = current_velocity_magnitude;
  }

  if(m_cap_velocity)
  {
    if(current_velocity.norm() > m_max_allowed_velocity)
    {
      current_velocity.normalize();
      current_velocity *= m_max_allowed_velocity;
      for(int i = 0; i < 3; i++)
        m_kalman->getState()(3+i) = current_velocity(i);
    }
  }
}

// TODO: check if this is correct: if not replace by running average
void Hypothesis::detected()
{
  m_detection_rate += DETECTION_RATE_INCREMENT;
  float sumDetectionRate = m_detection_rate + m_misdetection_rate;
  m_detection_rate /= sumDetectionRate;
  m_misdetection_rate /= sumDetectionRate;
  m_times_measured++;
}

void Hypothesis::undetected()
{
  m_misdetection_rate += DETECTION_RATE_INCREMENT;
  float sumDetectionRate = m_detection_rate + m_misdetection_rate;
  m_detection_rate /= sumDetectionRate;
  m_misdetection_rate /= sumDetectionRate;
}

bool Hypothesis::exceedsMaxCovariance(const Eigen::Matrix3f& covariance,
                                      float max_covariance)
{
  Eigen::EigenSolver<Eigen::Matrix3f> eigen_solver(covariance);
  auto eigen_values = eigen_solver.eigenvalues();

  return (eigen_values.col(0)[0].real() > max_covariance ||
          eigen_values.col(0)[1].real() > max_covariance ||
          eigen_values.col(0)[2].real() > max_covariance);
}

bool Hypothesis::isSpurious(double current_time)
{
  // TODO: magic numbers to parameters or members if this if clause is useful
  if((current_time - m_born_time > 0.65) && m_times_measured <= 5)
    return true;

  // TODO: Jan: should be a parameter
  if(current_time - m_last_correction_time > 90)
  {
    return true;
  }
  else
  {
//    if(exceedsMaxCovariance(getCovariance(), m_max_covariance))
//      return true;
//    else
      return false;
  }
}

void Hypothesis::verifyStatic()
{
  if(m_is_static)
  {
    double distance_from_origin = (getPosition() - m_first_position_in_track).norm();
    // TODO: test if check for max velocity only is better. or dist fram origin with a larger distance + check for max velocity to account for registration mistakes that statistically should keep the object position in a range around the origin
    if(distance_from_origin > m_static_distance_threshold /*&& (m_max_velocity_in_track.norm() > 0.85)*/)
      m_is_static = false;
  }
}



std::shared_ptr<Hypothesis> HypothesisFactory::createHypothesis(const Measurement& measurement,
                                                                unsigned int id)
{
  return std::make_shared<Hypothesis>(measurement, id);
}

};
