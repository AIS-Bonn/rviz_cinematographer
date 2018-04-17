#include "multi_object_tracking/hypothesis.h"


// #define DETECTION_RATE_INCREMENT 0.04f
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
{
  int number_of_state_dimensions = 6;
  Eigen::VectorXf meas(number_of_state_dimensions);
  meas.setZero();
  for(int i = 0; i < 3; i++)
    meas(i) = measurement.pos(i);

  m_kalman = std::make_shared<KalmanFilter>(meas);

  m_first_position_in_track = getPosition();
}

const TrackerParameters& Hypothesis::getParameters()
{
  static TrackerParameters params = {
          0.0075,	// cov_x_per_sec
          0.0075,	// cov_y_per_sec
          0.0075,	// cov_z_per_sec

          0.01,	// cov_vx_per_sec (independent of |vx|)
          0.01,	// cov_vy_per_sec (independent of |vy|)
          0.01,	// cov_vz_per_sec (independent of |vz|)

          0,	// alpha_vx_vx_per_sec
          0,	// alpha_vx_vy_per_sec
          0,	// alpha_vy_vy_per_sec
          0,	// alpha_vz_vz_per_sec

          // 0.06*0.06, // init_cov
          0.02*0.02, // init_cov
          0.10, // max_cov

          0.2*0.2, // measurementStd

// 		sqrt(2.204) // ambiguous_dist
          13.3*sqrt(2.204)
  };
  return params;
}

// TODO: check if this is correct: if not replace by running average
void Hypothesis::detected()
{
  m_detection_rate += DETECTION_RATE_INCREMENT;
  float sumDetectionRate = m_detection_rate + m_misdetection_rate;
  m_detection_rate /= sumDetectionRate;
  m_misdetection_rate /= sumDetectionRate;
}

void Hypothesis::undetected()
{
  m_misdetection_rate += DETECTION_RATE_INCREMENT;
  float sumDetectionRate = m_detection_rate + m_misdetection_rate;
  m_detection_rate /= sumDetectionRate;
  m_misdetection_rate /= sumDetectionRate;
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
    // TODO: reenable but test effects and tune parameter
    double max_position_cov = getParameters().max_cov;

    Eigen::EigenSolver<Eigen::Matrix3f> eigen_solver(getCovariance());
//     if( eigen_solver.eigenvalues().col(0)[0] > max_position_cov || eigen_solver.eigenvalues().col(0)[1] > max_position_cov || eigen_solver.eigenvalues().col(0)[2] > max_position_cov ) {
//     	return true;
//     }

    return false;
  }
}

void Hypothesis::verify_static()
{
  if(m_is_static)
  {
    double distance_from_origin = (getPosition() - m_first_position_in_track).norm();
    // TODO: test if check for max velocity only is better. or dist fram origin with a larger distance + check for max velocity to account for registration mistakes that statistically should keep the object position in a range around the origin
    if(distance_from_origin > m_static_distance_threshold /*&& (m_max_velocity_in_track.norm() > 0.85)*/)
      m_is_static = false;
  }
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

  verify_static();



  //TODO: extract to own function that checks for all hyp if their covariance is too big and the hyp should be deleted
//
//  //If covariance is bigger than the maximum cov than do not update it anymore
//  double max_position_cov = getParameters().max_cov;
//
//  //TODO: test if result is same with the one below
//  Eigen::EigenSolver<Eigen::Matrix3f> eigen_solver(m_covariance);
//  auto eigen_values = eigen_solver.eigenvalues();
//
//  if(eigen_values.col(0)[0].real() > max_position_cov || eigen_values.col(0)[1].real() > max_position_cov || eigen_values.col(0)[2].real() > max_position_cov)
//  {
//    //don't update
//  }
}

void Hypothesis::correct(const Measurement& measurement)
{
  m_kalman->correct(measurement.pos, measurement.cov);

  m_last_correction_time = measurement.time;


  // additional stuff and workarounds

  Eigen::Vector3f current_velocity = getVelocity();
  for(int i = 0; i < 3; i++)
    current_velocity(i) = m_kalman->getState()(3+i);

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

  // keep track of maximal recorded velocity
  double current_velocity_magnitude = current_velocity.norm();
  if(current_velocity_magnitude > m_max_tracked_velocity)
  {
    m_max_tracked_velocity = current_velocity_magnitude;
  }
}


std::shared_ptr<Hypothesis> HypothesisFactory::createHypothesis(const Measurement& measurement,
                                                                unsigned int id)
{
  return std::make_shared<Hypothesis>(measurement, id);
}

};
