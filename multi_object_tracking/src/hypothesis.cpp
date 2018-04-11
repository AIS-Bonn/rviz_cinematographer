#include "multi_object_tracking/hypothesis.h"


// #define DETECTION_RATE_INCREMENT 0.04f
#define DETECTION_RATE_INCREMENT 1.0f

namespace MultiHypothesisTracker
{

Hypothesis::Hypothesis()
{
  // NOTE: was dimensionality of 6
  m_mean.setZero();
  m_covariance.setIdentity();
  m_numStateDimensions = 3;

  m_last_measurement_time = 0;
  m_last_mean_with_measurement.setZero();
  m_is_first_position = true;
  m_velocity.setZero();
  m_max_velocity_in_track.setZero();
  m_born_time = 0;
  m_times_measured = 0;

  m_static_distance_threshold = 0.25;
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

void Hypothesis::initialize(const Measurement& measurement,
                            unsigned int id)
{
  m_mean(0) = measurement.pos(0);
  m_mean(1) = measurement.pos(1);
  m_mean(2) = measurement.pos(2);

  m_covariance = measurement.cov;

  m_color = measurement.color;
  m_first_position_in_track = m_mean;
  m_born_time = getTimeHighRes();
  m_times_measured = 1;

  m_last_measurement_time = getTimeHighRes();

  // TODO: should that be 1?
  m_detection_rate = 0.5f;
  m_misdetection_rate = 0.5f;

  m_id = id;
}

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

bool Hypothesis::isSpurious()
{
  double current_time = getTimeHighRes();

  // TODO: magic numbers to parameters or members if this if clause is useful
  if((current_time - m_born_time > 0.65) && m_times_measured <=1)
    return true;

  // TODO: Jan: should be a parameter
  if(current_time - m_last_measurement_time > 90)
  {
    return true;
  }
  else
  {
    // TODO: reenable but test effects and tune parameter
    double max_position_cov = getParameters().max_cov;

    Eigen::EigenSolver<Eigen::Matrix3f> eigen_solver(m_covariance);
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
    double distance_from_origin = (m_mean - m_first_position_in_track).norm();
    // TODO: test if check for max velocity only is better. or dist fram origin with a larger distance + check for max velocity to account for registration mistakes that statistically should keep the object position in a range around the origin
    if(distance_from_origin > m_static_distance_threshold /*&& (m_max_velocity_in_track.norm() > 0.85)*/)
      m_is_static = false;
  }
}

void Hypothesis::predict(float dt,
                         Eigen::Vector3f& control)
{
  m_kalman.predict(static_cast<float>(dt));

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
  m_kalman.correct(measurement.pos, measurement.cov);





  double curr_time = getTimeHighRes(); // TODO: unused - delete?
  double time_dif = measurement.time - m_previous_measurement.time;


  //Running average
  Eigen::Vector3f new_velocity;
  new_velocity = (m_mean - m_last_mean_with_measurement) / time_dif;
  new_velocity(2) = 0;
  double alpha=0.80;
  if(m_velocity(0)==0 && m_velocity(1)==0 && m_velocity(2)==0)
  {  //If it's the first velocity just integrate it so we don0t have this retency to movement
    m_velocity = (m_mean - m_last_mean_with_measurement) / time_dif;
    m_velocity(2) = 0;
  }
  else
  {
    m_velocity = m_velocity + alpha * (new_velocity - m_velocity);
  }


  //CAP
  double max_velocity = 1.4;   //1.4ms or 5kmh
  double slack = 1;
  if(m_velocity.norm() > max_velocity + slack){
    m_velocity.normalize();
    m_velocity *= (max_velocity + slack);
  }
  m_velocity(2)=0;

  //OBJECT ARE WAY TOO SLOW JUT MAKE THE VELOCITY 0
  m_velocity.fill(0);

  // std::cout << "---------velocity norm is ------------"<< m_velocity.two_norm() << '\n';
  if (m_velocity.norm() > m_max_velocity_in_track.norm()){
    m_max_velocity_in_track=m_velocity;
  }


  // TODO: check if necessary. correction should not happen for first position, as it is just initiated
  if (m_is_first_position){		//If this was the first position then the velocity which requiers a previous state is not valid
    m_velocity(0)=0;
    m_velocity(1)=0;
    m_velocity(2)=0;
  }

  m_last_mean_with_measurement = m_mean;
  m_is_first_position = false;
  m_previous_measurement = measurement;
  m_latest_measurement = measurement;
  m_last_measurement_time = getTimeHighRes();
}


std::shared_ptr<Hypothesis> HypothesisFactory::createHypothesis()
{
  return std::make_shared<Hypothesis>();
}

};
