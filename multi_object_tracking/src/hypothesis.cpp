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

void Hypothesis::initialize(const Measurement& measurement,
                            unsigned int id,
                            const std::string& label)
{
  // TODO: merge with hyopthesis3d
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

    Eigen::EigenSolver<Eigen::Matrix3d> eigen_solver(m_covariance);
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

void Hypothesis::predict(double dt,
                         Eigen::Vector3d& control)
{
  // control is neglected
  Eigen::Matrix3d stateTransitionMatrix, stateTransitionCovariance;

  stateTransitionModel(m_mean, stateTransitionMatrix, stateTransitionCovariance, m_mean, dt, control);

  verify_static();

  if(!m_is_static)
  {
    //If covariance is bigger than the maximum cov than do not update it anymore
    double max_position_cov = getParameters().max_cov;

    //TODO: test if result is same with the one below
    Eigen::EigenSolver<Eigen::Matrix3d> eigen_solver(m_covariance);
    auto eigen_values = eigen_solver.eigenvalues();

    if(eigen_values.col(0)[0].real() > max_position_cov || eigen_values.col(0)[1].real() > max_position_cov || eigen_values.col(0)[2].real() > max_position_cov)
    {
      //don't update
    }
    else
    {
      //update
      m_covariance = stateTransitionMatrix * m_covariance * stateTransitionMatrix.transpose() + stateTransitionCovariance;
    }
  }

  for(int i = 0; i < (int)m_covariance.rows(); i++)
    for(int j = i; j < (int)m_covariance.cols(); j++)
      m_covariance(i, j) = m_covariance(j, i);
}

void Hypothesis::correct(const Measurement& measurement)
{
  Eigen::Matrix3d measurementMatrix, measurementCovariance, kalmanGain;
  Eigen::Vector3d expectedMeasurement;
  measurementModel( expectedMeasurement, measurementMatrix, measurementCovariance, m_mean );

  Eigen::Matrix3d correctionCovariance = measurementMatrix * m_covariance * measurementMatrix.transpose() + measurement.cov;

  kalmanGain = m_covariance * measurementMatrix.transpose() * correctionCovariance.inverse();
  Eigen::Matrix3d identity = m_covariance;
  identity.setIdentity();

  double curr_time = getTimeHighRes(); // TODO: unused - delete?
  double time_dif = measurement.time - m_previous_measurement.time;

  m_mean = m_mean + kalmanGain * ( measurement.pos - expectedMeasurement );
  if(!m_is_static){  //TODO Not sure it's completely right
    m_covariance = ( identity - kalmanGain * measurementMatrix ) * m_covariance;
  }

  //Running average
  Eigen::Vector3d new_velocity;
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

  for(int i = 0; i < (int)m_covariance.rows(); i++)
    for(int j = i; j < (int)m_covariance.cols(); j++)
      m_covariance(i, j) = m_covariance(j, i);
}

void Hypothesis::stateTransitionModel(Eigen::Vector3d& predictedState,
                                      Eigen::Matrix3d& stateTransitionMatrix,
                                      Eigen::Matrix3d& stateTransitionCovariance,
                                      const Eigen::Vector3d& currentState,
                                      double dt,
                                      const Eigen::Vector3d& control )
{
  stateTransitionMatrix.setIdentity();
  // TODO: check what this was doing once
//  stateTransitionMatrix( 0, 3 ) = dt;
//  stateTransitionMatrix( 1, 4 ) = dt;
//  stateTransitionMatrix( 2, 5 ) = dt;

  stateTransitionCovariance.setIdentity();

  const TrackerParameters &params = getParameters();

  stateTransitionCovariance( 0, 0 ) = dt * params.cov_x_per_sec;
  stateTransitionCovariance( 1, 1 ) = dt * params.cov_y_per_sec;
  stateTransitionCovariance( 2, 2 ) = dt * params.cov_z_per_sec;
//  stateTransitionCovariance( 3, 3 ) = dt * ( params.cov_vx_per_sec + params.alpha_vx_vx_per_sec * params.alpha_vx_vx_per_sec * currentState( 3 ) * currentState( 3 ) + params.alpha_vx_vy_per_sec * params.alpha_vx_vy_per_sec * currentState( 4 ) * currentState( 4 ) + params.alpha_vx_vy_per_sec * params.alpha_vx_vy_per_sec * currentState( 5 ) * currentState( 5 ) );
//  stateTransitionCovariance( 4, 4 ) = dt * ( params.cov_vy_per_sec + params.alpha_vy_vy_per_sec * params.alpha_vy_vy_per_sec * currentState( 4 ) * currentState( 4 ) + params.alpha_vx_vy_per_sec * params.alpha_vx_vy_per_sec * currentState( 3 ) * currentState( 3 ) + params.alpha_vx_vy_per_sec * params.alpha_vx_vy_per_sec * currentState( 5 ) * currentState( 5 ) );
//  stateTransitionCovariance( 5, 5 ) = dt * ( params.cov_vz_per_sec + params.alpha_vz_vz_per_sec * params.alpha_vz_vz_per_sec * currentState( 5 ) * currentState( 5 ) + params.alpha_vx_vy_per_sec * params.alpha_vx_vy_per_sec * currentState( 3 ) * currentState( 3 ) + params.alpha_vx_vy_per_sec * params.alpha_vx_vy_per_sec * currentState( 4 ) * currentState( 4 ) );

  predictedState = stateTransitionMatrix * currentState;

  // add control influence
  // control: robot pose difference to last prediction step
  predictedState(0) = predictedState(0) - control(0);
  predictedState(1) = predictedState(1) - control(1);

  Eigen::Matrix2d R_control( 2, 2 );
  R_control( 0, 0 ) = cos( -control( 2 ) );
  R_control( 0, 1 ) = -sin( -control( 2 ) );
  R_control( 1, 0 ) = sin( -control( 2 ) );
  R_control( 1, 1 ) = cos( -control( 2 ) );

  Eigen::Vector2d predictedState2D( 2 );
  predictedState2D = R_control * predictedState.block(0, 0, 2, 2);
  predictedState(0) = predictedState2D(0);
  predictedState(1) = predictedState2D(1);
}

void Hypothesis::measurementModel(Eigen::Vector3d& expectedMeasurement,
                                  Eigen::Matrix3d& measurementMatrix,
                                  Eigen::Matrix3d& measurementCovariance,
                                  const Eigen::Vector3d& currentState)
{
  measurementMatrix.setIdentity();

  expectedMeasurement = measurementMatrix * currentState;

  measurementCovariance.setIdentity();
  double measurementStd = getParameters().measurementStd;
  measurementCovariance( 0, 0 ) = measurementStd * measurementStd;
  measurementCovariance( 1, 1 ) = measurementStd * measurementStd;
  measurementCovariance( 2, 2 ) = measurementStd * measurementStd;
}


std::shared_ptr<Hypothesis> HypothesisFactory::createHypothesis()
{
  return std::make_shared<Hypothesis>();
}

};
