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

  m_visible = true;
  m_lastMeasurementTime = 0;
  m_last_mean_with_measurement.setZero();
  m_is_first_position = true;
  m_velocity.setZero();
  m_max_velocity_in_track.setZero();
  m_is_picked = false;
  m_born_time = 0;
  m_times_measured = 0;
}

Hypothesis::~Hypothesis() {
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

void Hypothesis::initialize( const Measurement& measurement, unsigned int id, const std::string& label/*, const QColor& color*/ ) {
  // m_mean.fill( 0 );
  // m_mean( 0 ) = measurement( 0 );
  // m_mean( 1 ) = measurement( 1 );
  // m_mean( 2 ) = measurement( 2 );
  //
  // m_covariance.set_identity();
  // m_covariance *= getParameters().init_cov;
  //
  // timeval currTime;
  // gettimeofday( &currTime, NULL );
  // m_lastMeasurementTime = ((double)currTime.tv_sec) + ((double)currTime.tv_usec) * 1e-6;
  //
  // m_detectionRate = 0.5f;
  // m_misdetectionRate = 0.5f;
  //
  // m_ID = id;
  //
  // if( label.length() > 0 )
  // 	m_label = label;
  // else {
  // 	std::stringstream ss;
  // 	ss << id;
  // 	ss >> m_label;
  // }

// 		m_color = color;
}

void Hypothesis::detected() {
  m_detectionRate += DETECTION_RATE_INCREMENT;
  float sumDetectionRate = m_detectionRate + m_misdetectionRate;
  m_detectionRate /= sumDetectionRate;
  m_misdetectionRate /= sumDetectionRate;
}

void Hypothesis::undetected() {
  m_misdetectionRate += DETECTION_RATE_INCREMENT;
  float sumDetectionRate = m_detectionRate + m_misdetectionRate;
  m_detectionRate /= sumDetectionRate;
  m_misdetectionRate /= sumDetectionRate;
}

bool Hypothesis::isSpurious() {

  double currentTime = getTimeHighRes();

  //static object that went more than 2 seconds with less than 3 detections
  //theme may just be missdetections of the laser that gives really briefly a measurement for an object
  // if (m_is_static && (currentTime - m_born_time > 1.0) && m_times_measured <4  ){
  // 	return true;
  // }

  if ((currentTime - m_born_time > 0.65) && m_times_measured <=1  ){
    return true;
  }

// TODO: Jan: might not be the case for sparse data. 
  if (m_is_static){
    return false;
  }


// TODO: Jan: should be a parameter
  if( currentTime - m_lastMeasurementTime > 90 ){ //more than x second and the track will be deleted
    return true;
  }else {
    double maxPositionCov = getParameters().max_cov; // TODO unused - delete?

    Eigen::EigenSolver<Eigen::Matrix3d> eigen_solver(m_covariance);
//     if( eigen_solver.eigenvalues().col(0)[0] > maxPositionCov || eigen_solver.eigenvalues().col(0)[1] > maxPositionCov || eigen_solver.eigenvalues().col(0)[2] > maxPositionCov ) {
//     	return true;
//     }

    // if (!m_is_static){
    // 	std::cout << "------------------------" << '\n';
    // 	std::cout << "eigensystm: " << eigensystemPosition.get_eigenvalue( 0 ) << '\n';
    // }

    return false;
  }
}

// TODO: Jan: get rid of color stuff and add parameters, or at least compare to member variable
void Hypothesis::verify_static(){

  //In the case that it is a movable object it will forever be flagged as movable, ie, non-static
  // std::cout << "verfy_static: dist is " << (m_mean-m_first_position_in_track).two_norm() << " and flag is " << m_is_static << '\n';
  // std::cout << "verfy_static: max_velocity is  is " << m_max_velocity_in_track.two_norm() << " and flag is " << m_is_static << '\n';

  if (m_is_static){
    if (m_color=='R' || m_color=='B' || m_color=='G' || m_color== 'O') {   //If it's red, green, blue, orange we know it's static
      m_is_static=true;
      return;
    }else if (m_color=='Y' ){						//If it's yellow we know it's gonna move
      m_is_static=false;
      return;
    }else{																							//If it's of unknown color we have to check how much it moved
      if (  (m_mean-m_first_position_in_track).norm() > 0.25  ){
        m_is_static=false;
        return;
      }
    }
  }


  // if (m_is_static){
  // 	if (  (m_mean-m_first_position_in_track).norm() > 0.4   &&  (m_max_velocity_in_track.norm() > 0.85)  ){
  // 		m_is_static=false;
  // 	}
  // }

}

void Hypothesis::predict(double dt, Eigen::Vector3d& control)
{
  // control is neglected
  Eigen::Matrix3d stateTransitionMatrix, stateTransitionCovariance;

  // std::cout << "Hypothesis:predict dt is " << dt << '\n';
  // std::cout << "Hypothesis:predict state before prediction is " << m_mean << '\n';

  stateTransitionModel( m_mean, stateTransitionMatrix, stateTransitionCovariance, m_mean, dt, control );

  // std::cout << "Hypothesis:predict state after prediction is " << m_mean << '\n';

  verify_static();  //Verifies that the object is static or not


  if( !m_is_static){

    //If covariance is bigger than the maximum cov than do not update it anymore
    double maxPositionCov = getParameters().max_cov;

    //TODO: test if result is same with the one below
    Eigen::EigenSolver<Eigen::Matrix3d> eigen_solver(m_covariance);
    auto eigen_values = eigen_solver.eigenvalues();

    if( eigen_values.col(0)[0].real() > maxPositionCov || eigen_values.col(0)[1].real() > maxPositionCov || eigen_values.col(0)[2].real() > maxPositionCov ) {
      //don't update
    }else{
      //update
      m_covariance = stateTransitionMatrix * m_covariance * stateTransitionMatrix.transpose() + stateTransitionCovariance;
    }


  }

  for( int i = 0; i < (int)m_covariance.rows(); i++ )
    for( int j = i; j < (int)m_covariance.cols(); j++ )
      m_covariance( i, j ) = m_covariance( j, i );


}

void Hypothesis::correct( const Measurement& measurement ) {
  Eigen::Matrix3d measurementMatrix, measurementCovariance, kalmanGain;
  Eigen::Vector3d expectedMeasurement;
  measurementModel( expectedMeasurement, measurementMatrix, measurementCovariance, m_mean );

  Eigen::Matrix3d correctionCovariance = measurementMatrix * m_covariance * measurementMatrix.transpose() + measurement.cov;

  kalmanGain = m_covariance * measurementMatrix.transpose() * correctionCovariance.inverse();
  Eigen::Matrix3d identity = m_covariance;
  identity.setIdentity();


  double curr_time = getTimeHighRes(); // TODO: unused - delete?
  // double time_dif= curr_time-m_last_mean_time;
  // double time_dif= curr_time-m_lastMeasurementTime;  //Should be something like time_dif=measurement.time - previous_meaurement.time
  double time_dif= measurement.time-m_previous_measurement.time;



  // double time_dif= curr_time-m_last_prediction_time;
  // std::cout << "time dif is " << time_dif << '\n';
  // std::cout << "cur time: " << curr_time << '\n';
  // std::cout << "m_last_mean time: " << m_last_mean_time << '\n';
  // bool invalidate_velocity=false;
  // if (m_lastMeasurementTime<=0){     //This si the case when this is the first correction therfore there is not previous measurement
  // 	invalidate_velocity=true;
  // }


  m_mean = m_mean + kalmanGain * ( measurement.pos - expectedMeasurement );
  if (!m_is_static){  //TODO Not sure it's completely right
    m_covariance = ( identity - kalmanGain * measurementMatrix ) * m_covariance;
  }


  // m_last_mean_time = curr_time;

  // m_velocity=(m_mean-m_last_mean_with_measurement)/time_dif; m_velocity(2)=0;  //TODO cap velocity a certain max


  //Running average
  Eigen::Vector3d new_velocity;
  new_velocity = (m_mean - m_last_mean_with_measurement) / time_dif;
  new_velocity(2) = 0;
  double alpha=0.80;
  if (m_velocity(0)==0 && m_velocity(1)==0 && m_velocity(2)==0){  //If it's the first velocity just integrate it so we don0t have this retency to movement
    m_velocity=(m_mean-m_last_mean_with_measurement)/time_dif; m_velocity(2)=0;
  }else{
    m_velocity= m_velocity + alpha*(new_velocity-m_velocity);
  }


  //CAP
  // std::cout << "velocity is " << m_velocity.magnitude() << '\n';
  double max_velocity=1.4;   //1.4ms or 5kmh
  double slack=1;
  if (m_velocity.norm() > max_velocity + slack ){
    m_velocity.normalize();
    // std::cout << "over-----" << '\n';
    m_velocity*=(max_velocity+slack);
  }
  // std::cout << "velocity is " << m_velocity.magnitude() << '\n';
  m_velocity(2)=0;


  //OBJECT ARE WAY TOO SLOW JUT MAKE THE VELOCITY 0
  m_velocity.fill(0);




  //TODO Correct velocity depending on the angle between the base link and the base_footprint
  //get_drone_





  // std::cout << "---------velocity norm is ------------"<< m_velocity.two_norm() << '\n';
  if (m_velocity.norm() > m_max_velocity_in_track.norm()){
    m_max_velocity_in_track=m_velocity;
  }


  // std::cout << "----------------------------------------" << '\n';
  // std::cout << "correct of hypothesis " << this->getID() << '\n';
  // std::cout << "m_mean is " << m_mean << '\n';
  // std::cout << "m_last_mean_with_measurement is " << m_last_mean_with_measurement << '\n';
  // std::cout << "time diff is " << time_dif << '\n';
  // std::cout << "m_lastMeasurementTime is " << m_lastMeasurementTime << '\n';
  // std::cout << "velocity is " << m_velocity << '\n';


  if (m_is_first_position){		//If this was the first position then the velocity which requiers a previous state is not valid
    m_velocity(0)=0;
    m_velocity(1)=0;
    m_velocity(2)=0;
  }
  // if (velocity.two_norm()>30){
  // 	velocity=velocity.normalize();
  // 	velocity=velocity*0.2;
  // }
  // if (velocity.two_norm()<0.15){
  // 	velocity(0)=0;
  // 	velocity(1)=0;
  // }



  m_last_mean_with_measurement=m_mean;
  m_is_first_position=false;
  m_previous_measurement=measurement;
  m_latest_measurement=measurement;
  m_lastMeasurementTime = getTimeHighRes();



  for( int i = 0; i < (int)m_covariance.rows(); i++ )
    for( int j = i; j < (int)m_covariance.cols(); j++ )
      m_covariance( i, j ) = m_covariance( j, i );


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

//		std::cout << dt << "\n";

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
