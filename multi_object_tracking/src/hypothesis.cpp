#include "multihypothesistracker.h"

#include "vnl/vnl_inverse.h"
#include "vnl/algo/vnl_svd.h"
#include "vnl/algo/vnl_symmetric_eigensystem.h"

#include "hungarian.h"
#include <limits.h> // for INT_MAX

#include <sys/time.h>
#include <iostream>
#include <map>

// #define DETECTION_RATE_INCREMENT 0.04f
#define DETECTION_RATE_INCREMENT 1.0f

namespace MultiHypothesisTracker
{

double get_time_high_res (){
  // std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
  // auto duration = now.time_since_epoch();
  // double time_high_res= std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();

  timeval currTime;
  gettimeofday( &currTime, NULL );
  double time_high_res = ((double)currTime.tv_sec) + ((double)currTime.tv_usec) * 1e-6;

  return time_high_res;
}

Hypothesis::Hypothesis()
:	m_mean( vnl_vector< double >( 6 ) )
,	m_covariance( vnl_matrix< double >( 6, 6 ) )
,	m_numStateDimensions( 6 ) {
  m_visible = true;
  m_lastMeasurementTime=0;
  m_last_mean_with_measurement=	vnl_vector< double >(3);
  m_last_mean_with_measurement.fill(0);
  m_is_first_position=true;
  m_velocity = vnl_vector< double >(3);
  m_velocity.fill(0);
  m_max_velocity_in_track= vnl_vector<double> (3);
  m_max_velocity_in_track.fill(0);
  m_is_picked=false;
  m_born_time=0;
  m_times_measured=0;
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

  double currentTime = get_time_high_res();

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
    double maxPositionCov = getParameters().max_cov;

    vnl_symmetric_eigensystem< double > eigensystemPosition( m_covariance.extract( 3, 3 ) );
    // if( eigensystemPosition.get_eigenvalue( 0 ) > maxPositionCov || eigensystemPosition.get_eigenvalue( 1 ) > maxPositionCov || eigensystemPosition.get_eigenvalue( 2 ) > maxPositionCov ) {
    // 	return true;
    // }

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
      if (  (m_mean-m_first_position_in_track).two_norm() > 0.25  ){
        m_is_static=false;
        return;
      }
    }
  }


  // if (m_is_static){
  // 	if (  (m_mean-m_first_position_in_track).two_norm() > 0.4   &&  (m_max_velocity_in_track.two_norm() > 0.85)  ){
  // 		m_is_static=false;
  // 	}
  // }

}

void Hypothesis::predict( double dt, const vnl_vector< double >& control ) {
  // control is neglected
  vnl_matrix< double > stateTransitionMatrix, stateTransitionCovariance;

  // std::cout << "Hypothesis:predict dt is " << dt << '\n';
  // std::cout << "Hypothesis:predict state before prediction is " << m_mean << '\n';

  stateTransitionModel( m_mean, stateTransitionMatrix, stateTransitionCovariance, m_mean, dt, control );

  // std::cout << "Hypothesis:predict state after prediction is " << m_mean << '\n';

  verify_static();  //Verifies that the object is static or not


  if( !m_is_static){

    //If covariance is bigger than the maximum cov than do not update it anymore
    double maxPositionCov = getParameters().max_cov;
    vnl_symmetric_eigensystem< double > eigensystemPosition( m_covariance.extract( 3, 3 ) );
    if( eigensystemPosition.get_eigenvalue( 0 ) > maxPositionCov || eigensystemPosition.get_eigenvalue( 1 ) > maxPositionCov || eigensystemPosition.get_eigenvalue( 2 ) > maxPositionCov ) {
      //don't update
    }else{
      //update
      m_covariance = stateTransitionMatrix * m_covariance * stateTransitionMatrix.transpose() + stateTransitionCovariance;
    }


  }

  for( int i = 0; i < m_covariance.rows(); i++ )
    for( int j = i; j < m_covariance.cols(); j++ )
      m_covariance( i, j ) = m_covariance( j, i );


}

void Hypothesis::correct( const Measurement& measurement ) {
  vnl_matrix< double > measurementMatrix, measurementCovariance, kalmanGain;
  vnl_vector< double > expectedMeasurement;
  measurementModel( expectedMeasurement, measurementMatrix, measurementCovariance, m_mean );

  vnl_matrix< double > correctionCovariance = measurementMatrix * m_covariance * measurementMatrix.transpose() + measurement.cov;
  vnl_svd< double > svdCorrectionCovariance( correctionCovariance );
  vnl_matrix< double > invCorrectionCovariance = svdCorrectionCovariance.pinverse();

  kalmanGain = m_covariance * measurementMatrix.transpose() * invCorrectionCovariance;
  vnl_matrix< double > identity = m_covariance;
  identity.set_identity();


  double curr_time = get_time_high_res();
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
  vnl_vector<double> new_velocity = vnl_vector<double>(3);
  new_velocity=(m_mean-m_last_mean_with_measurement)/time_dif; new_velocity(2)=0;
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
  if (m_velocity.magnitude() > max_velocity + slack ){
    m_velocity=m_velocity.normalize();
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
  if (m_velocity.two_norm() > m_max_velocity_in_track.two_norm()){
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
  m_lastMeasurementTime = get_time_high_res();



  for( int i = 0; i < m_covariance.rows(); i++ )
    for( int j = i; j < m_covariance.cols(); j++ )
      m_covariance( i, j ) = m_covariance( j, i );


}

void Hypothesis::stateTransitionModel( vnl_vector< double >& predictedState, vnl_matrix< double >& stateTransitionMatrix, vnl_matrix< double >& stateTransitionCovariance, const vnl_vector< double >& currentState, double dt, const vnl_vector< double >& control ) {

  stateTransitionMatrix = vnl_matrix< double >( 6, 6 );
  stateTransitionMatrix.set_identity();
  stateTransitionMatrix( 0, 3 ) = dt;
  stateTransitionMatrix( 1, 4 ) = dt;
  stateTransitionMatrix( 2, 5 ) = dt;

//		std::cout << dt << "\n";

  stateTransitionCovariance = vnl_matrix< double >( 6, 6 );
  stateTransitionCovariance.set_identity();

  const TrackerParameters &params = getParameters();

  stateTransitionCovariance( 0, 0 ) = dt * params.cov_x_per_sec;
  stateTransitionCovariance( 1, 1 ) = dt * params.cov_y_per_sec;
  stateTransitionCovariance( 2, 2 ) = dt * params.cov_z_per_sec;
  stateTransitionCovariance( 3, 3 ) = dt * ( params.cov_vx_per_sec + params.alpha_vx_vx_per_sec * params.alpha_vx_vx_per_sec * currentState( 3 ) * currentState( 3 ) + params.alpha_vx_vy_per_sec * params.alpha_vx_vy_per_sec * currentState( 4 ) * currentState( 4 ) + params.alpha_vx_vy_per_sec * params.alpha_vx_vy_per_sec * currentState( 5 ) * currentState( 5 ) );
  stateTransitionCovariance( 4, 4 ) = dt * ( params.cov_vy_per_sec + params.alpha_vy_vy_per_sec * params.alpha_vy_vy_per_sec * currentState( 4 ) * currentState( 4 ) + params.alpha_vx_vy_per_sec * params.alpha_vx_vy_per_sec * currentState( 3 ) * currentState( 3 ) + params.alpha_vx_vy_per_sec * params.alpha_vx_vy_per_sec * currentState( 5 ) * currentState( 5 ) );
  stateTransitionCovariance( 5, 5 ) = dt * ( params.cov_vz_per_sec + params.alpha_vz_vz_per_sec * params.alpha_vz_vz_per_sec * currentState( 5 ) * currentState( 5 ) + params.alpha_vx_vy_per_sec * params.alpha_vx_vy_per_sec * currentState( 3 ) * currentState( 3 ) + params.alpha_vx_vy_per_sec * params.alpha_vx_vy_per_sec * currentState( 4 ) * currentState( 4 ) );

  predictedState = stateTransitionMatrix * currentState;





  // add control influence
  // control: robot pose difference to last prediction step
  predictedState(0) = predictedState(0) - control(0);
  predictedState(1) = predictedState(1) - control(1);

  vnl_matrix< double > R_control( 2, 2 );
  R_control( 0, 0 ) = cos( -control( 2 ) );
  R_control( 0, 1 ) = -sin( -control( 2 ) );
  R_control( 1, 0 ) = sin( -control( 2 ) );
  R_control( 1, 1 ) = cos( -control( 2 ) );

  vnl_vector< double > predictedState2D( 2 );
  predictedState2D = R_control * predictedState.extract( 2 );
  predictedState(0) = predictedState2D(0);
  predictedState(1) = predictedState2D(1);


}

void Hypothesis::measurementModel( vnl_vector< double >& expectedMeasurement, vnl_matrix< double >& measurementMatrix, vnl_matrix< double >& measurementCovariance, const vnl_vector< double >& currentState ) {

  measurementMatrix = vnl_matrix< double >( 3, 6 );
  measurementMatrix.fill( 0 );
  measurementMatrix( 0, 0 ) = 1;
  measurementMatrix( 1, 1 ) = 1;
  measurementMatrix( 2, 2 ) = 1;

  expectedMeasurement = vnl_vector< double >( 3 );
  expectedMeasurement = measurementMatrix * currentState;

  measurementCovariance = vnl_matrix< double >( 3, 3 );
  measurementCovariance.set_identity();
  double measurementStd = getParameters().measurementStd;
  measurementCovariance( 0, 0 ) = measurementStd * measurementStd;
  measurementCovariance( 1, 1 ) = measurementStd * measurementStd;
  measurementCovariance( 2, 2 ) = measurementStd * measurementStd;

}

};
