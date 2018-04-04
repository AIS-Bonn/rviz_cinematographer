#include "multi_object_tracking/hypothesis_3D.h"

namespace MultiHypothesisTracker
{

	Hypothesis3D::Hypothesis3D() {
		m_mean.setZero();
		m_covariance.setIdentity();
		m_numStateDimensions = 3; // TODO prob delete
		m_color='U';
		m_is_static=true;
		// m_last_prediction_time=-1;
	}

	Hypothesis3D::~Hypothesis3D() {
	}

	const TrackerParameters& Hypothesis3D::getParameters()
	{
		static TrackerParameters params = {
			0.001,	// cov_x_per_sec
			0.001,	// cov_y_per_sec
			0.001,	// cov_z_per_sec

			0.0,	// cov_vx_per_sec (dont care for 3D)
			0.0,	// cov_vy_per_sec (dont care for 3D)
			0.0,	// cov_vz_per_sec (dont care for 3D)

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

	void Hypothesis3D::initialize(const Measurement& measurement,
                                unsigned int id,
                                const std::string& label/*, const QColor& color*/)
  {
		m_mean( 0 ) = measurement.pos( 0 );
		m_mean( 1 ) = measurement.pos( 1 );
		m_mean( 2 ) = measurement.pos( 2 );

		m_color=measurement.color;
		m_first_position_in_track = m_mean;
		m_born_time=getTimeHighRes();
		verify_static();  //Verifies that the object is static or not
		m_times_measured++;


		m_covariance.setIdentity();
		// m_covariance *= getParameters().init_cov;
		m_covariance=measurement.cov;

		// std::cout << "initialize hypothesis with color " << measurement.color << '\n';
		// std::cout << "init_cov is " << getParameters().init_cov << '\n';
		// std::cout << "measuremetnt cov is " << measurement.cov << '\n';


		m_lastMeasurementTime = getTimeHighRes();

		m_detectionRate = 0.5f;
		m_misdetectionRate = 0.5f;

		m_ID = id;

		if( label.length() > 0 )
			m_label = label;
		else {
			std::stringstream ss;
			ss << id;
			ss >> m_label;
		}

// 		m_color = color;
	}

Eigen::Vector3d Hypothesis3D::velocity_decay(Eigen::Vector3d velocity_in)
{
  double cur_time = getTimeHighRes();
  //Velocity stays the same up until 1 second then it decays
  double time_start_decay=1.0;
  double time_finish_decay=4.0;
  double dif= time_finish_decay-time_start_decay;

  if (cur_time - m_lastMeasurementTime > time_start_decay){
    double weight = std::max (0.0, (time_finish_decay - (cur_time - m_lastMeasurementTime)) /dif ) ;
    return velocity_in*weight;
  }else{
    return velocity_in;
  }

}


	void Hypothesis3D::stateTransitionModel( Eigen::Vector3d& predictedState,
                                           Eigen::Matrix3d& stateTransitionMatrix,
                                           Eigen::Matrix3d& stateTransitionCovariance,
                                           const Eigen::Vector3d& currentState,
                                           double dt,
                                           const Eigen::Vector3d& control )
  {

    // TODO: this is only correct for zero velocity
		stateTransitionMatrix.setIdentity();

//		std::cout << dt << "\n";


		stateTransitionCovariance.setIdentity();

		const TrackerParameters &params = getParameters();


		stateTransitionCovariance(0, 0) = dt * params.cov_x_per_sec;
		stateTransitionCovariance(1, 1) = dt * params.cov_y_per_sec;
		stateTransitionCovariance(2, 2) = dt * params.cov_z_per_sec;





		predictedState = stateTransitionMatrix * currentState;

		//pseudo-velocity
		// std::cout << "stateTransitionModel:before velocity pred: " << predictedState << '\n';
		// std::cout << "stateTransitionModel:before velocity cur: " << currentState << '\n';
		// std::cout << "state transitin: velocity without time is " << velocity << '\n';
		// std::cout << "state transitin: time diff is " << dt << '\n';
		// std::cout << "state transitin: velocity with time is " << velocity*dt << '\n';
		if (!m_is_first_position && !m_is_static){  //If it's hte first position in the hypothesis then we don't have a velocity cuz we don't have a prev measurement
      Eigen::Vector3d velocity_decayed = velocity_decay(m_velocity);
			predictedState +=  velocity_decayed * (dt);

			// m_velocity=m_velocity*0.97; // TODO Not a very good way of doing decay becuase it depend on how often you do prediction
		}

		// std::cout << "stateTransitionModel:after velocity pred: " << predictedState << '\n';
		// std::cout << "stateTransitionModel:after velocity cur: " << currentState << '\n';
		// m_last_mean=predictedState;


		// add control influence
		// control: robot pose difference to last prediction step
		predictedState(0) = predictedState(0) - control(0);
		predictedState(1) = predictedState(1) - control(1);

		Eigen::Matrix2d R_control;
		R_control( 0, 0 ) = cos( -control( 2 ) );
		R_control( 0, 1 ) = -sin( -control( 2 ) );
		R_control( 1, 0 ) = sin( -control( 2 ) );
		R_control( 1, 1 ) = cos( -control( 2 ) );

    Eigen::Vector2d predictedState2D;
		predictedState2D = R_control * predictedState.block(0,0,2,2);
		predictedState(0) = predictedState2D(0);
		predictedState(1) = predictedState2D(1);


	}

void Hypothesis3D::measurementModel(Eigen::Vector3d& expectedMeasurement,
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


Hypothesis* Hypothesis3DFactory::createHypothesis() {
	return new Hypothesis3D();
}

};
