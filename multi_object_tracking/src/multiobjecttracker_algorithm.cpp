#include <sys/time.h>
#include <vnl/vnl_vector_fixed.h>

#include "multiobjecttracker_algorithm.h"
#include <boost/concept_check.hpp>


namespace MultiObjectTracker {


const MultiHypothesisTracker::TrackerParameters& MultiObjectHypothesis::getParameters()
{
	static MultiHypothesisTracker::TrackerParameters params = {
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

MultiObjectHypothesis::MultiObjectHypothesis()  {
}

MultiObjectHypothesis::~MultiObjectHypothesis() {
}


MultiObjectTrackerAlgorithm::MultiObjectTrackerAlgorithm()
: m_multi_hypothesis_tracker(new MultiObjectTracker::MultiObjectHypothesisFactory())
	, m_last_prediction_time(0)
{}

void MultiObjectTrackerAlgorithm::updateFilterWithPrediction( MultiHypothesisTracker::HypothesisFilter* filter ) {
	updateFilter( 0, 0, 0, filter );
}

void MultiObjectTrackerAlgorithm::updateFilter( double x, double y, double a, MultiHypothesisTracker::HypothesisFilter* filter ) {
	vnl_vector_fixed< double, 3 > movement;
	movement( 0 ) = x;
	movement( 1 ) = y;
	movement( 2 ) = a;

	double currentTime = MultiHypothesisTracker::get_time_high_res();

	// if( m_lastMeasurementTime > 0 ) {
	// 	m_multi_hypothesis_tracker.predict( currentTime - m_lastMeasurementTime, movement, filter );
	// }
	// m_lastMeasurementTime = currentTime;

	if( m_last_prediction_time > 0 ) {
		m_multi_hypothesis_tracker.predict( currentTime - m_last_prediction_time, movement, filter );
	}
	m_last_prediction_time=currentTime;
	// m_lastMeasurementTime = currentTime;

}


void MultiObjectTrackerAlgorithm::predictWithoutMeasurement(){

	vnl_vector_fixed< double, 3 > movement;
	movement( 0 ) = 0;
	movement( 1 ) = 0;
	movement( 2 ) = 0;

	double currentTime = MultiHypothesisTracker::get_time_high_res();

	HypothesisFilterBySource hypothesesFilter( "all" );

	if( m_last_prediction_time > 0 ) {
		m_multi_hypothesis_tracker.predict( currentTime - m_last_prediction_time, movement, &hypothesesFilter );
	}
	m_last_prediction_time=currentTime;

	m_multi_hypothesis_tracker.deleteSpuriosHypotheses(  &hypothesesFilter );


}

void MultiObjectTrackerAlgorithm::objectDetectionDataReceived(std::vector<Measurement>& measurements,
																															const std::string& sourceName)
{
//	HypothesisFilterBySourceAndFrustum hypothesesFilter( sourceName, frustum );
	HypothesisFilterBySource hypothesesFilter( sourceName );
	std::vector< MultiHypothesisTracker::Hypothesis* > hypotheses = hypothesesFilter.filter( m_multi_hypothesis_tracker.getHypotheses() );


	//make mesurement in previos format
	// std::vector< vnl_vector<double> > measurements_legacy;
	// for (size_t i = 0; i < measurements.size(); i++) {
	// 	measurements_legacy.push_back(measurements[i].pos);
	// }

	predictWithoutMeasurement();

	// return m_multi_hypothesis_tracker.correctAmbiguous( measurements_legacy, true, &hypothesesFilter );
	// m_multi_hypothesis_tracker.correctAmbiguous_simplified( measurements, true, &hypothesesFilter );
	m_multi_hypothesis_tracker.correct_hungarian_simplified( measurements, &hypothesesFilter );
	m_multi_hypothesis_tracker.mergeCloseHypotheses( m_merge_close_hypotheses_distance );
}



const std::vector< MultiHypothesisTracker::Hypothesis* >& MultiObjectTrackerAlgorithm::getHypotheses() {
	return m_multi_hypothesis_tracker.getHypotheses();
}


bool HypothesisFilterBySource::passthrough( MultiHypothesisTracker::Hypothesis* hypothesis ) {

	// MultiObjectHypothesis* hyp = (MultiObjectHypothesis*) hypothesis;
	// if( hyp->getSource() == m_source )
	// 	return true;
	// else
	// 	return false;
	return true;

}

std::vector< MultiHypothesisTracker::Hypothesis* > HypothesisFilterBySource::filter( const std::vector< MultiHypothesisTracker::Hypothesis* >& hypotheses ) {

	// std::vector< MultiHypothesisTracker::Hypothesis* > filtered;
	// for( unsigned int i = 0; i < hypotheses.size(); i++ ) {
	//
	// 	MultiObjectHypothesis* hyp = (MultiObjectHypothesis*) hypotheses[i];
	// 	if( passthrough( hyp ) )
	// 		filtered.push_back( hyp );
	//
	// }
	//
	// return filtered;

	return hypotheses;
}


//bool HypothesisFilterBySourceAndFrustum::passthrough( MultiHypothesisTracker::Hypothesis* hypothesis ) {
//
//	MultiObjectHypothesis* hyp = (MultiObjectHypothesis*) hypothesis;
//	if( hyp->getSource() != m_source ) {
//		ROS_ERROR("track %i wrong source %s -- %s", hyp->getID(), hyp->getSource().c_str(), m_source.c_str());
//		return false;
//	}
//
//	if( m_frustum.contains( "base_link", hyp->getMean() ) ) {
//		hyp->setVisible( true );
//		return true;
//	}
//	else {
//		ROS_ERROR("!!!!!!!!!! track %i not in frustum!", hyp->getID());
//		hyp->setVisible( false );
//		return false;
//	}
//
//}
//
//std::vector< MultiHypothesisTracker::Hypothesis* > HypothesisFilterBySourceAndFrustum::filter( const std::vector< MultiHypothesisTracker::Hypothesis* >& hypotheses ) {
//
//	std::vector< MultiHypothesisTracker::Hypothesis* > filtered;
//	for( unsigned int i = 0; i < hypotheses.size(); i++ ) {
//
//		MultiObjectHypothesis* hyp = (MultiObjectHypothesis*) hypotheses[i];
//		if( passthrough( hyp ) )
//			filtered.push_back( hyp );
//
//	}
//
//	return filtered;
//}


}
