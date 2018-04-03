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

void MultiObjectTrackerAlgorithm::predictWithoutMeasurement(){

	vnl_vector_fixed< double, 3 > movement;
	movement( 0 ) = 0;
	movement( 1 ) = 0;
	movement( 2 ) = 0;

	double currentTime = MultiHypothesisTracker::get_time_high_res();

	if( m_last_prediction_time > 0 ) {
		m_multi_hypothesis_tracker.predict( currentTime - m_last_prediction_time, movement);
	}
	m_last_prediction_time=currentTime;

	m_multi_hypothesis_tracker.deleteSpuriosHypotheses();


}

void MultiObjectTrackerAlgorithm::objectDetectionDataReceived(std::vector<Measurement>& measurements,
                                                              const std::string& sourceName)
{
  std::vector<MultiHypothesisTracker::Hypothesis*> hypotheses = m_multi_hypothesis_tracker.getHypotheses();

  predictWithoutMeasurement();

  // return m_multi_hypothesis_tracker.correctAmbiguous( measurements_legacy, true, &hypothesesFilter );
  // m_multi_hypothesis_tracker.correctAmbiguous_simplified( measurements, true, &hypothesesFilter );
  m_multi_hypothesis_tracker.correct_hungarian_simplified(measurements);
  m_multi_hypothesis_tracker.mergeCloseHypotheses( m_merge_close_hypotheses_distance );
}

const std::vector< MultiHypothesisTracker::Hypothesis* >& MultiObjectTrackerAlgorithm::getHypotheses() {
	return m_multi_hypothesis_tracker.getHypotheses();
}

}
