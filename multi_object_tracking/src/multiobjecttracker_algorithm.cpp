#include <sys/time.h>
#include <vnl/vnl_vector_fixed.h>

#include "multiobjecttracker_algorithm.h"
#include <boost/concept_check.hpp>


namespace MultiObjectTracker
{

MultiObjectTrackerAlgorithm::MultiObjectTrackerAlgorithm()
: m_multi_hypothesis_tracker(new MultiObjectTracker::MultiObjectHypothesisFactory())
	, m_last_prediction_time(0)
{}

void MultiObjectTrackerAlgorithm::predictWithoutMeasurement(){

	vnl_vector_fixed< double, 3 > movement;
	movement( 0 ) = 0;
	movement( 1 ) = 0;
	movement( 2 ) = 0;

	double currentTime = MultiHypothesisTracker::getTimeHighRes();

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
