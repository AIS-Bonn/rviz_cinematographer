#include <multi_object_tracking/multiobjecttracker_algorithm.h>

#include <iomanip>

namespace MultiObjectTracker
{

MultiObjectTrackerAlgorithm::MultiObjectTrackerAlgorithm()
//: m_multi_hypothesis_tracker(new MultiHypothesisTracker::HypothesisFactory())
: m_multi_hypothesis_tracker(new MultiObjectTracker::MultiObjectHypothesisFactory())
	, m_last_prediction_time(0)
{}

void MultiObjectTrackerAlgorithm::predictWithoutMeasurement(){

  Eigen::Vector3d movement;
  movement(0) = 0;
	movement(1) = 0;
	movement(2) = 0;

  double currentTime = MultiHypothesisTracker::getTimeHighRes();

	if( m_last_prediction_time > 0 ) {
		m_multi_hypothesis_tracker.predict( currentTime - m_last_prediction_time, movement);
	}
	m_last_prediction_time = currentTime;

	m_multi_hypothesis_tracker.deleteSpuriosHypotheses();


}

void MultiObjectTrackerAlgorithm::objectDetectionDataReceived(std::vector<Measurement>& measurements,
                                                              const std::string& sourceName)
{
  // TODO: unused?
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
