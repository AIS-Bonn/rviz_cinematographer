#include <multi_object_tracking/multiobjecttracker_algorithm.h>

namespace MultiObjectTracker
{

MultiObjectTrackerAlgorithm::MultiObjectTrackerAlgorithm()
: m_multi_hypothesis_tracker(new MultiObjectTracker::MultiObjectHypothesisFactory())
	, m_last_prediction_time(0)
{}

void MultiObjectTrackerAlgorithm::predictWithoutMeasurement()
{
  double currentTime = MultiHypothesisTracker::getTimeHighRes();

	if(m_last_prediction_time > 0)
  {
    Eigen::Vector3d correct(0.0, 0.0, 0.0);
    m_multi_hypothesis_tracker.predict(currentTime - m_last_prediction_time, correct);
  }

	m_last_prediction_time = currentTime;

	m_multi_hypothesis_tracker.deleteSpuriosHypotheses();
}

void MultiObjectTrackerAlgorithm::objectDetectionDataReceived(const std::vector<Measurement>& measurements)
{
  predictWithoutMeasurement();

  m_multi_hypothesis_tracker.correct_hungarian_simplified(measurements);

  m_multi_hypothesis_tracker.mergeCloseHypotheses(m_merge_distance);
}

const std::vector<MultiHypothesisTracker::Hypothesis*>& MultiObjectTrackerAlgorithm::getHypotheses()
{
	return m_multi_hypothesis_tracker.getHypotheses();
}

MultiObjectHypothesis* MultiObjectTrackerAlgorithm::getHypothesisByID(unsigned int id)
{
  return (MultiObjectHypothesis*)m_multi_hypothesis_tracker.getHypothesisByID(id);
}

}
