#include <multi_object_tracking/multiobjecttracker_algorithm.h>

namespace MultiHypothesisTracker
{

MultiObjectTrackerAlgorithm::MultiObjectTrackerAlgorithm()
: m_multi_hypothesis_tracker(std::make_shared<HypothesisFactory>())
	, m_last_prediction_time(0)
{}

void MultiObjectTrackerAlgorithm::predict(double prediction_time)
{
	if(m_last_prediction_time > 0)
    m_multi_hypothesis_tracker.predict(prediction_time - m_last_prediction_time);

	m_last_prediction_time = prediction_time;

	m_multi_hypothesis_tracker.deleteSpuriosHypotheses(prediction_time);
}

void MultiObjectTrackerAlgorithm::objectDetectionDataReceived(const std::vector<Measurement>& measurements)
{
  if(measurements.empty())
    return;

  predict(measurements.at(0).time);

  m_multi_hypothesis_tracker.correct(measurements);

  m_multi_hypothesis_tracker.mergeCloseHypotheses(m_merge_distance);
}

const std::vector<std::shared_ptr<Hypothesis>>& MultiObjectTrackerAlgorithm::getHypotheses()
{
	return m_multi_hypothesis_tracker.getHypotheses();
}

std::shared_ptr<Hypothesis> MultiObjectTrackerAlgorithm::getHypothesisByID(unsigned int id)
{
  return std::static_pointer_cast<Hypothesis>(m_multi_hypothesis_tracker.getHypothesisByID(id));
}

}
