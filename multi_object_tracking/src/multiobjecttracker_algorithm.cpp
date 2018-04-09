#include <multi_object_tracking/multiobjecttracker_algorithm.h>

namespace MultiHypothesisTracker
{

MultiObjectTrackerAlgorithm::MultiObjectTrackerAlgorithm()
: m_multi_hypothesis_tracker(std::make_shared<Hypothesis3DFactory>())
	, m_last_prediction_time(0)
{}

void MultiObjectTrackerAlgorithm::predict()
{
  double currentTime = getTimeHighRes();

	if(m_last_prediction_time > 0)
  {
    Eigen::Vector3d control(0.0, 0.0, 0.0);
    m_multi_hypothesis_tracker.predict(currentTime - m_last_prediction_time, control);
  }

	m_last_prediction_time = currentTime;

	m_multi_hypothesis_tracker.deleteSpuriosHypotheses();
}

void MultiObjectTrackerAlgorithm::objectDetectionDataReceived(const std::vector<Measurement>& measurements)
{
  predict();

  m_multi_hypothesis_tracker.correct(measurements);

  m_multi_hypothesis_tracker.mergeCloseHypotheses(m_merge_distance);
}

const std::vector<std::shared_ptr<Hypothesis>>& MultiObjectTrackerAlgorithm::getHypotheses()
{
	return m_multi_hypothesis_tracker.getHypotheses();
}

std::shared_ptr<Hypothesis3D> MultiObjectTrackerAlgorithm::getHypothesisByID(unsigned int id)
{
  return std::static_pointer_cast<Hypothesis3D>(m_multi_hypothesis_tracker.getHypothesisByID(id));
}

}
