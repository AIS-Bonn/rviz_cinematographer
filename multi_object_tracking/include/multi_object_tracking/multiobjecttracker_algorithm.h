#ifndef __MULTI_OBJECT_TRACKER_ALGORITHM_H__
#define __MULTI_OBJECT_TRACKER_ALGORITHM_H__

#include <multi_object_tracking/multihypothesistracker.h>

namespace MultiHypothesisTracker
{

class MultiObjectTrackerAlgorithm
{
public:

  MultiObjectTrackerAlgorithm();

  /**
   * @brief Calls prediction method of the hypotheses and deletes spurious hypothesis afterwards.
   */
  void predict();

  /**
   * @brief Predicts and corrects the hypotheses based on the new measurements.
   *
   * // TODO: rename
   *
   * @param measurements    new detections
   */
  void objectDetectionDataReceived(const std::vector<Measurement>& measurements);

  const std::vector<std::shared_ptr<Hypothesis>>& getHypotheses();

  std::shared_ptr<Hypothesis3D> getHypothesisByID(unsigned int id);

  void setMergeDistance(double distance){ m_merge_distance = distance; }

  MultiHypothesisTracker m_multi_hypothesis_tracker;

private:
  double m_last_prediction_time;
  double m_merge_distance;
};

}

#endif
