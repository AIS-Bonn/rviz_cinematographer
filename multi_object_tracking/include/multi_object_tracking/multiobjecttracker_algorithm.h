#ifndef __MULTI_OBJECT_TRACKER_ALGORITHM_H__
#define __MULTI_OBJECT_TRACKER_ALGORITHM_H__

#include <multi_object_tracking/multihypothesistracker.h>

namespace MultiObjectTracker
{

class MultiObjectTrackerAlgorithm
{
public:

  MultiObjectTrackerAlgorithm();

  void predictWithoutMeasurement();

  void objectDetectionDataReceived(std::vector<Measurement>& measurements,
                                   const std::string& sourceName);

  const std::vector<MultiHypothesisTracker::Hypothesis*>& getHypotheses();

/*		void lockHypotheses(){ m_multi_hypothesis_tracker.lockHypotheses(); }
  void unlockHypotheses() { m_multi_hypothesis_tracker.unlockHypotheses(); }*/

  inline MultiObjectHypothesis* getHypothesisByID(unsigned int ID)
  { return (MultiObjectHypothesis*)m_multi_hypothesis_tracker.getHypothesisByID(ID); }

  void clear(){ m_multi_hypothesis_tracker.clear(); }

  inline void set_merge_close_hypotheses_distance(double val){ m_merge_close_hypotheses_distance = val; }

  MultiHypothesisTracker::MultiHypothesisTracker m_multi_hypothesis_tracker;

private:
  // double m_lastMeasurementTime;
  double m_last_prediction_time;
  double m_merge_close_hypotheses_distance;
};

}

#endif
