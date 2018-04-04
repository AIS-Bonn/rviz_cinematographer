#ifndef __MULTIHYPOTHESISTRACKER_H__
#define __MULTIHYPOTHESISTRACKER_H__

#include <vector>
#include <string>
#include <algorithm>
#include <chrono>

#include <multi_object_tracking/hungarian.h>
#include <limits.h> // for INT_MAX

#include <iostream>
#include <map>

#include <multi_object_tracking/multi_object_hypothesis.h>



// JS: a generic multi hypothesis tracker
// of course, the measurement and state transition models have to be implemented for the specific task
// this class just implements the most basic models: 3D position, noisy velocity, and direct state measurement
// control input: robot pose difference to last prediction step (dx, dy, dtheta)




namespace MultiHypothesisTracker
{

class HypothesisFactory;

class MultiHypothesisTracker {
public:
  MultiHypothesisTracker( HypothesisFactory* hypothesisFactory );
  ~MultiHypothesisTracker();

  inline unsigned int getNumStateDimensions() { return m_numStateDimensions; }

  /**
   * @brief Calls predict for each hypothesis.
   *
   * @param[in] time_diff   time difference between last and current prediction
   * @param[in] control     for state transition model //TODO: better description
   */
  virtual void predict(double time_diff,
                       Eigen::Vector3d& control);

  // returns vector of assignments
  std::vector< unsigned int > correct_hungarian_simplified( const std::vector< Measurement >& measurements);

  /**
   * @brief Deletes all hypotheses that are too close to others.
   *
   * //TODO: implement a reasonable merging function.
   *
   * @param[in] distance_threshold  minimal distance two hypotheses have to have to not be "merged"
   */
  void mergeCloseHypotheses(double distance_threshold);

  inline std::vector< Hypothesis* >& getHypotheses() { return m_hypotheses; }
  Hypothesis* getHypothesisByID( unsigned int ID );

//  void clear(){ m_hypotheses.clear(); };

  /**
   * @brief Deletes hypotheses that are visible and spurious
   *
   * @see isVisible()
   * @see isSpurious()
   */
  void deleteSpuriosHypotheses();


  //params
  inline void setMaxMahalanobisDistance(double distance){ m_max_mahalanobis_distance = distance; }

protected:


  std::vector< Hypothesis* > m_hypotheses;
  unsigned int m_lastHypothesisID;
  unsigned int m_numStateDimensions;
  HypothesisFactory* m_hypothesisFactory;

  //Parameters
  // double m_merge_close_hypotheses_distance;
  double m_max_mahalanobis_distance;


// 		QMutex m_hypothesisMutex;
};







};

#endif //__MULTIHYPOTHESISTRACKER_H__
