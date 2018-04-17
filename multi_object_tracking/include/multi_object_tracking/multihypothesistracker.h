#ifndef __MULTIHYPOTHESISTRACKER_H__
#define __MULTIHYPOTHESISTRACKER_H__

#include <vector>
#include <string>
#include <algorithm>
#include <chrono>

#include <multi_object_tracking/hungarian.h>
#include <limits.h> // for INT_MAX

#include <iostream>

#include <multi_object_tracking/hypothesis.h>


// JS: a generic multi hypothesis tracker
// of course, the measurement and state transition models have to be implemented for the specific task
// this class just implements the most basic models: 3D position, noisy velocity, and direct state measurement
// control input: robot pose difference to last prediction step (dx, dy, dtheta)




namespace MultiHypothesisTracker
{

class HypothesisFactory;

class MultiHypothesisTracker {
public:
  MultiHypothesisTracker(std::shared_ptr<HypothesisFactory> hypothesis_factory);
  ~MultiHypothesisTracker();

  /**
   * @brief Calls predict for each hypothesis.
   *
   * @param[in] time_diff   time difference between last and current prediction
   */
  virtual void predict(double time_diff);

  /**
   * @brief Calls predict for each hypothesis.
   *
   * @param[in] time_diff   time difference between last and current prediction
   * @param[in] control     for state transition model //TODO: better description
   */
  virtual void predict(double time_diff,
                       Eigen::Vector3f& control);

  // returns vector of assignments
  void correct(const std::vector<Measurement>& measurements);

  /**
   * @brief Deletes all hypotheses that are too close to others.
   *
   * //TODO: implement a reasonable merging function.
   *
   * @param[in] distance_threshold  minimal distance two hypotheses have to have to not be "merged"
   */
  void mergeCloseHypotheses(double distance_threshold);

  inline std::vector<std::shared_ptr<Hypothesis>>& getHypotheses() { return m_hypotheses; }
  std::shared_ptr<Hypothesis> getHypothesisByID( unsigned int ID );

  void clear(){ m_hypotheses.clear(); };

  /**
   * @brief Deletes hypotheses that are visible and spurious
   *
   * @see isVisible()
   * @see isSpurious()
   */
  void deleteSpuriosHypotheses(double current_time);

  inline void setMaxMahalanobisDistance(double distance){ m_max_mahalanobis_distance = distance; }

protected:
  std::vector<std::shared_ptr<Hypothesis>> m_hypotheses;
  unsigned int m_lastHypothesisID;
  std::shared_ptr<HypothesisFactory> m_hypothesisFactory;

  int m_cost_factor;
  double m_max_mahalanobis_distance;

  /**
   * @brief Set up cost matrix for hungarian method.
   *
   * //TODO: better description
   * Pairwise thresholded Mahalanobis distance of each measurement to each
   * hypothesis in top left block of matrix.
   * Max values in top right and bottom left blocks.
   * Zeroes in bottom right block.
   *
   * @param[in]     measurements    detections.
   * @param[in]     hypotheses      already existing hypotheses.
   * @param[out]    cost_matrix     cost matrix.
   */
  void setupCostMatrix(const std::vector<Measurement>& measurements,
                       std::vector<std::shared_ptr<Hypothesis>>& hypotheses,
                       int**& cost_matrix);

  /**
   * @brief Use assignments to correct each hypothesis with assigned measurement.
   *
   * //TODO: description
   *
   * @param[in]     hung            assignments and costs from hungarian method.
   * @param[in]     measurements    detections.
   * @param[in,out] hypotheses      in current hypotheses, out corrected and new hypotheses.
   */
  void assign(const hungarian_problem_t& hung,
              const std::vector<Measurement>& measurements,
              std::vector<std::shared_ptr<Hypothesis>>& hypotheses);
};

};

#endif //__MULTIHYPOTHESISTRACKER_H__