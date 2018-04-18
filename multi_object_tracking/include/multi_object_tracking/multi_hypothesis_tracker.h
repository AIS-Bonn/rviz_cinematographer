#ifndef __MULTI_HYPOTHESIS_TRACKER_H__
#define __MULTI_HYPOTHESIS_TRACKER_H__

#include <vector>
#include <iostream>
#include <limits.h> // for INT_MAX

#include <multi_object_tracking/hypothesis.h>
#include <multi_object_tracking/hungarian.h>


namespace MultiHypothesisTracker
{

class HypothesisFactory;

class MultiHypothesisTracker
{
public:
  explicit MultiHypothesisTracker(std::shared_ptr<HypothesisFactory> hypothesis_factory);
  ~MultiHypothesisTracker() = default;

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
   * @param[in] control     control input for state prediction
   */
  virtual void predict(double time_diff,
                       Eigen::Vector3f& control);


  void correct(const std::vector<Measurement>& measurements);

  /**
   * @brief Deletes hypotheses that are visible and spurious
   *
   * @see isVisible()
   * @see isSpurious()
   */
  void deleteSpuriosHypotheses(double current_time);

  /**
   * @brief Deletes all hypotheses that are too close to others.
   *
   * //TODO: implement a reasonable merging function.
   *
   * @param[in] distance_threshold  minimal distance two hypotheses have to have to not be "merged"
   */
  void mergeCloseHypotheses(double distance_threshold);

  inline std::vector<std::shared_ptr<Hypothesis>>& getHypotheses(){ return m_hypotheses; }

  inline void setMaxMahalanobisDistance(double distance){ m_max_mahalanobis_distance = distance; }

protected:
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
   * @brief Calculates distance between hypothesis and measurement.
   *
   * //TODO: decide which distance to take and adapt return tag
   *
   * @param[in] hyp_position        hypothesis position.
   * @param[in] hyp_covariance      hypothesis covariance.
   * @param[in] meas_position       measurement position.
   * @param[in] meas_covariance     measurement covariance.
   *
   * @return Some kind of Mahalanobis distance.
   */
  double distance(const Eigen::Vector3f& hyp_position,
                  const Eigen::Matrix3f& hyp_covariance,
                  const Eigen::Vector3f& meas_position,
                  const Eigen::Matrix3f& meas_covariance);

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


  std::shared_ptr<HypothesisFactory> m_hypothesis_factory;
  std::vector<std::shared_ptr<Hypothesis>> m_hypotheses;

  unsigned int m_current_hypothesis_id;

  int m_cost_factor;
  double m_max_mahalanobis_distance;
};

};

#endif //__MULTI_HYPOTHESIS_TRACKER_H__