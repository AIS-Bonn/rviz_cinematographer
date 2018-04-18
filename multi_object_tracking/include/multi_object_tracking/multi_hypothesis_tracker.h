/** @file
 *
 * Multi hypothesis tracker implementation.
 *
 * @author Jan Razlaw
 */

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

/**
 * @brief Multi hypothesis tracker class.
 */
class MultiHypothesisTracker
{
public:
  /** @brief Constructor. */
  explicit MultiHypothesisTracker(std::shared_ptr<HypothesisFactory> hypothesis_factory);
  /** @brief Destructor. */
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

  /**
   * @brief Uses Hungarian method to assign measurements to hypotheses and corrects the latter.
   *
   * @param[in] measurements    measurements
   */
  void correct(const std::vector<Measurement>& measurements);

  /**
   * @brief Deletes hypotheses that are spurious
   *
   * @see isSpurious()
   */
  void deleteSpuriosHypotheses(double current_time);

  /**
   * @brief Deletes all hypotheses that are too close to others.
   *
   * //TODO: implement a reasonable merging function.
   *
   * @param[in] distance_threshold  two hypotheses with a distance below that threshold are merged
   */
  void mergeCloseHypotheses(double distance_threshold);

  /** @brief Getter for #m_hypotheses. */
  inline std::vector<std::shared_ptr<Hypothesis>>& getHypotheses(){ return m_hypotheses; }

  /** @brief Setter for distance threshold #m_max_mahalanobis_distance. */
  inline void setMaxMahalanobisDistance(double distance){ m_max_mahalanobis_distance = (int)m_dist_scale * distance; }

protected:
  /**
   * @brief Set up cost matrix for hungarian method.
   *
   * Top left block: Pairwise thresholded Mahalanobis distance between each
   * measurement and each hypothesis.
   * Top right block: Fake distance of hypotheses to dummy measurements
   *    -> #m_max_mahalanobis_distance
   * Bottom left block: Fake distance of measurements to dummy hypotheses
   *    -> #m_max_mahalanobis_distance
   * Bottom right block: Fake distance between dummy measurements and dummy
   * hyptheses -> Zeroes.
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
   * @brief Use assignments for correction and initialization.
   *
   * If measurement assigned to hypothesis -> correct latter.
   * If measurement not assigned -> new hypothesis.
   * If hypothesis not assigned -> failed to detect.
   *
   * @param[in]     assignments     assignments from hungarian method.
   * @param[in]     cost_matrix     original cost_matrix hungarian method was initialized with.
   * @param[in]     measurements    detections.
   * @param[in,out] hypotheses      in current hypotheses, out corrected and new hypotheses.
   */
  void applyAssignments(int**& assignments,
                        int**& cost_matrix,
                        const std::vector<Measurement>& measurements,
                        std::vector<std::shared_ptr<Hypothesis>>& hypotheses);


  /** @brief Hypothesis factory.*/
  std::shared_ptr<HypothesisFactory> m_hypothesis_factory;
  /** @brief Vector storing all tracked hypotheses.*/
  std::vector<std::shared_ptr<Hypothesis>> m_hypotheses;

  /** @brief Counter for hypotheses IDs.*/
  unsigned int m_current_hypothesis_id;

  /** @brief Scale from double to int, because distance is in double but hungarian needs int costs.*/
  int m_dist_scale;
  /** @brief Scaled distance threshold for assignments.*/
  int m_max_mahalanobis_distance;
};

};

#endif //__MULTI_HYPOTHESIS_TRACKER_H__