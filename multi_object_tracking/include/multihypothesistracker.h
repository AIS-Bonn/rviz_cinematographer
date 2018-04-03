#ifndef __MULTIHYPOTHESISTRACKER_H__
#define __MULTIHYPOTHESISTRACKER_H__

#include <vnl/vnl_matrix.h>
#include <vnl/vnl_vector.h>

#include <vector>
#include <string>
#include <algorithm>
#include <chrono>

#include <multi_object_hypothesis.h>

// #include <QMutex>
// #include <QColor>

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
                       const vnl_vector< double >& control);

  // returns vector of assignments
  virtual std::vector< unsigned int > correct( const std::vector< vnl_vector< double > >& measurements);
  virtual std::vector< unsigned int > correctAmbiguous( const std::vector< Measurement >& measurements, bool createNewHypotheses = true);

  virtual std::vector< unsigned int > correctAmbiguous_simplified( const std::vector< Measurement >& measurements, bool createNewHypotheses = true);
  std::vector< unsigned int > correct_hungarian_simplified( const std::vector< Measurement >& measurements);
  uint8_t compute_most_prominent_color(std::vector< Measurement >& measurements);

  void mergeCloseHypotheses( float mergeDistance );

  inline std::vector< Hypothesis* >& getHypotheses() { return m_hypotheses; }
  Hypothesis* getHypothesisByID( unsigned int ID );
// 		inline void lockHypotheses() { m_hypothesisMutex.lock(); }
// 		inline void unlockHypotheses() { m_hypothesisMutex.unlock(); }
  void clear();

  /**
   * @brief Deletes hypotheses that are visible and spurious
   *
   * @see isVisible()
   * @see isSpurious()
   */
  void deleteSpuriosHypotheses();


  //params
  inline void set_max_mahalanobis_distance(double val) {m_max_mahalanobis_distance=val;}
  inline void set_spurious_time(double val) {m_spurious_time=val;}
  inline void set_time_start_velocity_decay (double val) {m_time_start_velocity_decay=val;}
  inline void set_time_finish_velocity_decay (double val) {m_time_finish_velocity_decay=val;}

protected:


  std::vector< Hypothesis* > m_hypotheses;
  unsigned int m_lastHypothesisID;
  unsigned int m_numStateDimensions;
  HypothesisFactory* m_hypothesisFactory;

  //Parameters
  // double m_merge_close_hypotheses_distance;
  double m_max_mahalanobis_distance;
  double m_spurious_time;
  double m_time_start_velocity_decay;
  double m_time_finish_velocity_decay;


// 		QMutex m_hypothesisMutex;
};







};

#endif //__MULTIHYPOTHESISTRACKER_H__
