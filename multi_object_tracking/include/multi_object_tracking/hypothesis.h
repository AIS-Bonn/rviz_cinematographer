#ifndef __HYPOTHESIS_H__
#define __HYPOTHESIS_H__

// TODO: delete unused headers
#include <vnl/vnl_matrix.h>
#include <vnl/vnl_vector.h>

#include <vector>
#include <string>
#include <algorithm>
#include <chrono>

#include "vnl/vnl_inverse.h"
#include "vnl/algo/vnl_svd.h"
#include "vnl/algo/vnl_symmetric_eigensystem.h"

#include <multi_object_tracking/hungarian.h>
#include <limits.h> // for INT_MAX

#include <sys/time.h>
#include <iostream>
#include <map>

#include <multi_object_tracking/utils.h>

struct Measurement
{
  vnl_vector<double> pos;
  vnl_matrix<double> cov;
  uint8_t color;  // ascii code of the first letter of the color of the detected object ('r'ed, 'b'lue, 'g'reen, 'y'ellow, 'o'range, 'u'nknown)
  std::string frame;
  double time;  //time_stamp of the measurement
  vnl_vector<double> rotation_euler;  //rotation fo the drone with respect to to the horizontal axis
  double rotation_angle;
};


namespace MultiHypothesisTracker
{

struct TrackerParameters
{
  double cov_x_per_sec;
  double cov_y_per_sec;
  double cov_z_per_sec;
  double cov_vx_per_sec;
  double cov_vy_per_sec;
  double cov_vz_per_sec;
  double alpha_vx_vx_per_sec;
  double alpha_vx_vy_per_sec;
  double alpha_vy_vy_per_sec;
  double alpha_vz_vz_per_sec;

  double init_cov;
  double max_cov;

  double measurementStd;

  double ambiguous_dist;
};


class Hypothesis
{
public:

  Hypothesis();
  virtual ~Hypothesis();

  virtual const TrackerParameters& getParameters();

  inline unsigned int getID(){ return m_ID; }

  // TODO: delete? - used in tracker but is identical to ID
  inline std::string getLabel(){ return m_label; }
  inline void setLabel(const std::string& label){ m_label = label; }

  // TODO: delete? - never used, just set
  inline unsigned int getNumStateDimensions() { return m_numStateDimensions; }

  inline vnl_vector<double>& getMean(){ return m_mean; }
  inline vnl_matrix<double>& getCovariance(){ return m_covariance; }
  inline uint8_t getColor(){ return m_color; }
  inline bool isStatic(){ return m_is_static; }

  virtual void initialize(const Measurement& measurement, unsigned int id, const std::string& label = ""/*, const QColor& color = QColor( 0, 0, 0 )*/ );

  virtual bool isSpurious();

  inline void setVisible(bool v){ m_visible = v; }
  inline bool isVisible(){ return m_visible; }

  virtual void detected();
  virtual void undetected();

  inline float getDetectionRate() { return m_detectionRate; }
  inline float getMisdetectionRate() { return m_misdetectionRate; }

  // EKF
  virtual void predict( double dt, const vnl_vector< double >& control );
  virtual void correct( const Measurement& measurement );

  virtual void stateTransitionModel( vnl_vector< double >& predictedState, vnl_matrix< double >& stateTransitionMatrix, vnl_matrix< double >& stateTransitionCovariance, const vnl_vector< double >& currentState, double dt, const vnl_vector< double >& control );
  virtual void measurementModel( vnl_vector< double >& expectedMeasurement, vnl_matrix< double >& measurementMatrix, vnl_matrix< double >& measurementCovariance, const vnl_vector< double >& currentState );

  inline void set_picked(bool val){m_is_picked=val;}
  inline bool is_picked(){return m_is_picked;}
  inline double get_born_time(){return m_born_time;}
  inline void detected_absolute(){m_times_measured++;}  //total number of times that hypothesis had a measurement
  inline vnl_vector<double> get_velocity(){ return m_velocity;}
  inline Measurement get_latest_measurement(){ return m_latest_measurement;}
  inline double get_latest_measurement_time(){ return m_lastMeasurementTime;}

protected:
  vnl_vector< double > m_last_mean_with_measurement;
  // double m_last_prediction_time;
  bool m_is_first_position;
  vnl_vector< double > m_velocity;
  Measurement m_previous_measurement;
  Measurement m_latest_measurement;
  void velocity_decay();
  void verify_static();
  bool m_is_static;
  vnl_vector< double > m_first_position_in_track;
  vnl_vector<double> m_max_velocity_in_track;
  bool m_is_picked;
  double m_born_time;
  int m_times_measured;




  vnl_vector< double > m_mean;
  vnl_matrix< double > m_covariance;
  uint8_t m_color;
  double m_lastMeasurementTime;    //needed to calculate if it's spurious or not.
  float m_detectionRate;
  float m_misdetectionRate;
  bool m_visible;

  unsigned int m_ID;

  // for visualization
  std::string m_label;
// 		QColor m_color;

  unsigned int m_numStateDimensions;

};

class HypothesisFactory {
public:
  HypothesisFactory() {}
  virtual ~HypothesisFactory() {}

  virtual Hypothesis* createHypothesis();
};

};

#endif //__HYPOTHESIS_H__
