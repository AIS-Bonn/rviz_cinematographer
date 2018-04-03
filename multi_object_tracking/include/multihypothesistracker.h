#ifndef __MULTIHYPOTHESISTRACKER_H__
#define __MULTIHYPOTHESISTRACKER_H__

#include <vnl/vnl_matrix.h>
#include <vnl/vnl_vector.h>

#include <vector>
#include <string>
#include <algorithm>
#include <chrono>

// #include <QMutex>
// #include <QColor>

// JS: a generic multi hypothesis tracker
// of course, the measurement and state transition models have to be implemented for the specific task
// this class just implements the most basic models: 3D position, noisy velocity, and direct state measurement
// control input: robot pose difference to last prediction step (dx, dy, dtheta)

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

class Hypothesis;
class HypothesisFactory;

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


double get_time_high_res ();

class MultiHypothesisTracker {
public:
  MultiHypothesisTracker( HypothesisFactory* hypothesisFactory );
  ~MultiHypothesisTracker();

  inline unsigned int getNumStateDimensions() { return m_numStateDimensions; }

  /**
   * @brief Calls predict for each hypothesis.
   *
   * @param time_diff   time difference between last and current prediction
   * @param control     for state transition model //TODO: better description
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

class HypothesisFactory {
public:
  HypothesisFactory() {}
  ~HypothesisFactory() {}

  virtual Hypothesis* createHypothesis();
};

class Hypothesis {
public:
  Hypothesis();
  ~Hypothesis();

  virtual const TrackerParameters& getParameters();

  inline unsigned int getID() { return m_ID; }

  inline std::string getLabel() { return m_label; }
  inline void setLabel( const std::string& label ) { m_label = label; }

// 		inline QColor getColor() { return m_color; }
// 		inline void setColor( const QColor& color ) { m_color = color; }

  inline unsigned int getNumStateDimensions() { return m_numStateDimensions; }

  inline vnl_vector< double >& getMean() { return m_mean; }
  inline vnl_matrix< double >& getCovariance() { return m_covariance; }
  inline uint8_t getColor() { return m_color; }
  inline bool isStatic() {return m_is_static; }

  virtual void initialize( const Measurement& measurement, unsigned int id, const std::string& label = ""/*, const QColor& color = QColor( 0, 0, 0 )*/ );

  virtual bool isSpurious();

  inline void setVisible( bool v ) { m_visible = v; }
  inline bool isVisible() { return m_visible; }

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


class Hypothesis3DFactory : public HypothesisFactory {
public:
  Hypothesis3DFactory() {}
  ~Hypothesis3DFactory() {}

  virtual Hypothesis* createHypothesis();
};

class Hypothesis3D : public Hypothesis {
public:
  Hypothesis3D();
  ~Hypothesis3D();

  virtual const TrackerParameters& getParameters();

  virtual void initialize( const Measurement& measurement, unsigned int id, const std::string& label = ""/*, const QColor& color = QColor( 0, 0, 0 )*/ );
  vnl_vector<double> velocity_decay(vnl_vector<double>);


  // EKF
  virtual void stateTransitionModel( vnl_vector< double >& predictedState, vnl_matrix< double >& stateTransitionMatrix, vnl_matrix< double >& stateTransitionCovariance, const vnl_vector< double >& currentState, double dt, const vnl_vector< double >& control );
  virtual void measurementModel( vnl_vector< double >& expectedMeasurement, vnl_matrix< double >& measurementMatrix, vnl_matrix< double >& measurementCovariance, const vnl_vector< double >& currentState );

protected:

};

};

#endif //__MULTIHYPOTHESISTRACKER_H__
