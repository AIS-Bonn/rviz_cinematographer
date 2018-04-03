#ifndef __HYPOTHESIS_3D_H__
#define __HYPOTHESIS_3D_H__

#include <vnl/vnl_matrix.h>
#include <vnl/vnl_vector.h>

#include <vector>
#include <string>
#include <algorithm>
#include <chrono>

#include <hypothesis.h>

// #include <QMutex>
// #include <QColor>

// JS: a generic multi hypothesis tracker
// of course, the measurement and state transition models have to be implemented for the specific task
// this class just implements the most basic models: 3D position, noisy velocity, and direct state measurement
// control input: robot pose difference to last prediction step (dx, dy, dtheta)




namespace MultiHypothesisTracker
{

class HypothesisFactory;

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

class Hypothesis3DFactory : public HypothesisFactory {
public:
  Hypothesis3DFactory() {}
  ~Hypothesis3DFactory() {}

  virtual Hypothesis* createHypothesis();
};

};

#endif //__HYPOTHESIS_3D_H__
