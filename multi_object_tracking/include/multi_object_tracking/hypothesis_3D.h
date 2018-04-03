#ifndef __HYPOTHESIS_3D_H__
#define __HYPOTHESIS_3D_H__

#include <multi_object_tracking/hypothesis.h>

namespace MultiHypothesisTracker
{

class Hypothesis3D : public Hypothesis {
public:
  Hypothesis3D();
  virtual ~Hypothesis3D();

  virtual const TrackerParameters& getParameters();

  virtual void initialize( const Measurement& measurement, unsigned int id, const std::string& label = ""/*, const QColor& color = QColor( 0, 0, 0 )*/ );
  vnl_vector<double> velocity_decay(vnl_vector<double>);


  // EKF
  virtual void stateTransitionModel( vnl_vector< double >& predictedState, vnl_matrix< double >& stateTransitionMatrix, vnl_matrix< double >& stateTransitionCovariance, const vnl_vector< double >& currentState, double dt, const vnl_vector< double >& control );
  virtual void measurementModel( vnl_vector< double >& expectedMeasurement, vnl_matrix< double >& measurementMatrix, vnl_matrix< double >& measurementCovariance, const vnl_vector< double >& currentState );

protected:

};

class Hypothesis3DFactory : public HypothesisFactory
{
public:
  Hypothesis3DFactory(){}
  virtual ~Hypothesis3DFactory(){}

  virtual Hypothesis* createHypothesis();
};

};

#endif //__HYPOTHESIS_3D_H__
