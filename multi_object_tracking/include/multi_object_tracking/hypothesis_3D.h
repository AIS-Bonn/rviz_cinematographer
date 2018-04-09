#ifndef __HYPOTHESIS_3D_H__
#define __HYPOTHESIS_3D_H__

#include <multi_object_tracking/hypothesis.h>

namespace MultiHypothesisTracker
{

class Hypothesis3D : public Hypothesis
{
public:

  Hypothesis3D();
  virtual ~Hypothesis3D(){};

  /** @brief Returns the parameters of the tracker for 3D objects.*/
  virtual const TrackerParameters& getParameters();

  /**
   * @brief Initializes a Hypothesis.
   *
   * @param[in] measurement detection.
   * @param[in] id          id.
   * @param[in] label       label.
   */
  virtual void initialize(const Measurement& measurement,
                          unsigned int id,
                          const std::string& label = "");

  /**
   * @brief Computes the velocity decay.
   *
   * The decay depends on the last time the hypothesis was assigned to a measurement.
   *
   * @param[in] velocity_in input velocity.
   * @return updated velocity.
   */
  Eigen::Vector3d velocity_decay(Eigen::Vector3d velocity_in);


  // EKF
  virtual void stateTransitionModel(Eigen::Vector3d& predictedState,
                                    Eigen::Matrix3d& stateTransitionMatrix,
                                    Eigen::Matrix3d& stateTransitionCovariance,
                                    const Eigen::Vector3d& currentState,
                                    double dt,
                                    const Eigen::Vector3d& control);

  virtual void measurementModel(Eigen::Vector3d& expectedMeasurement,
                                Eigen::Matrix3d& measurementMatrix,
                                Eigen::Matrix3d& measurementCovariance,
                                const Eigen::Vector3d& currentState);

protected:

};

class Hypothesis3DFactory : public HypothesisFactory
{
public:
  Hypothesis3DFactory(){}
  virtual ~Hypothesis3DFactory(){}

  virtual std::shared_ptr<Hypothesis> createHypothesis();
};

};

#endif //__HYPOTHESIS_3D_H__
