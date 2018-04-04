#ifndef __MULTI_OBJECT_HYPOTHESIS_H__
#define __MULTI_OBJECT_HYPOTHESIS_H__

#include <multi_object_tracking/hypothesis_3D.h>

namespace MultiHypothesisTracker
{

class MultiObjectHypothesis : public Hypothesis3D {
public:
  MultiObjectHypothesis();
  virtual ~MultiObjectHypothesis();

  virtual const TrackerParameters& getParameters();

  inline Eigen::Vector3d getSize(){ return m_size; }
  inline void setSize(const Eigen::Vector3d& size){ m_size = size; }

  inline Eigen::Matrix3d getPrincipalComponents(){ return m_principalComponents; }
  inline void setPrincipalComponents(const Eigen::Matrix3d& principalComponents){ m_principalComponents = principalComponents; }

  inline std::string getSource(){ return m_source; }
  inline void setSource(const std::string& source){ m_source = source; }

protected:

  Eigen::Vector3d m_size;
  Eigen::Matrix3d m_principalComponents;
  std::string m_source;

};

class MultiObjectHypothesisFactory : public Hypothesis3DFactory
{
public:
  MultiObjectHypothesisFactory(){}
  virtual ~MultiObjectHypothesisFactory(){}

  virtual std::shared_ptr<Hypothesis> createHypothesis(){ return std::make_shared<MultiObjectHypothesis>(); }
};

}

#endif
