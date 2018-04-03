#ifndef __MULTI_OBJECT_HYPOTHESIS_H__
#define __MULTI_OBJECT_HYPOTHESIS_H__

#include <multi_object_tracking/hypothesis_3D.h>

namespace MultiObjectTracker
{

class MultiObjectHypothesis : public MultiHypothesisTracker::Hypothesis3D {
public:
  MultiObjectHypothesis();
  virtual ~MultiObjectHypothesis();

  virtual const MultiHypothesisTracker::TrackerParameters& getParameters();

  inline vnl_vector_fixed< float, 3 > getSize() { return m_size; }
  inline void setSize( const vnl_vector_fixed< float, 3 >& size ) { m_size = size; }

  inline vnl_matrix_fixed< float, 3, 3 > getPrincipalComponents() { return m_principalComponents; }
  inline void setPrincipalComponents( const vnl_matrix_fixed< float, 3, 3 >& principalComponents ) { m_principalComponents = principalComponents; }

  inline std::string getSource() { return m_source; }
  inline void setSource( const std::string& source ) { m_source = source; }

protected:

  vnl_vector_fixed< float, 3 > m_size;
  vnl_matrix_fixed< float, 3, 3 > m_principalComponents;
  std::string m_source;

};

class MultiObjectHypothesisFactory : public MultiHypothesisTracker::Hypothesis3DFactory {
public:
  MultiObjectHypothesisFactory() {}
  virtual ~MultiObjectHypothesisFactory() {}

  virtual MultiHypothesisTracker::Hypothesis* createHypothesis() { return new MultiObjectHypothesis(); }
};

}

#endif
