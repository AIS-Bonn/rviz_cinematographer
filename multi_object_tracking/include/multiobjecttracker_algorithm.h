#ifndef __MULTI_OBJECT_TRACKER_ALGORITHM_H__
#define __MULTI_OBJECT_TRACKER_ALGORITHM_H__

#include <multihypothesistracker.h>

#include <vnl/vnl_vector_fixed.h>
#include <vnl/vnl_matrix_fixed.h>


#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>



namespace MultiObjectTracker {

	class MultiObjectHypothesis;

	class MultiObjectTrackerAlgorithm {
	public:

		MultiObjectTrackerAlgorithm();

		void updateFilterWithPrediction( MultiHypothesisTracker::HypothesisFilter* filter = new MultiHypothesisTracker::HypothesisFilter() );
		void updateFilter( double dx, double dy, double da, MultiHypothesisTracker::HypothesisFilter* filter = new MultiHypothesisTracker::HypothesisFilter() );
		void predictWithoutMeasurement();

		void objectDetectionDataReceived( std::vector< Measurement >& measurements, const std::string& sourceName);

		const std::vector< MultiHypothesisTracker::Hypothesis*>& getHypotheses();

/*		void lockHypotheses(){ m_multiHypothesisTracker.lockHypotheses(); }
		void unlockHypotheses() { m_multiHypothesisTracker.unlockHypotheses(); }*/

		inline MultiObjectHypothesis* getHypothesisByID( unsigned int ID ) { return (MultiObjectHypothesis*) m_multiHypothesisTracker.getHypothesisByID( ID ); }

		void clear() { m_multiHypothesisTracker.clear(); }

		inline void set_merge_close_hypotheses_distance (double val) {m_merge_close_hypotheses_distance= val;}

		MultiHypothesisTracker::MultiHypothesisTracker m_multiHypothesisTracker;

	private:


		// double m_lastMeasurementTime;
		double m_last_prediction_time;
		double m_merge_close_hypotheses_distance;
	};

	class MultiObjectHypothesis : public MultiHypothesisTracker::Hypothesis3D {
	public:
		MultiObjectHypothesis();
		~MultiObjectHypothesis();

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
		~MultiObjectHypothesisFactory() {}

		virtual MultiHypothesisTracker::Hypothesis* createHypothesis() { return new MultiObjectHypothesis(); }
	};

	class HypothesisFilterBySource : public MultiHypothesisTracker::HypothesisFilter {
	public:
		HypothesisFilterBySource( const std::string& source ) { m_source = source; }
		~HypothesisFilterBySource() {}

		bool passthrough( MultiHypothesisTracker::Hypothesis* hypothesis );
		std::vector< MultiHypothesisTracker::Hypothesis* > filter( const std::vector< MultiHypothesisTracker::Hypothesis* >& hypotheses );

	protected:
		std::string m_source;
	};



//	class HypothesisFilterBySourceAndFrustum : public MultiHypothesisTracker::HypothesisFilter {
//	public:
//		HypothesisFilterBySourceAndFrustum( const std::string& source, const Frustum& frustum ) { m_source = source; m_frustum = frustum; }
//		~HypothesisFilterBySourceAndFrustum() {}
//
//		bool passthrough( MultiHypothesisTracker::Hypothesis* hypothesis );
//		std::vector< MultiHypothesisTracker::Hypothesis* > filter( const std::vector< MultiHypothesisTracker::Hypothesis* >& hypotheses );
//
//	protected:
//		std::string m_source;
//		Frustum m_frustum;
//	};

}

#endif
