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

		void predictWithoutMeasurement();

		void objectDetectionDataReceived( std::vector< Measurement >& measurements, const std::string& sourceName);

		const std::vector< MultiHypothesisTracker::Hypothesis*>& getHypotheses();

/*		void lockHypotheses(){ m_multi_hypothesis_tracker.lockHypotheses(); }
		void unlockHypotheses() { m_multi_hypothesis_tracker.unlockHypotheses(); }*/

		inline MultiObjectHypothesis* getHypothesisByID( unsigned int ID ) { return (MultiObjectHypothesis*) m_multi_hypothesis_tracker.getHypothesisByID( ID ); }

		void clear() { m_multi_hypothesis_tracker.clear(); }

		inline void set_merge_close_hypotheses_distance (double val) {m_merge_close_hypotheses_distance= val;}

		MultiHypothesisTracker::MultiHypothesisTracker m_multi_hypothesis_tracker;

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

}

#endif
