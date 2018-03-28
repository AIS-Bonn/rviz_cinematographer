#include "multihypothesistracker.h"

#include "vnl/vnl_inverse.h"
#include "vnl/algo/vnl_svd.h"
#include "vnl/algo/vnl_symmetric_eigensystem.h"

#include "hungarian.h"
#include <limits.h> // for INT_MAX

#include <sys/time.h>
#include <iostream>
#include <map>

// #define DETECTION_RATE_INCREMENT 0.04f
#define DETECTION_RATE_INCREMENT 1.0f

namespace MultiHypothesisTracker {


	double get_time_high_res (){
		// std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
		// auto duration = now.time_since_epoch();
		// double time_high_res= std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();

		timeval currTime;
		gettimeofday( &currTime, NULL );
		double time_high_res = ((double)currTime.tv_sec) + ((double)currTime.tv_usec) * 1e-6;

		return time_high_res;
	}




	bool HypothesisFilter::passthrough( Hypothesis* hypothesis ) {
		return true;
	}

	std::vector< Hypothesis* > HypothesisFilter::filter( const std::vector< Hypothesis* >& hypotheses ) {
		// std::cout << "HypothesisFilter::filter input size " << hypotheses.size() << '\n';
		return hypotheses;
	}


	MultiHypothesisTracker::MultiHypothesisTracker( HypothesisFactory* hypothesisFactory = new HypothesisFactory() )
	:	m_lastHypothesisID( 1 )
	,	m_numStateDimensions( 6 )
	,	m_hypothesisFactory( hypothesisFactory ) {
	}

	MultiHypothesisTracker::~MultiHypothesisTracker() {
// 		lockHypotheses();
		for( unsigned int i = 0; i < m_hypotheses.size(); i++ )
			delete m_hypotheses[i];
		m_hypotheses.clear();
		delete m_hypothesisFactory;
// 		unlockHypotheses();
	}

	Hypothesis* MultiHypothesisTracker::getHypothesisByID( unsigned int ID ) {
		for( unsigned int i = 0; i < m_hypotheses.size(); i++ )
			if( m_hypotheses[i]->getID() == ID )
				return m_hypotheses[i];
		return NULL;
	}

	void MultiHypothesisTracker::clear() {
// 		lockHypotheses();
		for( unsigned int i = 0; i < m_hypotheses.size(); i++ )
			delete m_hypotheses[i];
		m_hypotheses.clear();
// 		unlockHypotheses();
	}

	void MultiHypothesisTracker::predict( double dt, const vnl_vector< double >& control, HypothesisFilter* filter ) {
// 		lockHypotheses();

		std::vector< Hypothesis* > hypotheses = filter->filter( m_hypotheses );
		// std::cout << "predict::m_hypothesis size is: " << m_hypotheses.size()  << '\n';
		// std::cout << "predict::hypothesis size is: " << hypotheses.size()  << '\n';
		for( unsigned int i = 0; i < hypotheses.size(); i++ )
			hypotheses[i]->predict( dt, control );
// 		unlockHypotheses();

	}

	void MultiHypothesisTracker::deleteSpuriosHypotheses( HypothesisFilter* filter )
	{
		// delete spurious hypotheses
		std::vector< Hypothesis* >::iterator it = m_hypotheses.begin();
		while( it != m_hypotheses.end() ) {
			if( (*it)->isVisible() && (!filter || filter->passthrough( *it )) && (*it)->isSpurious() ) {
				delete (*it);
				it = m_hypotheses.erase( it );
				continue;
			}
			++it;
		}
	}

	std::vector< unsigned int > MultiHypothesisTracker::correct( const std::vector< vnl_vector< double > >& measurements, HypothesisFilter* filter ) {

// // 		lockHypotheses();
//
// 		std::vector< unsigned int > assignments( measurements.size() );
// 		for( unsigned int i = 0; i < assignments.size(); i++ )
// 			assignments[i] = 0;
//
// 		// only update visible hypotheses, but associate with all of them
// 		std::vector< Hypothesis* > hypotheses;
// 		if(filter)
// 			hypotheses = filter->filter( m_hypotheses );
// 		else
// 			hypotheses = m_hypotheses;
//
// 		const int COST_FACTOR = 10000;
// 		const double MAX_MAHALANOBIS_DISTANCE = sqrt( 2.204 );
// 		int jobs = hypotheses.size();
// 		int machines = measurements.size();
//
// 		// Use Hungarian Method for assignment
// 		int dim = machines+jobs;
// 		hungarian_problem_t hung;
// 		int **cost_matrix = new int*[dim];
// 		for (int i=0; i < dim; i++ ) {
// 			cost_matrix[i] = new int[dim];
//
// 			vnl_matrix< double > measurementMatrix, measurementCovariance, kalmanGain;
// 			vnl_vector< double > expectedMeasurement;
// 			if( i < jobs )
// 				hypotheses[i]->measurementModel( expectedMeasurement, measurementMatrix, measurementCovariance, hypotheses[i]->getMean() );
//
// 			vnl_matrix< double > correctionCovariance;
// 			vnl_matrix< double > invCorrectionCovariance;
// 			if( i < jobs ) {
// 				correctionCovariance = measurementMatrix * hypotheses[i]->getCovariance() * measurementMatrix.transpose() + measurementCovariance;
// 				vnl_svd< double > svdCorrectionCovariance( correctionCovariance );
// 				invCorrectionCovariance= svdCorrectionCovariance.pinverse();
// 			}
//
// 			for (int j=0; j < dim; j++ ) {
//
// 				if (i<jobs && j<machines) {
// 					// an observation with a corresponding hypothesis
//
// 					double mahalanobis_distance = sqrt( dot_product< double >( (measurements[j] - expectedMeasurement) * invCorrectionCovariance, (measurements[j] - expectedMeasurement) ) );
// 					if( mahalanobis_distance < MAX_MAHALANOBIS_DISTANCE )
// 						cost_matrix[i][j] = (int) (COST_FACTOR*mahalanobis_distance);
// 					else
// 						cost_matrix[i][j] = INT_MAX;
// 				}
// 				else if(i<jobs && j>=machines)  {
// 					// cost for a hypothesis with no corresponding observation
// 					cost_matrix[i][j] = (int) (COST_FACTOR*MAX_MAHALANOBIS_DISTANCE);
// 				}
// 				else if (i>=jobs && j<machines)  {
// 					// cost for an observation with no corresponding hypothesis
// 					cost_matrix[i][j] = (int) (COST_FACTOR*MAX_MAHALANOBIS_DISTANCE);
// 				}
// 				else if (i>=jobs && j>=machines)  {
// 					// cost for a dummy job to a dummy machine
// 					cost_matrix[i][j] = 0;
// 				}
// 			}
//
// 		}
//
// 		hungarian_init(&hung, cost_matrix, dim, dim, HUNGARIAN_MODE_MINIMIZE_COST);
//
// // 		hungarian_print_costmatrix(&hung);
// 		hungarian_solve(&hung);
// // 		hungarian_print_assignment(&hung);
//
// 		for (int i=0; i < dim; i++ ) {
// 			for (int j=0; j < dim; j++ ) {
//
// 				bool associated = false;
// 				if( i<jobs && j<machines ) {
// 					if( hung.assignment[i][j] == HUNGARIAN_ASSIGNED && hung.cost[i][j] < COST_FACTOR*MAX_MAHALANOBIS_DISTANCE )
// 						associated = true;
// 				}
// 				else
// 					associated = ( hung.assignment[i][j] != HUNGARIAN_ASSIGNED );
//
// 				if( i<jobs && j<machines ) {
// 					if( associated ) {
//
// //						if( hypotheses[i]->isVisible() ) {
// 							hypotheses[i]->correct( measurements[j] );
// 							assignments[j] = hypotheses[i]->getID();
// 							hypotheses[i]->detected();
// //						}
//
// 					}
// 					else if( hung.assignment[i][j] == HUNGARIAN_ASSIGNED ) {
// 						// hungarian method assigned with INT_MAX => observation and track unassigned
//
// 						// discount hypothesis detection rate
// 						if( hypotheses[i]->isVisible() ) {
// 							hypotheses[i]->undetected();
// 						}
//
// 						// create new hypothesis for observation
// 						Hypothesis* hypothesis = m_hypothesisFactory->createHypothesis();
// 						hypothesis->initialize( measurements[j], m_lastHypothesisID++ );
// 						m_hypotheses.push_back( hypothesis );
// 						assignments[j] = hypothesis->getID();
// 					}
// 				}
// 				else if (i<jobs && j>=machines)  {
// 					// a hypothesis with no corresponding observation
// 					if (!associated) {
// 						if( hypotheses[i]->isVisible() ) {
// 							hypotheses[i]->undetected();
// 						}
// 					}
// 				}
// 				else if (i>=jobs && j<machines)  {
// 					// an observation with no corresponding hypothesis -> add
// 					if (!associated) {
// 						Hypothesis* hypothesis = m_hypothesisFactory->createHypothesis();
// 						hypothesis->initialize( measurements[j], m_lastHypothesisID++ );
// 						m_hypotheses.push_back( hypothesis );
// 						assignments[j] = hypothesis->getID();
// 					}
// 				}
// 				else if (i>=jobs && j>=machines)  {
// 					// a dummy job to a dummy machine
// 				}
// 			}
// 		}
//
// 		for (int i=0; i<dim; i++)
// 			delete[] cost_matrix[i];
// 		delete[] cost_matrix;
// 		hungarian_free(&hung);
//
// 		deleteSpuriosHypotheses( filter );
//
// // 		unlockHypotheses();
//
// 		return assignments;
	}

	std::vector< unsigned int > MultiHypothesisTracker::correctAmbiguous( const std::vector< Measurement >& measurements, bool createNewHypotheses, HypothesisFilter* filter ) {

// 		typedef std::map<unsigned int, std::vector<vnl_vector<double> > > AssignmentMap;
// 		struct HypothesisInfo
// 		{
// 			vnl_vector< double > expectedMeasurement;
// 			vnl_matrix< double > invCorrectionCovariance;
// 		};
//
// // 		lockHypotheses();
//
// 		std::vector< unsigned int > assignments( measurements.size() );
// 		std::vector< Hypothesis* > hypotheses;
// 		if(filter)
// 			hypotheses = filter->filter( m_hypotheses );
// 		else
// 			hypotheses = m_hypotheses;
//
// 		AssignmentMap assignmentMap;
// 		HypothesisInfo *infoCache = new HypothesisInfo[hypotheses.size()];
// 		assert(infoCache);
//
// 		// Pre-calculate hypothesis measurement model informations
// 		for(unsigned int i = 0; i < hypotheses.size(); ++i)
// 		{
// 			Hypothesis *hypothesis = hypotheses[i];
// 			HypothesisInfo info;
//
// 			vnl_matrix< double > measurementMatrix, measurementCovariance, kalmanGain;
//
// 			hypothesis->measurementModel( info.expectedMeasurement, measurementMatrix, measurementCovariance, hypothesis->getMean() );
//
// 			vnl_matrix< double > correctionCovariance;
//
// 			correctionCovariance = measurementMatrix * hypothesis->getCovariance() * measurementMatrix.transpose() + measurementCovariance;
// 			vnl_svd< double > svdCorrectionCovariance( correctionCovariance );
// 			info.invCorrectionCovariance= svdCorrectionCovariance.pinverse();
//
// 			infoCache[i] = info;
// 		}
//
//
// 		unsigned int startNumHypotheses = hypotheses.size();
// 		for(unsigned int j = 0; j < measurements.size(); ++j)
// 		{
// 			double min_dist = -1;
// 			Hypothesis *min_dist_hypothesis = 0;
// 			unsigned int min_dist_hypothesis_num;
//
// 			for(unsigned int i = 0; i < startNumHypotheses; ++i)
// 			{
// 				Hypothesis *hypothesis = hypotheses[i];
// 				const HypothesisInfo &info = infoCache[i];
//
// 				double mahalanobis_distance = sqrt( dot_product< double >(
// 					(measurements[j] - info.expectedMeasurement) * info.invCorrectionCovariance, (measurements[j] - info.expectedMeasurement) ) );
//
// 				if(mahalanobis_distance < min_dist || !min_dist_hypothesis)
// 				{
// 					std::cout << "entered the if" << '\n';
// 					min_dist = mahalanobis_distance;
// 					min_dist_hypothesis = hypothesis;
// 					min_dist_hypothesis_num = i;
// 				}
// 			}
//
// 			// TODO: move parameter to tracker
// 			if(min_dist_hypothesis && min_dist < min_dist_hypothesis->getParameters().ambiguous_dist)
// 			{
// 				// std::cout << "correct ambigous: assign to existing hypothesis " << '\n';
// 				// assign to existing hypothesis
// 				assignments[j] = min_dist_hypothesis->getID();
//
// 				assignmentMap[min_dist_hypothesis_num].push_back(measurements[j]);
// 			}
// 			else if(createNewHypotheses)
// 			{
// 				// create new hypothesis
// 				// std::cout << "correct ambigous: creating new hypothesis" << '\n';
// 				Hypothesis* hypothesis = m_hypothesisFactory->createHypothesis();
// 				hypothesis->initialize( measurements[j], m_lastHypothesisID++ );
// 				m_hypotheses.push_back( hypothesis );
// 				assignments[j] = hypothesis->getID();
// 			}
// 			else // Creation of new hypotheses disabled
// 			{
// 				assignments[j] = INT_MAX;
// 			}
// 		}
//
// 		// Process new assignments
//
// 		for(unsigned int j = 0; j < hypotheses.size(); ++j)
// 		{
// 			Hypothesis *hypothesis = hypotheses[j];
//
// //			if( !hypothesis->isVisible() )
// //				continue;
//
//
// 			if(assignmentMap.count(j) > 0)
// 			{
// 				std::vector<vnl_vector<double> >& hypothesisMeasurements = assignmentMap[j];
// 				vnl_vector<double> mean(3);
// 				mean.fill(0);
//
// 				for(unsigned int i = 0; i < hypothesisMeasurements.size(); ++i)
// 				{
// 					mean += hypothesisMeasurements[i];
// 				}
//
// 				mean /= hypothesisMeasurements.size();
//
// 				hypothesis->correct(mean);
// 				hypothesis->detected();
// 			}
// 			else
// 			{
// 				if( hypothesis->isVisible() )
// 					hypothesis->undetected();
// 			}
// 		}
//
// 		deleteSpuriosHypotheses( filter );
//
// 		// std::cout << "correct ambious:: after deleting the spurious ones we have hypothesis size " << m_hypotheses.size() << '\n';
//
// // 		unlockHypotheses();
//
// 		delete[] infoCache;
//
// 		return assignments;
	}


	std::vector< unsigned int > MultiHypothesisTracker::correctAmbiguous_simplified( const std::vector< Measurement >& measurements, bool createNewHypotheses, HypothesisFilter* filter ) {


		// vnl_vector< double > expectedMeasurement;
		vnl_matrix< double > invCorrectionCovariance;


		std::vector< unsigned int > assignments( measurements.size() );
		std::vector< Hypothesis* > hypotheses;
		if(filter)
			hypotheses = filter->filter( m_hypotheses );
		else
			hypotheses = m_hypotheses;

		std::map<unsigned int, std::vector<Measurement > >  assignmentMap;


		// Pre-calculate hypothesis measurement model informations
		// for(unsigned int i = 0; i < hypotheses.size(); ++i)
		// {
		// 	Hypothesis *hypothesis = hypotheses[i];
		// 	HypothesisInfo info;
		//
		// 	vnl_matrix< double > measurementMatrix, measurementCovariance, kalmanGain;
		//
		// 	hypothesis->measurementModel( info.expectedMeasurement, measurementMatrix, measurementCovariance, hypothesis->getMean() );
		//
		// 	vnl_matrix< double > correctionCovariance;
		//
		// 	correctionCovariance = measurementMatrix * hypothesis->getCovariance() * measurementMatrix.transpose() + measurementCovariance;
		// 	vnl_svd< double > svdCorrectionCovariance( correctionCovariance );
		// 	info.invCorrectionCovariance= svdCorrectionCovariance.pinverse();
		//
		// 	infoCache[i] = info;
		// }


		unsigned int startNumHypotheses = hypotheses.size();
		for(unsigned int j = 0; j < measurements.size(); ++j)
		{
			double min_dist = -1;
			Hypothesis *min_dist_hypothesis = 0;
			unsigned int min_dist_hypothesis_num;

			for(unsigned int i = 0; i < startNumHypotheses; ++i)
			{
				Hypothesis *hypothesis = hypotheses[i];

				//Calculate the inverse correction covariance
				vnl_matrix< double > measurementMatrix = vnl_matrix< double >( 3, 3 );
				measurementMatrix.set_identity();
				vnl_matrix< double > correctionCovariance;
				correctionCovariance = measurementMatrix * hypothesis->getCovariance() * measurementMatrix.transpose() + measurements[j].cov;
				vnl_svd< double > svdCorrectionCovariance( correctionCovariance );
				vnl_matrix< double > invCorrectionCovariance= svdCorrectionCovariance.pinverse();


				double mahalanobis_distance = sqrt( dot_product< double >(
					(measurements[j].pos - hypothesis->getMean()) * invCorrectionCovariance, (measurements[j].pos -  hypothesis->getMean()) ) );

				//finds the hypothesis which is closes to the mesurement
				if( (mahalanobis_distance < min_dist || !min_dist_hypothesis) &&
						(measurements[j].color==hypothesis->getColor() || measurements[j].color=='U' || hypothesis->getColor()=='U') )
				{
					min_dist = mahalanobis_distance;
					min_dist_hypothesis = hypothesis;
					min_dist_hypothesis_num = i;
				}
			}

			// TODO: move parameter to tracker
			if(min_dist_hypothesis && min_dist < min_dist_hypothesis->getParameters().ambiguous_dist)
			{
				// std::cout << "correct ambigous: assign to existing hypothesis " << '\n';
				// std::cout << "correct ambigous: assigning measurement " << measurements[j].color << " to " << min_dist_hypothesis->getColor() << '\n';
				// assign to existing hypothesis
				assignments[j] = min_dist_hypothesis->getID();

				assignmentMap[min_dist_hypothesis_num].push_back(measurements[j]);
			}
			else if(createNewHypotheses)
			{
				// create new hypothesis
				// std::cout << "correct ambigous: creating new hypothesis" << '\n';
				Hypothesis* hypothesis = m_hypothesisFactory->createHypothesis();
				hypothesis->initialize( measurements[j], m_lastHypothesisID++ );
				m_hypotheses.push_back( hypothesis );
				assignments[j] = hypothesis->getID();
			}
			else // Creation of new hypotheses disabled
			{
				assignments[j] = INT_MAX;
			}
		}

		// Process new assignments

		for(unsigned int j = 0; j < hypotheses.size(); ++j){
			Hypothesis *hypothesis = hypotheses[j];

//			if( !hypothesis->isVisible() )
//				continue;


			if(assignmentMap.count(j) > 0) {
				std::vector<Measurement >& hypothesisMeasurements = assignmentMap[j];
				//make a mean measurement, mean will in this case be an average mean based on the covariance matrices
				Measurement mean_mes;
				mean_mes.pos=vnl_vector<double>(3);
				mean_mes.pos.fill(0);
				mean_mes.cov=vnl_matrix< double >( 3, 3 );
				mean_mes.cov.set_identity();

				mean_mes.color=compute_most_prominent_color(hypothesisMeasurements);


				double sum_inv_cov=0;
				double sum_cov=0;
				for(unsigned int i = 0; i < hypothesisMeasurements.size(); ++i)
				{
					mean_mes.pos += hypothesisMeasurements[i].pos * (1/hypothesisMeasurements[i].cov(0,0));
					sum_inv_cov += (1/hypothesisMeasurements[i].cov(0,0));
					sum_cov+=hypothesisMeasurements[i].cov(0,0);
				}

				mean_mes.pos /= sum_inv_cov;
				mean_mes.cov (0,0) = sum_cov/hypothesisMeasurements.size();
				mean_mes.cov (1,1) = sum_cov/hypothesisMeasurements.size();
				mean_mes.cov (2,2) = sum_cov/hypothesisMeasurements.size();



				hypothesis->correct(mean_mes);
				hypothesis->detected();
			}
			else
			{
				if( hypothesis->isVisible() )
					hypothesis->undetected();
			}
		}

		deleteSpuriosHypotheses( filter );

		// std::cout << "correct ambious:: after deleting the spurious ones we have hypothesis size " << m_hypotheses.size() << '\n';

// 		unlockHypotheses();



		return assignments;
	}

	std::vector< unsigned int > MultiHypothesisTracker::correct_hungarian_simplified( const std::vector< Measurement >& measurements, HypothesisFilter* filter ){
		// 		lockHypotheses();

		std::vector< unsigned int > assignments( measurements.size() , 0 );


		// only update visible hypotheses, but associate with all of them
		std::vector< Hypothesis* > hypotheses;
		if(filter)
			hypotheses = filter->filter( m_hypotheses );
		else
			hypotheses = m_hypotheses;

		const int COST_FACTOR = 10000;
		// const double MAX_MAHALANOBIS_DISTANCE = sqrt( 2.204 );
		const double MAX_MAHALANOBIS_DISTANCE = m_max_mahalanobis_distance;
		int jobs = hypotheses.size();
		int machines = measurements.size();

		// Use Hungarian Method for assignment
		int dim = machines+jobs;
		hungarian_problem_t hung;
		int **cost_matrix = new int*[dim];
		for (int i=0; i < dim; i++ ) {
			cost_matrix[i] = new int[dim];

			// vnl_matrix< double > measurementMatrix, measurementCovariance, kalmanGain;
			// vnl_vector< double > expectedMeasurement;
			// if( i < jobs )
			// 	hypotheses[i]->measurementModel( expectedMeasurement, measurementMatrix, measurementCovariance, hypotheses[i]->getMean() );
			//
			// vnl_matrix< double > correctionCovariance;
			// vnl_matrix< double > invCorrectionCovariance;
			// if( i < jobs ) {
			// 	correctionCovariance = measurementMatrix * hypotheses[i]->getCovariance() * measurementMatrix.transpose() + measurementCovariance;
			// 	vnl_svd< double > svdCorrectionCovariance( correctionCovariance );
			// 	invCorrectionCovariance= svdCorrectionCovariance.pinverse();
			// }

			for (int j=0; j < dim; j++ ) {

				if (i<jobs && j<machines) {
					// an observation with a corresponding hypothesis


					//Calculate the inverse correction covariance
					vnl_matrix< double > measurementMatrix = vnl_matrix< double >( 3, 3 );
					measurementMatrix.set_identity();
					vnl_matrix< double > correctionCovariance;
					correctionCovariance = measurementMatrix * hypotheses[i]->getCovariance() * measurementMatrix.transpose() + measurements[j].cov;
					vnl_svd< double > svdCorrectionCovariance( correctionCovariance );
					vnl_matrix< double > invCorrectionCovariance= svdCorrectionCovariance.pinverse();
					double mahalanobis_distance = sqrt( dot_product< double >(
						(measurements[j].pos - hypotheses[i]->getMean()) * invCorrectionCovariance, (measurements[j].pos -  hypotheses[i]->getMean()) ) );


					if( mahalanobis_distance < MAX_MAHALANOBIS_DISTANCE  &&
							((measurements[j].color==hypotheses[i]->getColor() || measurements[j].color=='U' || hypotheses[i]->getColor()=='U')) ){
						cost_matrix[i][j] = (int) (COST_FACTOR*mahalanobis_distance);
					}
					else{
						cost_matrix[i][j] = INT_MAX;
					}
				}
				else if(i<jobs && j>=machines)  {
					// cost for a hypothesis with no corresponding observation
					cost_matrix[i][j] = (int) (COST_FACTOR*MAX_MAHALANOBIS_DISTANCE);
				}
				else if (i>=jobs && j<machines)  {
					// cost for an observation with no corresponding hypothesis
					cost_matrix[i][j] = (int) (COST_FACTOR*MAX_MAHALANOBIS_DISTANCE);
				}
				else if (i>=jobs && j>=machines)  {
					// cost for a dummy job to a dummy machine
					cost_matrix[i][j] = 0;
				}
			}

		}

		hungarian_init(&hung, cost_matrix, dim, dim, HUNGARIAN_MODE_MINIMIZE_COST);

// 		hungarian_print_costmatrix(&hung);
		hungarian_solve(&hung);
// 		hungarian_print_assignment(&hung);

		for (int i=0; i < dim; i++ ) {
			for (int j=0; j < dim; j++ ) {

				bool associated = false;
				if( i<jobs && j<machines ) {
					if( hung.assignment[i][j] == HUNGARIAN_ASSIGNED && hung.cost[i][j] < COST_FACTOR*MAX_MAHALANOBIS_DISTANCE )
						associated = true;
				}
				else
					associated = ( hung.assignment[i][j] != HUNGARIAN_ASSIGNED );

				if( i<jobs && j<machines ) {
					if( associated ) {

//						if( hypotheses[i]->isVisible() ) {
							hypotheses[i]->correct( measurements[j] );
							assignments[j] = hypotheses[i]->getID();
							hypotheses[i]->detected();
							hypotheses[i]->detected_absolute();
//						}

					}
					else if( hung.assignment[i][j] == HUNGARIAN_ASSIGNED ) {
						// hungarian method assigned with INT_MAX => observation and track unassigned

						// discount hypothesis detection rate
						if( hypotheses[i]->isVisible() ) {
							hypotheses[i]->undetected();
						}

						// create new hypothesis for observation
						Hypothesis* hypothesis = m_hypothesisFactory->createHypothesis();
						hypothesis->initialize( measurements[j], m_lastHypothesisID++ );
						m_hypotheses.push_back( hypothesis );
						assignments[j] = hypothesis->getID();
					}
				}
				else if (i<jobs && j>=machines)  {
					// a hypothesis with no corresponding observation
					if (!associated) {
						if( hypotheses[i]->isVisible() ) {
							hypotheses[i]->undetected();
						}
					}
				}
				else if (i>=jobs && j<machines)  {
					// an observation with no corresponding hypothesis -> add
					if (!associated) {
						Hypothesis* hypothesis = m_hypothesisFactory->createHypothesis();
						hypothesis->initialize( measurements[j], m_lastHypothesisID++ );
						m_hypotheses.push_back( hypothesis );
						assignments[j] = hypothesis->getID();
					}
				}
				else if (i>=jobs && j>=machines)  {
					// a dummy job to a dummy machine
				}
			}
		}

		for (int i=0; i<dim; i++)
			delete[] cost_matrix[i];
		delete[] cost_matrix;
		hungarian_free(&hung);

		deleteSpuriosHypotheses( filter );


// 		unlockHypotheses();

		return assignments;


	}

	uint8_t MultiHypothesisTracker::compute_most_prominent_color(std::vector< Measurement >& measurements){
		// int num_r, num_b, num_g, num_y, num_o, num_u;
		std::vector<int> colors(5,0); //5 colors, all initialized to 0
		// num_r=num_b=num_g=num_y=num_o=num_u=0;
		for (size_t i = 0; i < measurements.size(); i++) {
			if (measurements[i].color=='R')	colors[0]++;
			if (measurements[i].color=='B')	colors[1]++;
			if (measurements[i].color=='G')	colors[2]++;
			if (measurements[i].color=='Y')	colors[3]++;
			if (measurements[i].color=='O')	colors[4]++;
			// if (hypothesisMeasurements.color='u')	colors[0]++;
		}
		auto biggest = std::max_element(std::begin(colors), std::end(colors)); //biggest element
		if ((*biggest)==0){
			return 'U';
		}
		int pos= std::distance(std::begin(colors), biggest); //position of the biggest element
		if (pos==0)	return 'R';
		if (pos==1)	return 'B';
		if (pos==2)	return 'G';
		if (pos==3)	return 'Y';
		if (pos==4)	return 'O';

	}


	void MultiHypothesisTracker::mergeCloseHypotheses( float mergeDistance ) {

		std::vector< Hypothesis* >::iterator it1 = m_hypotheses.begin();
		while( it1 != m_hypotheses.end() ) {

			std::vector< Hypothesis* >::iterator it2 = it1 + 1;
			while( it2 != m_hypotheses.end() ) {

				float dist = ((*it1)->getMean() - (*it2)->getMean()).two_norm();

				// std::cout << "mergeClose dist is " << dist << '\n';

				if( dist < mergeDistance  && ((*it1)->getColor()==(*it2)->getColor())  ) {
					delete (*it2);
					it2 = m_hypotheses.erase( it2 );
					continue;
				}

				++it2;
			}

			++it1;

		}

	}

	Hypothesis* HypothesisFactory::createHypothesis() {
		return new Hypothesis();
	}


	Hypothesis::Hypothesis()
	:	m_mean( vnl_vector< double >( 6 ) )
	,	m_covariance( vnl_matrix< double >( 6, 6 ) )
	,	m_numStateDimensions( 6 ) {
		m_visible = true;
		m_lastMeasurementTime=0;
		m_last_mean_with_measurement=	vnl_vector< double >(3);
		m_last_mean_with_measurement.fill(0);
		m_is_first_position=true;
		m_velocity = vnl_vector< double >(3);
		m_velocity.fill(0);
		m_max_velocity_in_track= vnl_vector<double> (3);
		m_max_velocity_in_track.fill(0);
		m_is_picked=false;
		m_born_time=0;
		m_times_measured=0;
	}

	Hypothesis::~Hypothesis() {
	}

	const TrackerParameters& Hypothesis::getParameters()
	{
		static TrackerParameters params = {
			0.001,	// cov_x_per_sec
			0.001,	// cov_y_per_sec
			0.001,	// cov_z_per_sec

			0.001,	// cov_vx_per_sec (independent of |vx|)
			0.001,	// cov_vy_per_sec (independent of |vy|)
			0.001,	// cov_vz_per_sec (independent of |vz|)

			0,	// alpha_vx_vx_per_sec
			0,	// alpha_vx_vy_per_sec
			0,	// alpha_vy_vy_per_sec
			0,	// alpha_vz_vz_per_sec

			0.05*0.05, // init_cov
			0.05*0.05, // max_cov

			0.1, // measurementStd

			0.4, // ambiguous_dist

			};
		return params;
	}

	void Hypothesis::initialize( const Measurement& measurement, unsigned int id, const std::string& label/*, const QColor& color*/ ) {
		// m_mean.fill( 0 );
		// m_mean( 0 ) = measurement( 0 );
		// m_mean( 1 ) = measurement( 1 );
		// m_mean( 2 ) = measurement( 2 );
		//
		// m_covariance.set_identity();
		// m_covariance *= getParameters().init_cov;
		//
		// timeval currTime;
		// gettimeofday( &currTime, NULL );
		// m_lastMeasurementTime = ((double)currTime.tv_sec) + ((double)currTime.tv_usec) * 1e-6;
		//
		// m_detectionRate = 0.5f;
		// m_misdetectionRate = 0.5f;
		//
		// m_ID = id;
		//
		// if( label.length() > 0 )
		// 	m_label = label;
		// else {
		// 	std::stringstream ss;
		// 	ss << id;
		// 	ss >> m_label;
		// }

// 		m_color = color;
	}

	void Hypothesis::detected() {
		m_detectionRate += DETECTION_RATE_INCREMENT;
		float sumDetectionRate = m_detectionRate + m_misdetectionRate;
		m_detectionRate /= sumDetectionRate;
		m_misdetectionRate /= sumDetectionRate;
	}

	void Hypothesis::undetected() {
		m_misdetectionRate += DETECTION_RATE_INCREMENT;
		float sumDetectionRate = m_detectionRate + m_misdetectionRate;
		m_detectionRate /= sumDetectionRate;
		m_misdetectionRate /= sumDetectionRate;
	}

	bool Hypothesis::isSpurious() {

		double currentTime = get_time_high_res();

		//static object that went more than 2 seconds with less than 3 detections
		//theme may just be missdetections of the laser that gives really briefly a measurement for an object
		// if (m_is_static && (currentTime - m_born_time > 1.0) && m_times_measured <4  ){
		// 	return true;
		// }

		if ((currentTime - m_born_time > 0.65) && m_times_measured <=1  ){
			return true;
		}


		if (m_is_static){
			return false;
		}



		if( currentTime - m_lastMeasurementTime > 90 ){ //more than x second and the track will be deleted
			return true;
		}else {
			double maxPositionCov = getParameters().max_cov;

			vnl_symmetric_eigensystem< double > eigensystemPosition( m_covariance.extract( 3, 3 ) );
			// if( eigensystemPosition.get_eigenvalue( 0 ) > maxPositionCov || eigensystemPosition.get_eigenvalue( 1 ) > maxPositionCov || eigensystemPosition.get_eigenvalue( 2 ) > maxPositionCov ) {
			// 	return true;
			// }

			// if (!m_is_static){
			// 	std::cout << "------------------------" << '\n';
			// 	std::cout << "eigensystm: " << eigensystemPosition.get_eigenvalue( 0 ) << '\n';
			// }

			return false;
		}
	}

	void Hypothesis::verify_static(){

		//In the case that it is a movable object it will forever be flagged as movable, ie, non-static
		// std::cout << "verfy_static: dist is " << (m_mean-m_first_position_in_track).two_norm() << " and flag is " << m_is_static << '\n';
		// std::cout << "verfy_static: max_velocity is  is " << m_max_velocity_in_track.two_norm() << " and flag is " << m_is_static << '\n';

		if (m_is_static){
			if (m_color=='R' || m_color=='B' || m_color=='G' || m_color== 'O') {   //If it's red, green, blue, orange we know it's static
				m_is_static=true;
				return;
			}else if (m_color=='Y' ){						//If it's yellow we know it's gonna move
				m_is_static=false;
				return;
			}else{																							//If it's of unknown color we have to check how much it moved
				if (  (m_mean-m_first_position_in_track).two_norm() > 0.25  ){
					m_is_static=false;
					return;
				}
			}
		}


		// if (m_is_static){
		// 	if (  (m_mean-m_first_position_in_track).two_norm() > 0.4   &&  (m_max_velocity_in_track.two_norm() > 0.85)  ){
		// 		m_is_static=false;
		// 	}
		// }

	}

	void Hypothesis::predict( double dt, const vnl_vector< double >& control ) {
		// control is neglected
		vnl_matrix< double > stateTransitionMatrix, stateTransitionCovariance;

		// std::cout << "Hypothesis:predict dt is " << dt << '\n';
		// std::cout << "Hypothesis:predict state before prediction is " << m_mean << '\n';

		stateTransitionModel( m_mean, stateTransitionMatrix, stateTransitionCovariance, m_mean, dt, control );

		// std::cout << "Hypothesis:predict state after prediction is " << m_mean << '\n';

		verify_static();  //Verifies that the object is static or not


		if( !m_is_static){

			//If covariance is bigger than the maximum cov than do not update it anymore
			double maxPositionCov = getParameters().max_cov;
			vnl_symmetric_eigensystem< double > eigensystemPosition( m_covariance.extract( 3, 3 ) );
			if( eigensystemPosition.get_eigenvalue( 0 ) > maxPositionCov || eigensystemPosition.get_eigenvalue( 1 ) > maxPositionCov || eigensystemPosition.get_eigenvalue( 2 ) > maxPositionCov ) {
				//don't update
			}else{
				//update
				m_covariance = stateTransitionMatrix * m_covariance * stateTransitionMatrix.transpose() + stateTransitionCovariance;
			}


		}

		for( int i = 0; i < m_covariance.rows(); i++ )
			for( int j = i; j < m_covariance.cols(); j++ )
				m_covariance( i, j ) = m_covariance( j, i );


	}

	void Hypothesis::correct( const Measurement& measurement ) {
		vnl_matrix< double > measurementMatrix, measurementCovariance, kalmanGain;
		vnl_vector< double > expectedMeasurement;
		measurementModel( expectedMeasurement, measurementMatrix, measurementCovariance, m_mean );

		vnl_matrix< double > correctionCovariance = measurementMatrix * m_covariance * measurementMatrix.transpose() + measurement.cov;
		vnl_svd< double > svdCorrectionCovariance( correctionCovariance );
		vnl_matrix< double > invCorrectionCovariance = svdCorrectionCovariance.pinverse();

		kalmanGain = m_covariance * measurementMatrix.transpose() * invCorrectionCovariance;
		vnl_matrix< double > identity = m_covariance;
		identity.set_identity();


		double curr_time = get_time_high_res();
		// double time_dif= curr_time-m_last_mean_time;
		// double time_dif= curr_time-m_lastMeasurementTime;  //Should be something like time_dif=measurement.time - previous_meaurement.time
		double time_dif= measurement.time-m_previous_measurement.time;



		// double time_dif= curr_time-m_last_prediction_time;
		// std::cout << "time dif is " << time_dif << '\n';
		// std::cout << "cur time: " << curr_time << '\n';
		// std::cout << "m_last_mean time: " << m_last_mean_time << '\n';
		// bool invalidate_velocity=false;
		// if (m_lastMeasurementTime<=0){     //This si the case when this is the first correction therfore there is not previous measurement
		// 	invalidate_velocity=true;
		// }


		m_mean = m_mean + kalmanGain * ( measurement.pos - expectedMeasurement );
		if (!m_is_static){  //TODO Not sure it's completely right
			m_covariance = ( identity - kalmanGain * measurementMatrix ) * m_covariance;
		}


		// m_last_mean_time = curr_time;

		// m_velocity=(m_mean-m_last_mean_with_measurement)/time_dif; m_velocity(2)=0;  //TODO cap velocity a certain max


		//Running average
		vnl_vector<double> new_velocity = vnl_vector<double>(3);
		new_velocity=(m_mean-m_last_mean_with_measurement)/time_dif; new_velocity(2)=0;
		double alpha=0.80;
		if (m_velocity(0)==0 && m_velocity(1)==0 && m_velocity(2)==0){  //If it's the first velocity just integrate it so we don0t have this retency to movement
			m_velocity=(m_mean-m_last_mean_with_measurement)/time_dif; m_velocity(2)=0;
		}else{
			m_velocity= m_velocity + alpha*(new_velocity-m_velocity);
		}


		//CAP
		// std::cout << "velocity is " << m_velocity.magnitude() << '\n';
		double max_velocity=1.4;   //1.4ms or 5kmh
		double slack=1;
		if (m_velocity.magnitude() > max_velocity + slack ){
			m_velocity=m_velocity.normalize();
			// std::cout << "over-----" << '\n';
			m_velocity*=(max_velocity+slack);
		}
		// std::cout << "velocity is " << m_velocity.magnitude() << '\n';
		m_velocity(2)=0;


		//OBJECT ARE WAY TOO SLOW JUT MAKE THE VELOCITY 0
		m_velocity.fill(0);




		//TODO Correct velocity depending on the angle between the base link and the base_footprint
		//get_drone_





		// std::cout << "---------velocity norm is ------------"<< m_velocity.two_norm() << '\n';
		if (m_velocity.two_norm() > m_max_velocity_in_track.two_norm()){
			m_max_velocity_in_track=m_velocity;
		}


		// std::cout << "----------------------------------------" << '\n';
		// std::cout << "correct of hypothesis " << this->getID() << '\n';
		// std::cout << "m_mean is " << m_mean << '\n';
		// std::cout << "m_last_mean_with_measurement is " << m_last_mean_with_measurement << '\n';
		// std::cout << "time diff is " << time_dif << '\n';
		// std::cout << "m_lastMeasurementTime is " << m_lastMeasurementTime << '\n';
		// std::cout << "velocity is " << m_velocity << '\n';


		if (m_is_first_position){		//If this was the first position then the velocity which requiers a previous state is not valid
			m_velocity(0)=0;
			m_velocity(1)=0;
			m_velocity(2)=0;
		}
		// if (velocity.two_norm()>30){
		// 	velocity=velocity.normalize();
		// 	velocity=velocity*0.2;
		// }
		// if (velocity.two_norm()<0.15){
		// 	velocity(0)=0;
		// 	velocity(1)=0;
		// }



		m_last_mean_with_measurement=m_mean;
		m_is_first_position=false;
		m_previous_measurement=measurement;
		m_latest_measurement=measurement;
		m_lastMeasurementTime = get_time_high_res();



		for( int i = 0; i < m_covariance.rows(); i++ )
			for( int j = i; j < m_covariance.cols(); j++ )
				m_covariance( i, j ) = m_covariance( j, i );


	}

	void Hypothesis::stateTransitionModel( vnl_vector< double >& predictedState, vnl_matrix< double >& stateTransitionMatrix, vnl_matrix< double >& stateTransitionCovariance, const vnl_vector< double >& currentState, double dt, const vnl_vector< double >& control ) {

		stateTransitionMatrix = vnl_matrix< double >( 6, 6 );
		stateTransitionMatrix.set_identity();
		stateTransitionMatrix( 0, 3 ) = dt;
		stateTransitionMatrix( 1, 4 ) = dt;
		stateTransitionMatrix( 2, 5 ) = dt;

//		std::cout << dt << "\n";

		stateTransitionCovariance = vnl_matrix< double >( 6, 6 );
		stateTransitionCovariance.set_identity();

		const TrackerParameters &params = getParameters();

		stateTransitionCovariance( 0, 0 ) = dt * params.cov_x_per_sec;
		stateTransitionCovariance( 1, 1 ) = dt * params.cov_y_per_sec;
		stateTransitionCovariance( 2, 2 ) = dt * params.cov_z_per_sec;
		stateTransitionCovariance( 3, 3 ) = dt * ( params.cov_vx_per_sec + params.alpha_vx_vx_per_sec * params.alpha_vx_vx_per_sec * currentState( 3 ) * currentState( 3 ) + params.alpha_vx_vy_per_sec * params.alpha_vx_vy_per_sec * currentState( 4 ) * currentState( 4 ) + params.alpha_vx_vy_per_sec * params.alpha_vx_vy_per_sec * currentState( 5 ) * currentState( 5 ) );
		stateTransitionCovariance( 4, 4 ) = dt * ( params.cov_vy_per_sec + params.alpha_vy_vy_per_sec * params.alpha_vy_vy_per_sec * currentState( 4 ) * currentState( 4 ) + params.alpha_vx_vy_per_sec * params.alpha_vx_vy_per_sec * currentState( 3 ) * currentState( 3 ) + params.alpha_vx_vy_per_sec * params.alpha_vx_vy_per_sec * currentState( 5 ) * currentState( 5 ) );
		stateTransitionCovariance( 5, 5 ) = dt * ( params.cov_vz_per_sec + params.alpha_vz_vz_per_sec * params.alpha_vz_vz_per_sec * currentState( 5 ) * currentState( 5 ) + params.alpha_vx_vy_per_sec * params.alpha_vx_vy_per_sec * currentState( 3 ) * currentState( 3 ) + params.alpha_vx_vy_per_sec * params.alpha_vx_vy_per_sec * currentState( 4 ) * currentState( 4 ) );

		predictedState = stateTransitionMatrix * currentState;





		// add control influence
		// control: robot pose difference to last prediction step
		predictedState(0) = predictedState(0) - control(0);
		predictedState(1) = predictedState(1) - control(1);

		vnl_matrix< double > R_control( 2, 2 );
		R_control( 0, 0 ) = cos( -control( 2 ) );
		R_control( 0, 1 ) = -sin( -control( 2 ) );
		R_control( 1, 0 ) = sin( -control( 2 ) );
		R_control( 1, 1 ) = cos( -control( 2 ) );

		vnl_vector< double > predictedState2D( 2 );
		predictedState2D = R_control * predictedState.extract( 2 );
		predictedState(0) = predictedState2D(0);
		predictedState(1) = predictedState2D(1);


	}

	void Hypothesis::measurementModel( vnl_vector< double >& expectedMeasurement, vnl_matrix< double >& measurementMatrix, vnl_matrix< double >& measurementCovariance, const vnl_vector< double >& currentState ) {

		measurementMatrix = vnl_matrix< double >( 3, 6 );
		measurementMatrix.fill( 0 );
		measurementMatrix( 0, 0 ) = 1;
		measurementMatrix( 1, 1 ) = 1;
		measurementMatrix( 2, 2 ) = 1;

		expectedMeasurement = vnl_vector< double >( 3 );
		expectedMeasurement = measurementMatrix * currentState;

		measurementCovariance = vnl_matrix< double >( 3, 3 );
		measurementCovariance.set_identity();
		double measurementStd = getParameters().measurementStd;
		measurementCovariance( 0, 0 ) = measurementStd * measurementStd;
		measurementCovariance( 1, 1 ) = measurementStd * measurementStd;
		measurementCovariance( 2, 2 ) = measurementStd * measurementStd;

	}



	Hypothesis* Hypothesis3DFactory::createHypothesis() {
		return new Hypothesis3D();
	}


	Hypothesis3D::Hypothesis3D() {
		m_mean = vnl_vector< double >( 3 );
		m_covariance = vnl_matrix< double >( 3, 3 );
		m_numStateDimensions = 3;
		m_color='U';
		m_is_static=true;
		// m_last_prediction_time=-1;
	}

	Hypothesis3D::~Hypothesis3D() {
	}

	const TrackerParameters& Hypothesis3D::getParameters()
	{
		static TrackerParameters params = {
			0.001,	// cov_x_per_sec
			0.001,	// cov_y_per_sec
			0.001,	// cov_z_per_sec

			0.0,	// cov_vx_per_sec (dont care for 3D)
			0.0,	// cov_vy_per_sec (dont care for 3D)
			0.0,	// cov_vz_per_sec (dont care for 3D)

			0,	// alpha_vx_vx_per_sec
			0,	// alpha_vx_vy_per_sec
			0,	// alpha_vy_vy_per_sec
			0,	// alpha_vz_vz_per_sec

			0.05*0.05, // init_cov
			0.05*0.05, // max_cov

			0.1, // measurementStd

			0.4, // ambiguous_dist

			};
		return params;
	}

	void Hypothesis3D::initialize( const Measurement& measurement, unsigned int id, const std::string& label/*, const QColor& color*/ ) {
		m_mean( 0 ) = measurement.pos( 0 );
		m_mean( 1 ) = measurement.pos( 1 );
		m_mean( 2 ) = measurement.pos( 2 );

		m_color=measurement.color;
		m_first_position_in_track = m_mean;
		m_born_time=get_time_high_res();
		verify_static();  //Verifies that the object is static or not
		m_times_measured++;


		m_covariance.set_identity();
		// m_covariance *= getParameters().init_cov;
		m_covariance=measurement.cov;

		// std::cout << "initialize hypothesis with color " << measurement.color << '\n';
		// std::cout << "init_cov is " << getParameters().init_cov << '\n';
		// std::cout << "measuremetnt cov is " << measurement.cov << '\n';


		m_lastMeasurementTime = get_time_high_res();

		m_detectionRate = 0.5f;
		m_misdetectionRate = 0.5f;

		m_ID = id;

		if( label.length() > 0 )
			m_label = label;
		else {
			std::stringstream ss;
			ss << id;
			ss >> m_label;
		}

// 		m_color = color;
	}

	vnl_vector<double> Hypothesis3D::velocity_decay(vnl_vector<double> velocity_in ){
		double cur_time = get_time_high_res();
		//Velocity stays the same up until 1 second then it decays
		double time_start_decay=1.0;
		double time_finish_decay=4.0;
		double dif= time_finish_decay-time_start_decay;

		if (cur_time - m_lastMeasurementTime > time_start_decay){
			double weight = std::max (0.0, (time_finish_decay - (cur_time - m_lastMeasurementTime)) /dif ) ;
			return velocity_in*weight;
		}else{
			return velocity_in;
		}

	}


	void Hypothesis3D::stateTransitionModel( vnl_vector< double >& predictedState, vnl_matrix< double >& stateTransitionMatrix, vnl_matrix< double >& stateTransitionCovariance, const vnl_vector< double >& currentState, double dt, const vnl_vector< double >& control ) {

		stateTransitionMatrix = vnl_matrix< double >( 3, 3 );
		stateTransitionMatrix.set_identity();

//		std::cout << dt << "\n";


		stateTransitionCovariance = vnl_matrix< double >( 3, 3 );
		stateTransitionCovariance.set_identity();

		const TrackerParameters &params = getParameters();


		stateTransitionCovariance( 0, 0 ) = dt * params.cov_x_per_sec;
		stateTransitionCovariance( 1, 1 ) = dt * params.cov_y_per_sec;
		stateTransitionCovariance( 2, 2 ) = dt * params.cov_z_per_sec;





		predictedState = stateTransitionMatrix * currentState;

		//pseudo-velocity
		// std::cout << "stateTransitionModel:before velocity pred: " << predictedState << '\n';
		// std::cout << "stateTransitionModel:before velocity cur: " << currentState << '\n';
		// std::cout << "state transitin: velocity without time is " << velocity << '\n';
		// std::cout << "state transitin: time diff is " << dt << '\n';
		// std::cout << "state transitin: velocity with time is " << velocity*dt << '\n';
		if (!m_is_first_position && !m_is_static){  //If it's hte first position in the hypothesis then we don't have a velocity cuz we don't have a prev measurement
			vnl_vector<double> velocity_decayed= velocity_decay(m_velocity);
			// vnl_vector<double> velocity_decayed = m_velocity;
			predictedState +=  velocity_decayed*(dt);

			// m_velocity=m_velocity*0.97; // TODO Not a very good way of doing decay becuase it depend on how often you do prediction
		}

		// std::cout << "stateTransitionModel:after velocity pred: " << predictedState << '\n';
		// std::cout << "stateTransitionModel:after velocity cur: " << currentState << '\n';
		// m_last_mean=predictedState;


		// add control influence
		// control: robot pose difference to last prediction step
		predictedState(0) = predictedState(0) - control(0);
		predictedState(1) = predictedState(1) - control(1);

		vnl_matrix< double > R_control( 2, 2 );
		R_control( 0, 0 ) = cos( -control( 2 ) );
		R_control( 0, 1 ) = -sin( -control( 2 ) );
		R_control( 1, 0 ) = sin( -control( 2 ) );
		R_control( 1, 1 ) = cos( -control( 2 ) );

		vnl_vector< double > predictedState2D( 2 );
		predictedState2D = R_control * predictedState.extract( 2 );
		predictedState(0) = predictedState2D(0);
		predictedState(1) = predictedState2D(1);


	}

	void Hypothesis3D::measurementModel( vnl_vector< double >& expectedMeasurement, vnl_matrix< double >& measurementMatrix, vnl_matrix< double >& measurementCovariance, const vnl_vector< double >& currentState ) {

		measurementMatrix = vnl_matrix< double >( 3, 3 );
		measurementMatrix.set_identity();

		expectedMeasurement = vnl_vector< double >( 3 );
		expectedMeasurement = measurementMatrix * currentState;

		measurementCovariance = vnl_matrix< double >( 3, 3 );
		measurementCovariance.set_identity();
		double measurementStd = getParameters().measurementStd;
		measurementCovariance( 0, 0 ) = measurementStd * measurementStd;
		measurementCovariance( 1, 1 ) = measurementStd * measurementStd;
		measurementCovariance( 2, 2 ) = measurementStd * measurementStd;

	}

};
