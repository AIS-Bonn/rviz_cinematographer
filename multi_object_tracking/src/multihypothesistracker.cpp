#include <multi_object_tracking/multihypothesistracker.h>

namespace MultiHypothesisTracker {

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

void MultiHypothesisTracker::predict(double time_diff,
                                     Eigen::Vector3d& control)
{
// 	lockHypotheses();

  // TODO: necessary? : check if hypotheses are changed in predict function and if this is useful
  std::vector<Hypothesis*> hypotheses = m_hypotheses;

  for(auto& hypothesis : hypotheses)
    hypothesis->predict(time_diff, control);

// 	unlockHypotheses();
}

void MultiHypothesisTracker::deleteSpuriosHypotheses()
{
  // delete spurious hypotheses
  std::vector< Hypothesis* >::iterator it = m_hypotheses.begin();
  while(it != m_hypotheses.end())
  {
    if((*it)->isVisible() && (*it)->isSpurious())
    {
      delete (*it);
      it = m_hypotheses.erase(it);
      continue;
    }
    ++it;
  }
}

	std::vector< unsigned int > MultiHypothesisTracker::correct_hungarian_simplified( const std::vector< Measurement >& measurements){
		// 		lockHypotheses();

		std::vector< unsigned int > assignments( measurements.size() , 0 );


		// only update visible hypotheses, but associate with all of them
		std::vector< Hypothesis* > hypotheses;
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
					Eigen::Matrix3d measurementMatrix;
					measurementMatrix.setIdentity();
          Eigen::Matrix3d correctionCovariance;
					correctionCovariance = measurementMatrix * hypotheses[i]->getCovariance() * measurementMatrix.transpose() + measurements[j].cov;

					Eigen::Vector3d diff_hyp_meas = measurements[j].pos - hypotheses[i]->getMean();

					auto mahalanobis_distance_squared = diff_hyp_meas.transpose()
													 * correctionCovariance.inverse()
													 * diff_hyp_meas;

					double mahalanobis_distance = sqrt(mahalanobis_distance_squared);

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
							hypotheses[i]->detected_absolute(); // Jan: this was added by radu
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


		deleteSpuriosHypotheses();


// 		unlockHypotheses();

		return assignments;


	}

	void MultiHypothesisTracker::mergeCloseHypotheses(double mergeDistance)
  {
		auto it1 = m_hypotheses.begin();
		while(it1 != m_hypotheses.end())
    {
			auto it2 = it1 + 1;
			while(it2 != m_hypotheses.end())
      {
				double dist = ((*it1)->getMean() - (*it2)->getMean()).norm();

        // TODO: delete color comparison if color not used
				if(dist < mergeDistance  && ((*it1)->getColor()==(*it2)->getColor()))
        {
					delete (*it2);
					it2 = m_hypotheses.erase(it2);
					continue;
				}
				++it2;
			}
			++it1;
		}
	}

};
