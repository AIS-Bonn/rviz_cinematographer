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
                                     const vnl_vector<double>& control)
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

	std::vector< unsigned int > MultiHypothesisTracker::correct( const std::vector< vnl_vector< double > >& measurements) {

// // 		lockHypotheses();
//
// 		std::vector< unsigned int > assignments( measurements.size() );
// 		for( unsigned int i = 0; i < assignments.size(); i++ )
// 			assignments[i] = 0;
//
// 		// only update visible hypotheses, but associate with all of them
// 		std::vector< Hypothesis* > hypotheses;
// 		hypotheses = m_hypotheses;
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
// 		deleteSpuriosHypotheses();
//
// // 		unlockHypotheses();
//
// 		return assignments;
	}

	std::vector< unsigned int > MultiHypothesisTracker::correctAmbiguous( const std::vector< Measurement >& measurements, bool createNewHypotheses) {

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
// 		hypotheses = m_hypotheses;
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
// 		deleteSpuriosHypotheses();
//
// 		// std::cout << "correct ambious:: after deleting the spurious ones we have hypothesis size " << m_hypotheses.size() << '\n';
//
// // 		unlockHypotheses();
//
// 		delete[] infoCache;
//
// 		return assignments;
	}


	std::vector< unsigned int > MultiHypothesisTracker::correctAmbiguous_simplified( const std::vector< Measurement >& measurements, bool createNewHypotheses) {


		// vnl_vector< double > expectedMeasurement;
		vnl_matrix< double > invCorrectionCovariance;


		std::vector< unsigned int > assignments( measurements.size() );
		std::vector< Hypothesis* > hypotheses;

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

		deleteSpuriosHypotheses();

		// std::cout << "correct ambious:: after deleting the spurious ones we have hypothesis size " << m_hypotheses.size() << '\n';

// 		unlockHypotheses();



		return assignments;
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

		deleteSpuriosHypotheses();


// 		unlockHypotheses();

		return assignments;


	}

// TODO: probably delete
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







};
