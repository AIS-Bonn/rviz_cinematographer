#include <multi_object_tracking/multihypothesistracker.h>

namespace MultiHypothesisTracker
{

MultiHypothesisTracker::MultiHypothesisTracker(std::shared_ptr<HypothesisFactory> hypothesis_factory = std::make_shared<HypothesisFactory>())
:	m_lastHypothesisID(1)
	,	m_numStateDimensions(6)
	,	m_hypothesisFactory(hypothesis_factory)
  , m_cost_factor(10000)
{
}

MultiHypothesisTracker::~MultiHypothesisTracker()
{
}

std::shared_ptr<Hypothesis> MultiHypothesisTracker::getHypothesisByID(unsigned int id)
{
	for(unsigned int i = 0; i < m_hypotheses.size(); i++)
		if(m_hypotheses[i]->getID() == id)
			return m_hypotheses[i];

	return nullptr;
}

void MultiHypothesisTracker::predict(double time_diff)
{
  for(auto& hypothesis : m_hypotheses)
    hypothesis->predict(time_diff);
}

void MultiHypothesisTracker::predict(double time_diff,
                                     Eigen::Vector3f& control)
{
  for(auto& hypothesis : m_hypotheses)
    hypothesis->predict(time_diff, control);
}

void MultiHypothesisTracker::deleteSpuriosHypotheses(double current_time)
{
  auto it = m_hypotheses.begin();
  while(it != m_hypotheses.end())
  {
    if((*it)->isSpurious(current_time))
    {
      it = m_hypotheses.erase(it);
      continue;
    }
    ++it;
  }
}

void MultiHypothesisTracker::correct(const std::vector<Measurement>& measurements)
{
  if(measurements.empty())
  {
    for(auto& hypothesis : m_hypotheses)
      hypothesis->undetected();

    return;
  }

  int **cost_matrix;
  setupCostMatrix(measurements, m_hypotheses, cost_matrix);

  hungarian_problem_t hung;
  size_t dim = measurements.size() + m_hypotheses.size();
  hungarian_init(&hung, cost_matrix, dim, dim, HUNGARIAN_MODE_MINIMIZE_COST);

// 		hungarian_print_costmatrix(&hung);
  hungarian_solve(&hung);
// 		hungarian_print_assignment(&hung);

  assign(hung, measurements, m_hypotheses);

  for(size_t i = 0; i < dim; i++)
    delete[] cost_matrix[i];
  delete[] cost_matrix;
  hungarian_free(&hung);

  deleteSpuriosHypotheses(measurements.at(0).time);
}

void MultiHypothesisTracker::setupCostMatrix(const std::vector<Measurement>& measurements,
                                             std::vector<std::shared_ptr<Hypothesis>>& hypotheses,
                                             int**& cost_matrix)
{
  size_t hyp_size = hypotheses.size();
  size_t meas_size = measurements.size();
  size_t dim = hyp_size + meas_size;

  cost_matrix = new int*[dim];

  for(size_t i=0; i < dim; i++)
  {
    cost_matrix[i] = new int[dim];

    // vnl_matrix< double > measurementMatrix, measurementCovariance, kalmanGain;
    // vnl_vector< double > expectedMeasurement;
    // if( i < hyp_size )
    // 	m_hypotheses[i]->measurementModel( expectedMeasurement, measurementMatrix, measurementCovariance, m_hypotheses[i]->getMean() );
    //
    // vnl_matrix< double > correctionCovariance;
    // vnl_matrix< double > invCorrectionCovariance;
    // if( i < hyp_size ) {
    // 	correctionCovariance = measurementMatrix * m_hypotheses[i]->getCovariance() * measurementMatrix.transpose() + measurementCovariance;
    // 	vnl_svd< double > svdCorrectionCovariance( correctionCovariance );
    // 	invCorrectionCovariance= svdCorrectionCovariance.pinverse();
    // }

    for(size_t j=0; j < dim; j++)
    {
      if(i < hyp_size && j < meas_size)
      {
        // an observation with a corresponding hypothesis

        //Calculate the inverse correction covariance
        Eigen::Matrix3f measurementMatrix;
        measurementMatrix.setIdentity();
        Eigen::Matrix3f correctionCovariance;
        correctionCovariance = measurementMatrix * m_hypotheses[i]->getCovariance() * measurementMatrix.transpose() + measurements[j].cov;

        Eigen::Vector3f diff_hyp_meas = measurements[j].pos - m_hypotheses[i]->getMean();

        auto mahalanobis_distance_squared = diff_hyp_meas.transpose()
                         * correctionCovariance.inverse()
                         * diff_hyp_meas;

        double mahalanobis_distance = sqrt(mahalanobis_distance_squared);

        if(mahalanobis_distance < m_max_mahalanobis_distance)
        {
          cost_matrix[i][j] = (int)(m_cost_factor * mahalanobis_distance);
        }
        else
        {
          cost_matrix[i][j] = INT_MAX;
        }
      }
      else if(i < hyp_size && j >= meas_size)
      {
        // cost for a hypothesis with no corresponding observation
        cost_matrix[i][j] = (int)(m_cost_factor * m_max_mahalanobis_distance);
      }
      else if(i >= hyp_size && j < meas_size)
      {
        // cost for an observation with no corresponding hypothesis
        cost_matrix[i][j] = (int)(m_cost_factor * m_max_mahalanobis_distance);
      }
      else if(i >= hyp_size && j >= meas_size)
      {
        // cost for a dummy job to a dummy machine
        cost_matrix[i][j] = 0;
      }
    }
  }
}

void MultiHypothesisTracker::assign(const hungarian_problem_t& hung,
                                    const std::vector<Measurement>& measurements,
                                    std::vector<std::shared_ptr<Hypothesis>>& hypotheses)
{
  size_t hyp_size = hypotheses.size();
  size_t meas_size = measurements.size();
  size_t dim = hyp_size + meas_size;

  for(size_t i = 0; i < dim; i++)
  {
    for(size_t j = 0; j < dim; j++)
    {
      bool associated = false;
      if(i < hyp_size && j < meas_size)
      {
        if(hung.assignment[i][j] == HUNGARIAN_ASSIGNED && hung.cost[i][j] < m_cost_factor * m_max_mahalanobis_distance)
          associated = true;
      }
      else
        associated = ( hung.assignment[i][j] != HUNGARIAN_ASSIGNED );

      if(i < hyp_size && j < meas_size)
      {
        if(associated)
        {
          m_hypotheses[i]->correct(measurements[j]);
          m_hypotheses[i]->detected();
          m_hypotheses[i]->detected_absolute(); // Jan: this was added by radu
        }
        else if(hung.assignment[i][j] == HUNGARIAN_ASSIGNED)
        {
          // hungarian method assigned with INT_MAX => observation and track unassigned

          // discount hypothesis detection rate
//          if( m_hypotheses[i]->isVisible() ) {
            m_hypotheses[i]->undetected();
//          }

          // create new hypothesis for observation
          std::shared_ptr<Hypothesis> hypothesis = m_hypothesisFactory->createHypothesis(measurements[j], m_lastHypothesisID++);
          m_hypotheses.push_back(hypothesis);
        }
      }
      else if(i < hyp_size && j >= meas_size)
      {
        // a hypothesis with no corresponding observation
        if(!associated)
          m_hypotheses[i]->undetected();
      }
      else if(i >= hyp_size && j < meas_size)
      {
        // an observation with no corresponding hypothesis -> add
        if(!associated)
        {
          std::cout << " creating hypothesis " << std::endl;
          std::shared_ptr<Hypothesis> hypothesis = m_hypothesisFactory->createHypothesis(measurements[j], m_lastHypothesisID++);
          m_hypotheses.push_back(hypothesis);
        }
      }
      else if(i >= hyp_size && j >= meas_size)
      {
        // a dummy job to a dummy machine
      }
    }
  }
}

void MultiHypothesisTracker::mergeCloseHypotheses(double distance_threshold)
{
	auto it1 = m_hypotheses.begin();
	while(it1 != m_hypotheses.end())
	{
		auto it2 = it1 + 1;
		while(it2 != m_hypotheses.end())
		{
			double distance = ((*it1)->getMean() - (*it2)->getMean()).norm();

			if(distance < distance_threshold)
			{
				it2 = m_hypotheses.erase(it2);
				continue;
			}
			++it2;
		}
		++it1;
	}
}

};
