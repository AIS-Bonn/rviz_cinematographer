/** @file
 *
 * Multi hypothesis tracker implementation.
 *
 * @author Jan Razlaw
 */

#include <multi_object_tracking/multi_hypothesis_tracker.h>

namespace MultiHypothesisTracker
{

MultiHypothesisTracker::MultiHypothesisTracker(std::shared_ptr<HypothesisFactory> hypothesis_factory)
:	m_hypothesis_factory(hypothesis_factory)
 , m_current_hypothesis_id(1)
 , m_dist_scale(10000)
 , m_max_mahalanobis_distance((int)(m_dist_scale * 20.0))
{
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

  applyAssignments(hung.assignment, cost_matrix, measurements, m_hypotheses);

  for(size_t i = 0; i < dim; i++)
    delete[] cost_matrix[i];
  delete[] cost_matrix;
  hungarian_free(&hung);
}

double MultiHypothesisTracker::distance(const Eigen::Vector3f& hyp_position,
                                        const Eigen::Matrix3f& hyp_covariance,
                                        const Eigen::Vector3f& meas_position,
                                        const Eigen::Matrix3f& meas_covariance)
{
  // Calculate the inverse correction covariance
  Eigen::Matrix3f measurementMatrix;
  measurementMatrix.setIdentity();
  Eigen::Matrix3f correctionCovariance;
  correctionCovariance = measurementMatrix * hyp_covariance * measurementMatrix.transpose() + meas_covariance;

  Eigen::Vector3f diff_hyp_meas = meas_position - hyp_position;

  auto mahalanobis_distance_squared = diff_hyp_meas.transpose()
                                      * correctionCovariance.inverse()
                                      * diff_hyp_meas;

  return sqrt(mahalanobis_distance_squared);
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

    for(size_t j=0; j < dim; j++)
    {
      if(i < hyp_size && j < meas_size)
      {
        // Calculate distance between hypothesis and measurement
        double mahalanobis_distance = distance(m_hypotheses[i]->getPosition(),
                                               m_hypotheses[i]->getCovariance(),
                                               measurements[j].pos.block<3,1>(0, 0),
                                               measurements[j].cov.block<3,3>(0,0));

        int scaled_mahalanobis_distance = (int)(m_dist_scale * mahalanobis_distance);
        if(scaled_mahalanobis_distance < m_max_mahalanobis_distance)
        {
          cost_matrix[i][j] = scaled_mahalanobis_distance;
        }
        else
        {
          // if threshold exceeded, make sure assignment algorithm doesn't assign here
          cost_matrix[i][j] = INT_MAX;
        }
      }
      else if(i < hyp_size && j >= meas_size)
      {
        // distance from a hypothesis to a dummy measurement
        cost_matrix[i][j] = m_max_mahalanobis_distance;
      }
      else if(i >= hyp_size && j < meas_size)
      {
        // distance from a measurement to a dummy hypothesis
        cost_matrix[i][j] = m_max_mahalanobis_distance;
      }
      else if(i >= hyp_size && j >= meas_size)
      {
        // distance from a dummy hypothesis to a dummy measurement
        cost_matrix[i][j] = 0;
      }
    }
  }
}

void MultiHypothesisTracker::applyAssignments(int**& assignments,
                                              int**& cost_matrix,
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
      if(i < hyp_size && j < meas_size)
      {
        // if hypothesis assigned to measurement and distance below threshold -> correct hypothesis
        if(assignments[i][j] == HUNGARIAN_ASSIGNED && cost_matrix[i][j] < m_max_mahalanobis_distance)
        {
          m_hypotheses[i]->correct(measurements[j]);
          m_hypotheses[i]->detected();
        }
        else if(assignments[i][j] == HUNGARIAN_ASSIGNED)
        {
          // if assigned but distance too high -> prohibited assignment -> hypothesis undetected
          m_hypotheses[i]->undetected();

          // create new hypothesis from measurement
          m_hypotheses.emplace_back(m_hypothesis_factory->createHypothesis(measurements[j], m_current_hypothesis_id++));
        }
      }
      else if(i < hyp_size && j >= meas_size)
      {
        // if hypothesis assigned to dummy measurement -> failed to detect hypothesis
        if(assignments[i][j] == HUNGARIAN_ASSIGNED)
          m_hypotheses[i]->undetected();
      }
      else if(i >= hyp_size && j < meas_size)
      {
        // if measurement assigned to dummy hypothesis -> create new hypothesis
        if(assignments[i][j] == HUNGARIAN_ASSIGNED)
          m_hypotheses.emplace_back(m_hypothesis_factory->createHypothesis(measurements[j], m_current_hypothesis_id++));
      }
      else if(i >= hyp_size && j >= meas_size)
      {
        // dummy hypothesis to dummy measurement
      }
    }
  }
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

//TODO: implement a reasonable merging function.
void MultiHypothesisTracker::mergeCloseHypotheses(double distance_threshold)
{
	auto it1 = m_hypotheses.begin();
	while(it1 != m_hypotheses.end())
	{
		auto it2 = it1 + 1;
		while(it2 != m_hypotheses.end())
		{
			double distance = ((*it1)->getPosition() - (*it2)->getPosition()).norm();

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
