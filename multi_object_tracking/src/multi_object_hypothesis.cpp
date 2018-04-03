#include <multi_object_tracking/multi_object_hypothesis.h>

namespace MultiObjectTracker {

const MultiHypothesisTracker::TrackerParameters& MultiObjectHypothesis::getParameters()
{
	static MultiHypothesisTracker::TrackerParameters params = {
		0.0075,	// cov_x_per_sec
		0.0075,	// cov_y_per_sec
		0.0075,	// cov_z_per_sec

		0.01,	// cov_vx_per_sec (independent of |vx|)
		0.01,	// cov_vy_per_sec (independent of |vy|)
		0.01,	// cov_vz_per_sec (independent of |vz|)

		0,	// alpha_vx_vx_per_sec
		0,	// alpha_vx_vy_per_sec
		0,	// alpha_vy_vy_per_sec
		0,	// alpha_vz_vz_per_sec

		// 0.06*0.06, // init_cov
		 0.02*0.02, // init_cov
		0.10, // max_cov

		0.2*0.2, // measurementStd

// 		sqrt(2.204) // ambiguous_dist
		13.3*sqrt(2.204)
	};
	return params;
}

MultiObjectHypothesis::MultiObjectHypothesis()  {
}

MultiObjectHypothesis::~MultiObjectHypothesis() {
}

}
