/** @file
 *
 * Helper functions for multi object tracker.
 *
 * @author Jan Razlaw
 */

#ifndef MULTI_OBJECT_TRACKING_UTILS_H
#define MULTI_OBJECT_TRACKING_UTILS_H

#include <sys/time.h>

namespace MultiHypothesisTracker
{

/**
 * @breif Returns current time of the day as a double value
 *
 * @return current time of the day
 */
inline double getTimeHighRes()
{
  timeval curr_time;
  gettimeofday(&curr_time, NULL);
  double time_high_res = ((double)curr_time.tv_sec) + ((double)curr_time.tv_usec) * 1e-6;

  return time_high_res;
}

}

#endif //MULTI_OBJECT_TRACKING_UTILS_H
