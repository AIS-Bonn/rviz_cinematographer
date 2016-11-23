/* -*- mode: C++ -*-
 *
 *  Copyright (C) 2011, 2012 Austin Robot Technology
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id: data_base.h 1554 2011-06-14 22:11:17Z jack.oquin $
 */

/** \file
 *
 *  Point Cloud Library point structures for Velodyne data.
 *
 *  @author Jesse Vera
 *  @author Jack O'Quin
 *  @author Piyush Khandelwal
 */

#ifndef __VELODYNE_POINTCLOUD_POINT_TYPES_H
#define __VELODYNE_POINTCLOUD_POINT_TYPES_H

#include <pcl/point_types.h>

namespace velodyne_pointcloud
{
  /** Euclidean Velodyne coordinate, including intensity and ring number. */
  struct PointXYZIR
  {
    PCL_ADD_POINT4D;                    // quad-word XYZ
    float    intensity;                 ///< laser intensity reading
    uint16_t ring;                      ///< laser ring number
    /*float average;                      ///< laser ring number*/
    float true_distance;                      ///< laser ring number
    float difference;                      ///< laser ring number
    float expected_dist;                      ///< laser ring number
    float obstacle;                      ///< laser ring number
    uint32_t order;                      ///< laser ring number
    int cluster_id; 
    PCL_ADD_RGB
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // ensure proper alignment
  } EIGEN_ALIGN16;

}; // namespace velodyne_pointcloud


POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_pointcloud::PointXYZIR,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (uint16_t, ring, ring)
				 /* (float, average, average)*/
				  (float, true_distance, true_distance)
				  (float, difference, difference)
				  (float, expected_dist, expected_dist)
				  (float, obstacle, obstacle)
				  (uint32_t, order , order)
				  (int, cluster_id, cluster_id)
				  (uint32_t, rgb, rgb)
)

#endif // __VELODYNE_POINTCLOUD_POINT_TYPES_H

