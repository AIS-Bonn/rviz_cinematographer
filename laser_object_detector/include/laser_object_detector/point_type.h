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

#define PCL_NO_PRECOMPILE

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>


#include <pcl/search/kdtree.h>
#include <pcl/search/impl/kdtree.hpp>

#include <pcl/search/flann_search.h>
#include <pcl/search/impl/flann_search.hpp>


#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/impl/extract_clusters.hpp>


namespace velodyne_pointcloud
{
   /** Euclidean Velodyne coordinate, including intensity, ring number and detection features. */
   struct PointXYZIRDetection
   {
      PCL_ADD_POINT4D;                    // quad-word XYZ
      float    intensity;                 // laser intensity reading
      uint16_t ring;                      // laser ring number
      float detection;                    // detection probability
      float detection_distance; 		      // debug info distance
      float detection_intensity;          // debug info intensity
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // ensure proper alignment
   } EIGEN_ALIGN16;

   /** Euclidean Velodyne coordinate, including intensity, distance and ring number. */
   struct PointXYZIDR
   {
      PCL_ADD_POINT4D;                    // quad-word XYZ
      float    intensity;                 ///< laser intensity reading
      float    distance;                  ///< distance of point to sensor
      uint16_t ring;                      ///< laser ring number
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // ensure proper alignment
   } EIGEN_ALIGN16;

   /** Euclidean Velodyne coordinate + object detection value. */
   struct PointXYZDetection
   {
      PCL_ADD_POINT4D;                    // quad-word XYZ
      float detection;                    // detection probability
      uint16_t ring;                      ///< laser ring number
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // ensure proper alignment
   } EIGEN_ALIGN16;
}; // namespace velodyne_pointcloud

POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_pointcloud::PointXYZIRDetection,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (uint16_t, ring, ring)
                                  (float, detection, detection)
                                  (float, detection_distance, detection_distance)
                                  (float, detection_intensity, detection_intensity)
)

POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_pointcloud::PointXYZIDR,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (float, distance, distance)
                                  (uint16_t, ring, ring)
)

POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_pointcloud::PointXYZDetection,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, detection, detection)
                                  (uint16_t, ring, ring)
)

#endif // __VELODYNE_POINTCLOUD_POINT_TYPES_H

