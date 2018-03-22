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
   /** Euclidean Velodyne coordinate, including intensity, ring number and segmentation features. */
   struct PointXYZIRSegmentation
   {
      PCL_ADD_POINT4D;                    ///< quad-word XYZ
      float intensity;                    ///< laser intensity reading
      uint16_t ring;                      ///< laser ring number
      float segmentation;                 ///< segmentation probability
      float segmentation_distance; 		    ///< debug info - distance delta
      float segmentation_intensity;       ///< debug info - intensity delta
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // ensure proper alignment
   } EIGEN_ALIGN16;

   /** Euclidean Velodyne coordinate, including intensity, distance and ring number. */
   struct PointXYZIDR
   {
      PCL_ADD_POINT4D;                    ///< quad-word XYZ
      float intensity;                    ///< laser intensity reading
      float distance;                     ///< distance of point to sensor
      uint16_t ring;                      ///< laser ring number
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // ensure proper alignment
   } EIGEN_ALIGN16;

   /** Euclidean Velodyne coordinate, including object segmentation flag and ring number. */
   struct PointXYZSegmentation
   {
      PCL_ADD_POINT4D;                    ///< quad-word XYZ
      uint8_t segmentation;               ///< segmentation flag
      uint16_t ring;                      ///< laser ring number
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // ensure proper alignment
   } EIGEN_ALIGN16;

   /** Euclidean Velodyne coordinate, including scan_line_id and scan_id for mrs_mapping and segmentation flag. */
   struct PointXYZIdsSegment
   {
      PCL_ADD_POINT4D;                    ///< quad-word XYZ
      uint32_t scan_line_id;              ///< scan line id
      uint16_t scan_id;                   ///< scan id
      uint8_t segment;                    ///< segmentation flag
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // ensure proper alignment
   } EIGEN_ALIGN16;

}; // namespace velodyne_pointcloud

POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_pointcloud::PointXYZIRSegmentation,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (uint16_t, ring, ring)
                                  (float, segmentation, segmentation)
                                  (float, segmentation_distance, segmentation_distance)
                                  (float, segmentation_intensity, segmentation_intensity)
)

POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_pointcloud::PointXYZIDR,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (float, distance, distance)
                                  (uint16_t, ring, ring)
)

POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_pointcloud::PointXYZSegmentation,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (uint8_t, segmentation, segmentation)
                                  (uint16_t, ring, ring)
)

POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_pointcloud::PointXYZIdsSegment,
                                 (float, x, x)
                                 (float, y, y)
                                 (float, z, z)
                                 (uint32_t, scan_line_id, scan_line_id)
                                 (uint16_t, scan_id, scan_id)
                                 (uint8_t, segment, segment)
)

#endif // __VELODYNE_POINTCLOUD_POINT_TYPES_H

