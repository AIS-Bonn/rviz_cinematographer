#ifndef ROS_TOOLS_H_INCLUDED
#define ROS_TOOLS_H_INCLUDED

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "StringTools.h"

template <typename T>
T getParamElseError ( ros::NodeHandle & nh, const std::string & paramName )
{
   T out;
   
   if ( ! nh.getParam( paramName, out ) )
   {
      ROS_ERROR_STREAM( "Parameter " << paramName << " could not be retrieved." );
   }
   
   ROS_INFO_STREAM ( paramName << ": " << to_string ( out ) );
   
   return out;
}

template <typename T>
T getParamElseThrow( ros::NodeHandle & nh, const std::string & paramName )
{
  T out;

  if ( ! nh.getParam( paramName, out ) )
  {
    throw std::string("Parameter " + paramName + " could not be retrieved." );
  }
    
  ROS_INFO_STREAM( paramName << ": " << to_string( out ) );
  return out;
}

template <typename T>
T getParamElseDefault( ros::NodeHandle & nh, const std::string & paramName, const T & def )
{
  T out;

  if ( ! nh.getParam( paramName, out ) )
  {
    ROS_INFO_STREAM( paramName << ": Default value used (" << to_string( def ) << ")" );
    return def;
  };
  
  ROS_INFO_STREAM( paramName << ": " << to_string( out ) );

  return out; 
}

void addCamPoseToMarkers (visualization_msgs::Marker * markers,
                          const Eigen::Isometry3d & camPose_cw,
                          const double camSizeInM, const Eigen::Isometry3d & T_vo = Eigen::Isometry3d::Identity());

void addTagPoseToMarkers (visualization_msgs::Marker * markers,
                          const Eigen::Isometry3d & tagPose_wt, const double tagSize, const Eigen::Isometry3d & T_vo = Eigen::Isometry3d::Identity());

geometry_msgs::Point convertToGeomMsgPoint( const Eigen::Vector3d & point );

Eigen::Vector3d convertToEigenVec (const geometry_msgs::Point & msgs_point);

template <typename T>
void getSingleMsgFromTopic( T * msg, const std::string & topicName, ros::NodeHandle & nh = ros::NodeHandle(),  const int freq = 20 )
{
  bool inited = false;
  
  typedef boost::shared_ptr< T const > TConstPtr;
  
  ros::Subscriber sub = nh.subscribe<T>(
    topicName, 1, [&] (const TConstPtr & m) { *msg = *m; inited = true; } );
  
  ros::Rate r(freq);

  while ( ros::ok() && ! inited )
  {
    ros::spinOnce();
    r.sleep();
  }
}
  
#endif // ROS_TOOLS_H_INCLUDED