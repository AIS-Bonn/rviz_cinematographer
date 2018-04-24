#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <boost/lexical_cast.hpp>
#include <list>

interactive_markers::InteractiveMarkerServer* server;
ros::ServiceClient trajectoryService;

ros::Publisher g_view_poses_array_pub;

geometry_msgs::PoseStamped g_robot_pose;
tf::TransformListener* g_tf_listener;

typedef std::list< visualization_msgs::InteractiveMarker > MarkerList;
MarkerList g_markers;
std::string g_frame_id;

using namespace visualization_msgs;
using namespace interactive_markers;

MenuHandler g_menu_handler;

void odometryCallback( const nav_msgs::Odometry::ConstPtr& odom_msg ) {
  ros::Time common_time;
  geometry_msgs::PoseStamped p_in;
  p_in.header = odom_msg->header;
  p_in.pose = odom_msg->pose.pose;
  if( odom_msg->header.frame_id == g_frame_id ) {
    g_robot_pose = p_in;
    return;
  }
  try {
    if( g_tf_listener->getLatestCommonTime( odom_msg->header.frame_id, g_frame_id, common_time, NULL ) == 0 ) {
      geometry_msgs::PoseStamped p_out;
      g_tf_listener->transformPose( g_frame_id, common_time, p_in, g_frame_id, p_out );
      g_robot_pose = p_out;
    }
    else {
      ROS_ERROR( "Interactive Trajectory Control: Failed to transform odometry." );
    }
  } catch( ... ) {
  }
}

visualization_msgs::Marker makeBox(visualization_msgs::InteractiveMarker &msg) {
  visualization_msgs::Marker marker;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.scale.x = msg.scale * 0.45;
  marker.scale.y = msg.scale * 0.45;
  marker.scale.z = msg.scale * 0.45;
  marker.color.r = 1.;
  marker.color.g = 0.;
  marker.color.b = 0.;
  marker.color.a = 1.0;
  return marker;
}

visualization_msgs::Marker makeArrow(visualization_msgs::InteractiveMarker &msg) {
  visualization_msgs::Marker marker;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.pose.orientation.w = 1.;
  marker.scale.x = msg.scale * 0.7;
  marker.scale.y = msg.scale * 0.1;
  marker.scale.z = msg.scale * 0.1;
  marker.color.r = 1.;
  marker.color.g = 0.;
  marker.color.b = 0.;
  marker.color.a = 1.0;
  return marker;
}

visualization_msgs::InteractiveMarkerControl& makeBoxControl(
        visualization_msgs::InteractiveMarker &msg) {
  visualization_msgs::InteractiveMarkerControl control;
  control.always_visible = true;
  control.markers.push_back(makeBox(msg));
  control.markers.push_back(makeArrow(msg));
  //control.markers.push_back(makeArrow(msg));
  msg.controls.push_back(control);
  return msg.controls.back();
}

visualization_msgs::InteractiveMarker makeMarker() {
  visualization_msgs::InteractiveMarker marker;
  marker.header.frame_id = g_frame_id;
  marker.name = "wp0";
  marker.description = "WP 0\n(click to submit)";
  marker.scale = 2.22;

  visualization_msgs::InteractiveMarkerControl& submitControl = makeBoxControl(
          marker);
  submitControl.interaction_mode =
          visualization_msgs::InteractiveMarkerControl::BUTTON;
  submitControl.name = "submit_button";

  visualization_msgs::InteractiveMarkerControl poseControl;
  poseControl.orientation.w = 1;
  poseControl.orientation.x = 0;
  poseControl.orientation.y = 1;
  poseControl.orientation.z = 0;
  poseControl.interaction_mode =
          visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE;
  marker.controls.push_back(poseControl);
  poseControl.interaction_mode =
          visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  marker.controls.push_back(poseControl);
  return marker;
}


visualization_msgs::InteractiveMarker makeTrajectory() {
  visualization_msgs::InteractiveMarker int_marker;
  visualization_msgs::InteractiveMarkerControl control;
  visualization_msgs::Marker marker;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.scale.x = 0.1;
  marker.color.r = 1.;
  marker.color.g = 0.;
  marker.color.b = 0.;
  marker.color.a = 1.0;

  control.always_visible = true;
  control.markers.push_back( marker );

  int_marker.header.frame_id = g_frame_id;
  int_marker.name = "trajectory";
  int_marker.scale = 1.0;

  int_marker.controls.push_back( control );
  return int_marker;
}

void updateTrajectory() {
  visualization_msgs::InteractiveMarker trajectory = makeTrajectory();
  for( MarkerList::const_iterator it = g_markers.begin(); it != g_markers.end(); ++it ) {
    visualization_msgs::InteractiveMarker int_marker;
    server->get( it->name, int_marker );
    trajectory.controls.front().markers.front().points.push_back( int_marker.pose.position );
    server->erase( "trajectory" );
    server->insert( trajectory );
    server->applyChanges();
  }
}

void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
  visualization_msgs::InteractiveMarker marker;

  switch (feedback->event_type) {
    case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK: {

      if( server->get( feedback->marker_name, marker ) && marker.controls[ 0 ].markers[ 0 ].color.g < 0.5  ) {
        //ROS_ERROR( "Changing Color to green" );
        marker.controls[ 0 ].markers[ 0 ].color.r = 0.;
        marker.controls[ 0 ].markers[ 0 ].color.g = 1.;
        server->erase( feedback->marker_name );
        server->insert( marker );
        server->applyChanges();
      }
    }
      break;
    case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
      if( server->get( feedback->marker_name, marker ) && marker.controls[ 0 ].markers[ 0 ].color.r < 0.5  ) {
        //ROS_ERROR( "Changing Color to red" );
        marker.controls[ 0 ].markers[ 0 ].color.r = 1.;
        marker.controls[ 0 ].markers[ 0 ].color.g = 0.;
        server->erase( feedback->marker_name );
        server->insert( marker );
      }
      updateTrajectory();
      break;
  }
  //ROS_ERROR( "Done" );
}

void submit( const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback ) {
  ROS_INFO_STREAM( "button click");
  nav_msgs::Path path;
  path.header = feedback->header;

  for( MarkerList::const_iterator it = g_markers.begin(); it != g_markers.end(); ++it ) {
    visualization_msgs::InteractiveMarker int_marker;
    server->get( it->name, int_marker );

    geometry_msgs::PoseStamped waypoint;
    waypoint.pose = int_marker.pose;
    waypoint.header = int_marker.header;
    path.poses.push_back( waypoint );
  }

  g_view_poses_array_pub.publish( path );
}

void addWaypointHere( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  ROS_INFO( "addWaypointHere" );

  MarkerList::iterator searched_element = g_markers.end();
  for( MarkerList::iterator it = g_markers.begin(); it != g_markers.end(); ++it ) {
    server->get( it->name, *it );
    server->erase( it->name );
    if( it->name == feedback->marker_name ) {
      searched_element = it;
    }
  }

  if( searched_element != g_markers.end() ) {
    visualization_msgs::InteractiveMarker new_marker = *searched_element;
    new_marker.pose.position.x = g_robot_pose.pose.position.x;
    new_marker.pose.position.y = g_robot_pose.pose.position.y;
    new_marker.pose.position.z = g_robot_pose.pose.position.z;
    searched_element = g_markers.insert( searched_element, new_marker );
  }

  size_t count = 0;
  for( MarkerList::iterator it = g_markers.begin(); it != g_markers.end(); ++it ) {
    it->name = std::string( "wp" ) + boost::lexical_cast< std::string >( count );
    it->description = std::string( "WP ") + boost::lexical_cast< std::string >( count ) + std::string( "\n(click to submit)" );
    count++;
    server->insert( *it, &processFeedback );
    g_menu_handler.apply( *server, it->name );
  }

  //g_menu_handler.reApply( *server );
  updateTrajectory();
}

void addWaypointBefore( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  ROS_INFO( "addWaypointBefore" );

  // save marker state
  geometry_msgs::Pose pose_before, pose_behind;
  bool pose_before_initialized = false;
  bool pose_behind_initialized = false;

  MarkerList::iterator searched_element = g_markers.end();
  for( MarkerList::iterator it = g_markers.begin(); it != g_markers.end(); ++it ) {
    server->get( it->name, *it );
    server->erase( it->name );
    if( it->name == feedback->marker_name ) {
      searched_element = it;
      pose_behind = it->pose;
      pose_behind_initialized = true;
    }
    else if( !pose_behind_initialized ) {
      pose_before = it->pose;
      pose_before_initialized = true;
    }
  }

  if( searched_element != g_markers.end() ) {
    visualization_msgs::InteractiveMarker new_marker = *searched_element;
    if( pose_before_initialized && pose_behind_initialized ) {
      new_marker.pose.position.x = ( pose_before.position.x + pose_behind.position.x ) / 2.;
      new_marker.pose.position.y = ( pose_before.position.y + pose_behind.position.y ) / 2.;
      new_marker.pose.position.z = ( pose_before.position.z + pose_behind.position.z ) / 2.;
    }
    else {
      new_marker.pose.position.x -= 0.5;
    }
    searched_element = g_markers.insert( searched_element, new_marker );
  }

  size_t count = 0;
  for( MarkerList::iterator it = g_markers.begin(); it != g_markers.end(); ++it ) {
    it->name = std::string( "wp" ) + boost::lexical_cast< std::string >( count );
    it->description = std::string( "WP ") + boost::lexical_cast< std::string >( count ) + std::string( "\n(click to submit)" );
    count++;
    server->insert( *it, &processFeedback );
    g_menu_handler.apply( *server, it->name );
  }

  //g_menu_handler.reApply( *server );
  updateTrajectory();
}

void addWaypointBehind( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  ROS_INFO( "addWaypointAfter" );

  // save marker state
  geometry_msgs::Pose pose_before, pose_behind;
  bool pose_before_initialized = false;
  bool pose_behind_initialized = false;

  MarkerList::iterator searched_element = g_markers.end();
  for( MarkerList::iterator it = g_markers.begin(); it != g_markers.end(); ++it ) {
    server->get( it->name, *it );
    server->erase( it->name );
    if( it->name == feedback->marker_name ) {
      searched_element = it;
      pose_before = it->pose;
      pose_before_initialized = true;
    }
    else if( pose_before_initialized && !pose_behind_initialized ) {
      pose_behind = it->pose;
      pose_behind_initialized = true;
    }
  }

  if( searched_element != g_markers.end() ) {
    visualization_msgs::InteractiveMarker new_marker = *searched_element;
    ++searched_element;
    if( pose_before_initialized && pose_behind_initialized ) {
      new_marker.pose.position.x = ( pose_before.position.x + pose_behind.position.x ) / 2.;
      new_marker.pose.position.y = ( pose_before.position.y + pose_behind.position.y ) / 2.;
      new_marker.pose.position.z = ( pose_before.position.z + pose_behind.position.z ) / 2.;
    }
    else {
      new_marker.pose.position.x -= 0.5;
    }
    searched_element = g_markers.insert( searched_element, new_marker );
  }

  size_t count = 0;
  for( MarkerList::iterator it = g_markers.begin(); it != g_markers.end(); ++it ) {
    it->name = std::string( "wp" ) + boost::lexical_cast< std::string >( count );
    it->description = std::string( "WP ") + boost::lexical_cast< std::string >( count ) + std::string( "\n(click to submit)" );
    count++;
    server->insert( *it, &processFeedback );
    g_menu_handler.apply( *server, it->name );
  }

  //g_menu_handler.reApply( *server );
  updateTrajectory();
}

void removeWaypoint( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  ROS_INFO("Remove Waypoint");
  if( g_markers.size() == 1 ) {
    ROS_ERROR( "Cannot remove last marker" );
    return;
  }

  // save marker state
  MarkerList::iterator searched_element = g_markers.end();
  for( MarkerList::iterator it = g_markers.begin(); it != g_markers.end(); ++it ) {
    server->get( it->name, *it );
    server->erase( it->name );
    if( it->name == feedback->marker_name )
      searched_element = it;
  }

  if( searched_element != g_markers.end() ) {
    g_markers.erase( searched_element );
  }

  size_t count = 0;
  for( MarkerList::iterator it = g_markers.begin(); it != g_markers.end(); ++it ) {
    it->name = std::string( "wp" ) + boost::lexical_cast< std::string >( count );
    it->description = std::string( "WP ") + boost::lexical_cast< std::string >( count ) + std::string( "\n(click to submit)" );
    count++;
    server->insert( *it, &processFeedback );
    g_menu_handler.apply( *server, it->name );
  }

  updateTrajectory();
}

void initMenu() {
  g_menu_handler.insert( "Remove waypoint", &removeWaypoint );
  g_menu_handler.insert( "Add waypoint after", &addWaypointBehind );
  g_menu_handler.insert( "Add waypoint before", &addWaypointBefore );
  g_menu_handler.insert( "Add waypoint here", &addWaypointHere);
  g_menu_handler.insert( "Submit", &submit);
}

void loadParams( ros::NodeHandle& nh ) {
  XmlRpc::XmlRpcValue poseList;
  nh.getParam( "poses", poseList );
  ROS_ASSERT( poseList.getType() == XmlRpc::XmlRpcValue::TypeArray );

  for ( int i = 0; i < poseList.size(); ++i ) {
    ROS_ASSERT( poseList[i].getType() == XmlRpc::XmlRpcValue::TypeStruct );
    XmlRpc::XmlRpcValue& v = poseList[ i ];

    geometry_msgs::Pose p;
    p.orientation.w = v["orientation"]["w"];
    p.orientation.x = v["orientation"]["x"];
    p.orientation.y = v["orientation"]["y"];
    p.orientation.z = v["orientation"]["z"];

    p.position.x = v["position"]["x"];
    p.position.y = v["position"]["y"];
    p.position.z = v["position"]["z"];

    visualization_msgs::InteractiveMarker wp_marker = makeMarker();
    wp_marker.pose = p;

    wp_marker.name = std::string( "wp" ) + boost::lexical_cast< std::string >( i );
    wp_marker.description = std::string( "WP ") + boost::lexical_cast< std::string >( i ) + std::string( "\n(click to submit)" );

    g_markers.push_back( wp_marker );

    //ROS_ERROR_STREAM( "Loaded: " << p );
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "interactive_control_node");

  ros::NodeHandle n( "~" );

  n.param< std::string >( "frame_id", g_frame_id, "local_map" );

  g_tf_listener = new tf::TransformListener();

  initMenu();

  visualization_msgs::InteractiveMarker wp_marker = makeMarker();

  server = new interactive_markers::InteractiveMarkerServer("trajectory");

  // add the interactive marker to our collection &
  // tell the server to call processFeedback() when feedback arrives for it
  if( n.hasParam( "poses" ) ) {
    loadParams( n );
  }
  else {
    g_markers.push_back( wp_marker );
  }

  for( MarkerList::const_iterator it = g_markers.begin(); it != g_markers.end(); ++it ) {
    server->insert( *it, &processFeedback);
    g_menu_handler.apply( *server, it->name );
  }

  visualization_msgs::InteractiveMarker trajectory = makeTrajectory();
  trajectory.controls.front().markers.front().points.push_back( wp_marker.pose.position );

  // 'commit' changes and send to all clients
  server->applyChanges();

  g_view_poses_array_pub = n.advertise< nav_msgs::Path >( "/transformedPath", 1 );

  ros::Subscriber odom_sub = n.subscribe( "odometry", 1, odometryCallback );

  // start the ROS main loop
  ros::Rate loopRate( 1. );
  while( ros::ok() ) {
    ros::spinOnce();
    loopRate.sleep();
  }

  delete server;

  return 0;
}
