#include <multi_object_tracking/mot_publisher.h>

namespace MultiHypothesisTracker
{

MOTPublisher::MOTPublisher():
  m_debug_counter(0)
{
  ros::NodeHandle n("~");
  ros::NodeHandle pub_n;

  m_hypothesis_object_msg_publisher = pub_n.advertise<object_detection::ObjectDetections>("object_tracks_object_msg_tracked", 1);
  m_hypothesis_future_object_msg_publisher = pub_n.advertise<object_detection::ObjectDetections>("object_tracks_object_msg", 1);

  m_measurement_marker_publisher = n.advertise<visualization_msgs::Marker>(n.getNamespace()+ "/measurements", 1);
  m_hypothesesPublisher          = n.advertise<visualization_msgs::Marker>(n.getNamespace()+ "/object_tracks", 1);
  m_track_linePublisher          = n.advertise<visualization_msgs::Marker>(n.getNamespace()+ "/object_tracks_line", 1);
  m_measurementCovPublisher      = n.advertise<visualization_msgs::Marker>(n.getNamespace()+ "/measurements_covariances", 1);
  m_hypothesisCovPublisher       = n.advertise<visualization_msgs::Marker>(n.getNamespace()+ "/hypotheses_covariances", 1);
  m_static_objectsPublisher      = n.advertise<visualization_msgs::Marker>(n.getNamespace()+ "/static_objects", 1);
  m_dynamic_objectsPublisher     = n.advertise<visualization_msgs::Marker>(n.getNamespace()+ "/dynamic_objects", 1);
  m_hypotheses_future_publisher  = n.advertise<visualization_msgs::Marker>(n.getNamespace()+ "/future_tracks", 1);
  m_debug_publisher              = n.advertise<multi_object_tracking::DebugTracking>( n.getNamespace()+ "/debug", 1);

  n.param<std::string>("m_world_frame", m_world_frame, "world");
  n.param<double>("m_born_time_threshold", m_born_time_threshold, 0.5);
  n.param<double>("m_future_time", m_future_time, 0.0);
}

void MOTPublisher::publishAll(const std::vector<std::shared_ptr<Hypothesis>>& hypotheses)
{
  publish_hypotheses(hypotheses);
  publish_hypotheses_object_msg(hypotheses);
  publish_hypotheses_future_object_msg(hypotheses);
  publish_hypotheses_future(hypotheses);
  publish_hypothesis_covariance(hypotheses);
  publish_static_hypotheses(hypotheses);
  publish_dynamic_hypotheses(hypotheses);
//  publish_debug(hypotheses);
}

// TODO: check if correct - marker.id , ints for position , frame_id, and so on
visualization_msgs::Marker MOTPublisher::createMarker(int x, int y, int z, float r, float g, float b, std::string ns)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id    = m_world_frame;
  marker.header.stamp       = ros::Time::now();
  marker.ns                 = ns;
  marker.id                 = 0;
  marker.type               = visualization_msgs::Marker::POINTS;
  marker.action             = visualization_msgs::Marker::ADD;
  marker.pose.position.x    = x;
  marker.pose.position.y    = y;
  marker.pose.position.z    = z;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x            = 0.1;
  marker.scale.y            = 0.1;
  marker.scale.z            = 0.1;
  marker.color.a            = 1.0; // Don't forget to set the alpha!
  marker.color.r            = r;
  marker.color.g            = g;
  marker.color.b            = b;
  marker.lifetime           = ros::Duration(1.0);
  return marker;
}

void MOTPublisher::publishMeasurementMarkers(const std::vector<Measurement> &measurements)
{
  if(m_measurement_marker_publisher.getNumSubscribers() == 0)
    return;

  if(measurements.empty())
    return;

  visualization_msgs::Marker marker = createMarker(0, 0, 0, 1.0, 0.0, 0.0, "mot_measurement_markers"); //red marker
  marker.header.frame_id = measurements.at(0).frame;

  marker.points.resize(measurements.size());
  for(size_t i = 0; i < measurements.size(); i++) 
  {
    marker.points[i].x = measurements[i].pos(0);
    marker.points[i].y = measurements[i].pos(1);
    marker.points[i].z = measurements[i].pos(2);
  }
  m_measurement_marker_publisher.publish(marker);
}

void MOTPublisher::publish_measurement_covariance(const std::vector<Measurement>& measurements)
{
  for(size_t i = 0; i < measurements.size(); i++) 
  {
    visualization_msgs::Marker marker_cov;
    marker_cov.header.frame_id    = measurements[i].frame;
    marker_cov.header.stamp       = ros::Time::now();
    marker_cov.ns                 = "multi_object_tracking";
    marker_cov.id                 = 1 + (int)i;
    marker_cov.type               = visualization_msgs::Marker::SPHERE;
    marker_cov.action             = visualization_msgs::Marker::ADD;
    marker_cov.pose.position.x    = measurements[i].pos(0);
    marker_cov.pose.position.y    = measurements[i].pos(1);
    marker_cov.pose.position.z    = measurements[i].pos(2);
    marker_cov.pose.orientation.x = 0.0;
    marker_cov.pose.orientation.y = 0.0;
    marker_cov.pose.orientation.z = 0.0;
    marker_cov.pose.orientation.w = 1.0;

    marker_cov.scale.x            = sqrt(4.204) * sqrt(measurements[i].cov(0, 0));
    marker_cov.scale.y            = sqrt(4.204) * sqrt(measurements[i].cov(1, 1));
    marker_cov.scale.z            = sqrt(4.204) * sqrt(measurements[i].cov(2, 2));

    marker_cov.color.r            = 1.f;
    marker_cov.color.g            = 0.0f;
    marker_cov.color.b            = 0.0f;
    marker_cov.color.a            = 0.5f;
    marker_cov.lifetime           = ros::Duration(1.0);
    m_measurementCovPublisher.publish(marker_cov);
  }
}

void MOTPublisher::publish_hypothesis_covariance(const std::vector<std::shared_ptr<Hypothesis>>& hypotheses)
{
  double cur = getTimeHighRes();

  for (size_t i = 0; i < hypotheses.size(); i++) {
    visualization_msgs::Marker marker_cov;
    marker_cov.header.frame_id    = m_world_frame;
    marker_cov.header.stamp       = ros::Time::now();
    marker_cov.ns                 = "multi_object_tracking";
    marker_cov.id                 = 1+i;
    marker_cov.type               = visualization_msgs::Marker::SPHERE;
    marker_cov.action             = visualization_msgs::Marker::ADD;
    marker_cov.pose.position.x    = hypotheses[i]->getMean()(0);
    marker_cov.pose.position.y    = hypotheses[i]->getMean()(1);
    marker_cov.pose.position.z    = hypotheses[i]->getMean()(2);
    marker_cov.pose.orientation.x = 0.0;
    marker_cov.pose.orientation.y = 0.0;
    marker_cov.pose.orientation.z = 0.0;
    marker_cov.pose.orientation.w = 1.0;


    marker_cov.scale.x            = sqrt( 4.204 ) * sqrt(hypotheses[i]->getCovariance()(0, 0));
    marker_cov.scale.y            = sqrt( 4.204 ) * sqrt(hypotheses[i]->getCovariance()(1, 1));
    marker_cov.scale.z            = sqrt( 4.204 ) * sqrt(hypotheses[i]->getCovariance()(2, 2));

    marker_cov.color.r            = 0.0f;
    marker_cov.color.g            = 1.0f;
    marker_cov.color.b            = 0.0f;
    marker_cov.color.a            = 0.5f;
    marker_cov.lifetime           = ros::Duration(1.0);



    if (!hypotheses[i]->is_picked()  &&  ( cur -  hypotheses[i]->get_born_time() > m_born_time_threshold )  ){
      m_hypothesisCovPublisher.publish( marker_cov );
    }

  }


}



void MOTPublisher::publish_hypotheses(const std::vector<std::shared_ptr<Hypothesis>>& hypotheses)
{

  visualization_msgs::Marker hypothesis_marker= createMarker();
  double cur= getTimeHighRes();

  // Publish tracks
  for(size_t i = 0; i < hypotheses.size(); ++i)
  {
    std::shared_ptr<Hypothesis3D> hypothesis = std::static_pointer_cast<Hypothesis3D>(hypotheses[i]);

    const Eigen::Vector3d& mean = hypothesis->getMean();
    // std::cout << "publishhypothesis: mean of hypothesis " << i << " is " << mean << '\n';
    // std::cout << "color is-------------------------------------" << hypothesis->getColor() << '\n';
    geometry_msgs::Point p;
    p.x = mean(0);
    p.y = mean(1);
    p.z = mean(2);

    if (!hypothesis->is_picked()   && (cur - hypothesis->get_born_time() > m_born_time_threshold )  ){
      hypothesis_marker.points.push_back(p);
    }
    // m_hypothesesPublisher.publish (createMaker(mean(0),mean(1), mean(2), 0.0, 1.0, 0.0)); //green marker
  }
  m_hypothesesPublisher.publish (hypothesis_marker);

  // 		m_algorithm->unlockHypotheses();


}

void MOTPublisher::publish_hypotheses_object_msg(const std::vector<std::shared_ptr<Hypothesis>>& hypotheses)
{
  double cur= getTimeHighRes();
  object_detection::ObjectDetections object_detecions;
  object_detection::ObjectDetection object;
  object_detecions.header.stamp=ros::Time::now();
  object_detecions.header.frame_id=m_world_frame;


  // Publish tracks
  for(size_t i = 0; i < hypotheses.size(); ++i)
  {
    std::shared_ptr<Hypothesis3D> hypothesis = std::static_pointer_cast<Hypothesis3D>(hypotheses[i]);

    const Eigen::Vector3d& mean = hypothesis->getMean();
    object.position.x = mean(0);
    object.position.y = mean(1);
    object.position.z = mean(2);
    object.color=hypothesis->getColor();

    if (!hypothesis->is_picked()   && (cur - hypothesis->get_born_time() > m_born_time_threshold )  ){
      object_detecions.object_detections.push_back(object);
    }
  }
  m_hypothesis_object_msg_publisher.publish (object_detecions);

}

void MOTPublisher::publish_hypotheses_future_object_msg(const std::vector<std::shared_ptr<Hypothesis>>& hypotheses)
{
  double cur= getTimeHighRes();
  object_detection::ObjectDetections object_detecions;
  object_detection::ObjectDetection object;
  object_detecions.header.stamp=ros::Time::now();
  object_detecions.header.frame_id=m_world_frame;


  // Publish tracks
  for(size_t i = 0; i < hypotheses.size(); ++i)
  {
    std::shared_ptr<Hypothesis3D> hypothesis = std::static_pointer_cast<Hypothesis3D>(hypotheses[i]);
    Eigen::Vector3d mean = hypothesis->getMean();

    //Predict a little bit into the future
    mean += hypothesis->get_velocity() * m_future_time;

    object.position.x = mean(0);
    object.position.y = mean(1);
    object.position.z = mean(2);
    object.color=hypothesis->getColor();

    if (!hypothesis->is_picked()   && (cur - hypothesis->get_born_time() > m_born_time_threshold )  ){
      object_detecions.object_detections.push_back(object);
    }
  }
  m_hypothesis_future_object_msg_publisher.publish (object_detecions);

}

void MOTPublisher::publish_hypotheses_future(const std::vector<std::shared_ptr<Hypothesis>>& hypotheses)
{
  visualization_msgs::Marker hypothesis_marker= createMarker();
  hypothesis_marker.color.r=0.0;
  hypothesis_marker.color.g=0.0;
  hypothesis_marker.color.b=1.0;
  double cur= getTimeHighRes();

  // Publish tracks
  for(size_t i = 0; i < hypotheses.size(); ++i)
  {
    std::shared_ptr<Hypothesis3D> hypothesis = std::static_pointer_cast<Hypothesis3D>(hypotheses[i]);

    Eigen::Vector3d mean = hypothesis->getMean();

    //Predict a little bit into the future
    mean += hypothesis->get_velocity()* m_future_time;


    geometry_msgs::Point p;
    p.x = mean(0);
    p.y = mean(1);
    p.z = mean(2);

    if (!hypothesis->is_picked()   && (cur - hypothesis->get_born_time() > m_born_time_threshold )  ){
      hypothesis_marker.points.push_back(p);
    }
    // m_hypothesesPublisher.publish (createMaker(mean(0),mean(1), mean(2), 0.0, 1.0, 0.0)); //green marker
  }
  m_hypotheses_future_publisher.publish (hypothesis_marker);

  // 		m_algorithm->unlockHypotheses();

}

void MOTPublisher::publish_static_hypotheses(const std::vector<std::shared_ptr<Hypothesis>>& hypotheses)
{
  visualization_msgs::Marker static_object_marker= createMarker();
  static_object_marker.type = visualization_msgs::Marker::LINE_LIST;
  static_object_marker.color.a = 0.5; // Don't forget to set the alpha!
  double cur= getTimeHighRes();

  // Publish tracks
  for(size_t i = 0; i < hypotheses.size(); ++i){
    std::shared_ptr<Hypothesis3D> hypothesis = std::static_pointer_cast<Hypothesis3D>(hypotheses[i]);

    if (hypothesis->isStatic() && !hypothesis->is_picked() && (cur - hypothesis->get_born_time() > m_born_time_threshold )    ){
      const Eigen::Vector3d& mean = hypothesis->getMean();
      geometry_msgs::Point p;
      p.x = mean(0);
      p.y = mean(1);
      p.z = mean(2);
      static_object_marker.points.push_back(p);

      //push another point for the second point of the line
      p.x = mean(0);
      p.y = mean(1);
      p.z = mean(2) + 3;
      static_object_marker.points.push_back(p);
    }

  }
  m_static_objectsPublisher.publish (static_object_marker);

  // 		m_algorithm->unlockHypotheses();

}


void MOTPublisher::publish_dynamic_hypotheses(const std::vector<std::shared_ptr<Hypothesis>>& hypotheses)
{
  visualization_msgs::Marker dynamic_object_marker= createMarker(0,0,0 , 0.0, 0.5, 0.5);
  dynamic_object_marker.type = visualization_msgs::Marker::LINE_LIST;
  dynamic_object_marker.color.a = 0.5; // Don't forget to set the alpha!
  double cur= getTimeHighRes();

  // Publish tracks
  for(size_t i = 0; i < hypotheses.size(); ++i){
    std::shared_ptr<Hypothesis3D> hypothesis = std::static_pointer_cast<Hypothesis3D>(hypotheses[i]);

    if (!hypothesis->isStatic() && !hypothesis->is_picked() && (cur - hypothesis->get_born_time() > m_born_time_threshold )    ){
      const Eigen::Vector3d& mean = hypothesis->getMean();
      geometry_msgs::Point p;
      p.x = mean(0);
      p.y = mean(1);
      p.z = mean(2);
      dynamic_object_marker.points.push_back(p);

      //push another point for the second point of the line
      p.x = mean(0);
      p.y = mean(1);
      p.z = mean(2)+1.5;
      dynamic_object_marker.points.push_back(p);
    }

  }
  m_dynamic_objectsPublisher.publish (dynamic_object_marker);

  // 		m_algorithm->unlockHypotheses();

}


void MOTPublisher::publish_debug(const std::vector<std::shared_ptr<Hypothesis>>& hypotheses)
{
  if(hypotheses.empty())
    return;

  if(hypotheses[0]->get_latest_measurement_time() == 0)
    return;

  std::shared_ptr<Hypothesis3D> hypothesis = std::static_pointer_cast<Hypothesis3D>(hypotheses[0]);

  multi_object_tracking::DebugTracking debug_msg;
  debug_msg.header.seq = m_debug_counter++;
  debug_msg.header.stamp = ros::Time::now();
  debug_msg.header.frame_id = m_world_frame;

  debug_msg.velocity.x = hypothesis->get_velocity()(0);
  debug_msg.velocity.y = hypothesis->get_velocity()(1);
  debug_msg.velocity.z = hypothesis->get_velocity()(2);
  debug_msg.velocity_norm = hypothesis->get_velocity().norm();
  debug_msg.position.x = hypothesis->getMean()(0);
  debug_msg.position.y = hypothesis->getMean()(1);
  debug_msg.position.z = hypothesis->getMean()(2);
//  debug_msg.drone_tilt_angle = hypothesis->get_latest_measurement().rotation_angle;

  m_debug_publisher.publish(debug_msg);
}

}