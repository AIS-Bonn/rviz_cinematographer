#include <multi_object_tracking/mot_publisher.h>

namespace MultiHypothesisTracker
{

MOTPublisher::MOTPublisher():
  m_debug_counter(0)
{
  ros::NodeHandle n("~");
  ros::NodeHandle pub_n;

  m_hypotheses_full_publisher = pub_n.advertise<object_detection::ObjectDetections>("hypotheses_full", 1);
  m_hypotheses_predictions_publisher = pub_n.advertise<object_detection::ObjectDetections>("hypotheses_predictions_full", 1);

  m_measurement_positions_publisher           = n.advertise<visualization_msgs::Marker>(n.getNamespace()+ "/measurements_positions", 1);
  m_measurements_covariances_publisher        = n.advertise<visualization_msgs::Marker>(n.getNamespace()+ "/measurements_covariances", 1);
  m_hypotheses_positions_publisher            = n.advertise<visualization_msgs::Marker>(n.getNamespace()+ "/hypotheses_positions", 1);
  m_track_line_publisher                      = n.advertise<visualization_msgs::Marker>(n.getNamespace()+ "/track_line", 1);
  m_hypotheses_covariance_publisher           = n.advertise<visualization_msgs::Marker>(n.getNamespace()+ "/hypotheses_covariances", 1);
  m_static_hypotheses_positions_publisher     = n.advertise<visualization_msgs::Marker>(n.getNamespace()+ "/static_hypotheses_positions", 1);
  m_dynamic_hypotheses_positions_publisher    = n.advertise<visualization_msgs::Marker>(n.getNamespace()+ "/dynamic_hypotheses_positions", 1);
  m_hypotheses_predicted_positions_publisher  = n.advertise<visualization_msgs::Marker>(n.getNamespace()+ "/hypotheses_predicted_positions", 1);
  m_debug_publisher                           = n.advertise<multi_object_tracking::DebugTracking>( n.getNamespace()+ "/debug", 1);

  n.param<std::string>("m_world_frame", m_world_frame, "world");
  n.param<double>("m_born_time_threshold", m_born_time_threshold, 0.5);
  n.param<double>("m_future_time", m_future_time, 0.0);
}

void MOTPublisher::publishAll(const std::vector<std::shared_ptr<Hypothesis>>& hypotheses)
{
  if(hypotheses.empty())
  {
    ROS_DEBUG_STREAM("No hypotheses available for publishing.");
    return;
  }

  publishHypothesesPositions(hypotheses);
  publishHypothesesFull(hypotheses);
  publishHypothesesPredictions(hypotheses);
  publishHypothesesPredictedPositions(hypotheses);
  publishHypothesesCovariances(hypotheses);
  publishStaticHypothesesPositions(hypotheses);
  publishDynamicHypothesesPositions(hypotheses);
//  publishFullTracks(hypotheses);
//  publishDebug(hypotheses);
}

visualization_msgs::Marker MOTPublisher::createMarker(float r, float g, float b, std::string ns)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id    = m_world_frame;
  marker.header.stamp       = ros::Time::now();
  marker.ns                 = ns;
  marker.id                 = 0;
  marker.type               = visualization_msgs::Marker::POINTS;
  marker.action             = visualization_msgs::Marker::ADD;
  marker.pose.position.x    = 0.0;
  marker.pose.position.y    = 0.0;
  marker.pose.position.z    = 0.0;
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

void MOTPublisher::publishMeasurementPositions(const std::vector<Measurement> &measurements)
{
  if(m_measurement_positions_publisher.getNumSubscribers() == 0 || measurements.empty())
    return;

  visualization_msgs::Marker measurement_positions_marker = createMarker(1.0, 0.0, 0.0, "mot_measurements_markers"); //red marker
  measurement_positions_marker.header.frame_id = measurements.at(0).frame;

  measurement_positions_marker.points.resize(measurements.size());
  for(size_t i = 0; i < measurements.size(); i++) 
  {
    measurement_positions_marker.points[i].x = measurements[i].pos(0);
    measurement_positions_marker.points[i].y = measurements[i].pos(1);
    measurement_positions_marker.points[i].z = measurements[i].pos(2);
  }
  m_measurement_positions_publisher.publish(measurement_positions_marker);
}

void MOTPublisher::publishMeasurementsCovariances(const std::vector<Measurement> &measurements)
{
  if(m_measurements_covariances_publisher.getNumSubscribers() == 0 || measurements.empty())
    return;

  visualization_msgs::Marker measurement_cov_marker = createMarker(1.0, 0.0, 0.0, "mot_measurement_covariance_marker");;
  measurement_cov_marker.type = visualization_msgs::Marker::SPHERE;
  measurement_cov_marker.color.a = 0.5f;

  for(size_t i = 0; i < measurements.size(); i++) 
  {
    measurement_cov_marker.header.frame_id    = measurements.at(i).frame;
    measurement_cov_marker.id                 = (int)i;
    measurement_cov_marker.pose.position.x    = measurements[i].pos(0);
    measurement_cov_marker.pose.position.y    = measurements[i].pos(1);
    measurement_cov_marker.pose.position.z    = measurements[i].pos(2);

    measurement_cov_marker.scale.x            = sqrt(4.204) * sqrt(measurements[i].cov(0, 0));
    measurement_cov_marker.scale.y            = sqrt(4.204) * sqrt(measurements[i].cov(1, 1));
    measurement_cov_marker.scale.z            = sqrt(4.204) * sqrt(measurements[i].cov(2, 2));

    m_measurements_covariances_publisher.publish(measurement_cov_marker);
  }
}

void MOTPublisher::publishHypothesesCovariances(const std::vector<std::shared_ptr<Hypothesis>> &hypotheses)
{
  if(m_hypotheses_covariance_publisher.getNumSubscribers() == 0 || hypotheses.empty())
    return;

  double current_time = getTimeHighRes();

  visualization_msgs::Marker hyp_covariance_marker = createMarker(1.0, 0.0, 0.0, "mot_hypotheses_covariance_marker");;
  hyp_covariance_marker.type = visualization_msgs::Marker::SPHERE;
  hyp_covariance_marker.color.a = 0.5f;

  for(size_t i = 0; i < hypotheses.size(); i++)
  {
    hyp_covariance_marker.id                 = (int)i;
    hyp_covariance_marker.pose.position.x    = hypotheses[i]->getPosition()(0);
    hyp_covariance_marker.pose.position.y    = hypotheses[i]->getPosition()(1);
    hyp_covariance_marker.pose.position.z    = hypotheses[i]->getPosition()(2);

    hyp_covariance_marker.scale.x            = sqrt( 4.204 ) * sqrt(hypotheses[i]->getCovariance()(0, 0));
    hyp_covariance_marker.scale.y            = sqrt( 4.204 ) * sqrt(hypotheses[i]->getCovariance()(1, 1));
    hyp_covariance_marker.scale.z            = sqrt( 4.204 ) * sqrt(hypotheses[i]->getCovariance()(2, 2));

    if(current_time - hypotheses[i]->getBornTime() > m_born_time_threshold)
      m_hypotheses_covariance_publisher.publish(hyp_covariance_marker);
  }
}



void MOTPublisher::publishHypothesesPositions(const std::vector<std::shared_ptr<Hypothesis>>& hypotheses)
{
  if(m_hypotheses_positions_publisher.getNumSubscribers() == 0 || hypotheses.empty())
    return;

  visualization_msgs::Marker hypothesis_marker = createMarker(0.0, 1.0, 0.0, "mot_hypotheses_positions_markers");
  double current_time = getTimeHighRes();

  for(size_t i = 0; i < hypotheses.size(); ++i)
  {
    std::shared_ptr<Hypothesis> hypothesis = std::static_pointer_cast<Hypothesis>(hypotheses[i]);

    const Eigen::Vector3f& mean = hypothesis->getPosition();
    geometry_msgs::Point p;
    p.x = mean(0);
    p.y = mean(1);
    p.z = mean(2);

    if(current_time - hypothesis->getBornTime() > m_born_time_threshold)
      hypothesis_marker.points.push_back(p);
  }
  m_hypotheses_positions_publisher.publish(hypothesis_marker);
}

// TODO: currently not needed but has to be implemented later with a different message type
void MOTPublisher::publishHypothesesFull(const std::vector<std::shared_ptr<Hypothesis>>& hypotheses)
{
  if(m_hypotheses_full_publisher.getNumSubscribers() == 0 || hypotheses.empty())
    return;

//  double current_time = getTimeHighRes();
//  object_detection::ObjectDetections object_detecions;
//  object_detection::ObjectDetection object;
//  object_detecions.header.stamp = ros::Time::now();
//  object_detecions.header.frame_id = m_world_frame;
//
//  // Publish tracks
//  for(size_t i = 0; i < hypotheses.size(); ++i)
//  {
//    std::shared_ptr<Hypothesis> hypothesis = std::static_pointer_cast<Hypothesis>(hypotheses[i]);
//
//    const Eigen::Vector3f& mean = hypothesis->getPosition();
//    object.position.x = mean(0);
//    object.position.y = mean(1);
//    object.position.z = mean(2);
//    object.color = hypothesis->getColor();
//
//    if(current_time - hypothesis->getBornTime() > m_born_time_threshold)
//      object_detecions.object_detections.push_back(object);
//  }
//  m_hypotheses_full_publisher.publish(object_detecions);
}

void MOTPublisher::publishHypothesesPredictions(const std::vector<std::shared_ptr<Hypothesis>>& hypotheses)
{
  if(m_hypotheses_predictions_publisher.getNumSubscribers() == 0 || hypotheses.empty())
    return;

  double current_time = getTimeHighRes();
  object_detection::ObjectDetections object_detecions;
  object_detection::ObjectDetection object;
  object_detecions.header.stamp = ros::Time::now();
  object_detecions.header.frame_id = m_world_frame;

  // Publish tracks
  for(size_t i = 0; i < hypotheses.size(); ++i)
  {
    std::shared_ptr<Hypothesis> hypothesis = std::static_pointer_cast<Hypothesis>(hypotheses[i]);
    Eigen::Vector3f mean = hypothesis->getPosition();

    //Predict a little bit into the future
    mean += hypothesis->getVelocity() * m_future_time;

    object.position.x = mean(0);
    object.position.y = mean(1);
    object.position.z = mean(2);
//    object.color = hypothesis->getColor();

    if(current_time - hypothesis->getBornTime() > m_born_time_threshold)
      object_detecions.object_detections.push_back(object);
  }
  m_hypotheses_predictions_publisher.publish(object_detecions);
}

void MOTPublisher::publishHypothesesPredictedPositions(const std::vector<std::shared_ptr<Hypothesis>>& hypotheses)
{
  if(m_hypotheses_predicted_positions_publisher.getNumSubscribers() == 0 || hypotheses.empty())
    return;

  visualization_msgs::Marker hypothesis_marker = createMarker(0.0, 0.0, 1.0, "mot_hypotheses_predicted_positions_markers");
  double current_time = getTimeHighRes();

  for(size_t i = 0; i < hypotheses.size(); ++i)
  {
    std::shared_ptr<Hypothesis> hypothesis = std::static_pointer_cast<Hypothesis>(hypotheses[i]);

    //Predict a little bit into the future
    Eigen::Vector3f mean = hypothesis->getPosition();
    mean += hypothesis->getVelocity() * m_future_time;

    geometry_msgs::Point p;
    p.x = mean(0);
    p.y = mean(1);
    p.z = mean(2);

    if(current_time - hypothesis->getBornTime() > m_born_time_threshold)
      hypothesis_marker.points.push_back(p);
  }
  m_hypotheses_predicted_positions_publisher.publish(hypothesis_marker);
}

void MOTPublisher::publishStaticHypothesesPositions(const std::vector<std::shared_ptr<Hypothesis>>& hypotheses)
{
  if(m_static_hypotheses_positions_publisher.getNumSubscribers() == 0 || hypotheses.empty())
    return;

  visualization_msgs::Marker static_objects_marker = createMarker(0.0, 1.0, 0.0, "mot_static_hypotheses_positions");
  static_objects_marker.type = visualization_msgs::Marker::LINE_LIST;
  static_objects_marker.color.a = 1;
  static_objects_marker.scale.x = 1.0;
  double current_time = getTimeHighRes();

  for(size_t i = 0; i < hypotheses.size(); ++i)
  {
    std::shared_ptr<Hypothesis> hypothesis = std::static_pointer_cast<Hypothesis>(hypotheses[i]);

    if(hypothesis->isStatic() && current_time - hypothesis->getBornTime() > m_born_time_threshold)
    {
      const Eigen::Vector3f& mean = hypothesis->getPosition();
      geometry_msgs::Point p;
      p.x = mean(0);
      p.y = mean(1);
      p.z = mean(2);
      static_objects_marker.points.push_back(p);

      //push another point for the second point of the line
      p.z += 15;
      static_objects_marker.points.push_back(p);
    }
  }
  m_static_hypotheses_positions_publisher.publish(static_objects_marker);
}

void MOTPublisher::publishDynamicHypothesesPositions(const std::vector<std::shared_ptr<Hypothesis>>& hypotheses)
{
  if(m_dynamic_hypotheses_positions_publisher.getNumSubscribers() == 0 || hypotheses.empty())
    return;

  visualization_msgs::Marker dynamic_objects_marker = createMarker(0.0, 0.5, 0.5, "mot_dynamic_hypotheses_positions");
  dynamic_objects_marker.type = visualization_msgs::Marker::LINE_LIST;
  dynamic_objects_marker.color.a = 1.0;
  dynamic_objects_marker.scale.x = 0.5;
  double current_time = getTimeHighRes();

  // Publish tracks
  for(size_t i = 0; i < hypotheses.size(); ++i)
  {
    std::shared_ptr<Hypothesis> hypothesis = std::static_pointer_cast<Hypothesis>(hypotheses[i]);

    if(!hypothesis->isStatic() && current_time - hypothesis->getBornTime() > m_born_time_threshold)
    {
      const Eigen::Vector3f& mean = hypothesis->getPosition();
      geometry_msgs::Point p;
      p.x = mean(0);
      p.y = mean(1);
      p.z = mean(2);
      dynamic_objects_marker.points.push_back(p);

      //push another point for the second point of the line
      p.z += 20;
      dynamic_objects_marker.points.push_back(p);
    }
  }
  m_dynamic_hypotheses_positions_publisher.publish(dynamic_objects_marker);
}

// TODO: implement
void MOTPublisher::publishFullTracks(const std::vector<std::shared_ptr<Hypothesis>>& hypotheses)
{
  if(m_track_line_publisher.getNumSubscribers() == 0)
    return;

  //Full track for first hypothesis
  // std::vector<Hypothesis*> hypotheses = m_algorithm->getHypotheses();
  // Hypothesis *hypothesis = (Hypothesis *) hypotheses[0];
  // const Eigen::Vector3f& mean = hypothesis->getPosition();
  // geometry_msgs::Point p;
  // p.x=mean(0);
  // p.y=mean(1);
  // p.z=mean(2);
  // full_track.points.push_back(p);
  // m_track_line_publisher.publish(full_track);

//  m_track_line_publisher.publish(dynamic_objects_marker);
}

void MOTPublisher::publishDebug(const std::vector<std::shared_ptr<Hypothesis>>& hypotheses)
{
  if(m_debug_publisher.getNumSubscribers() == 0 || hypotheses.empty())
    return;

  std::shared_ptr<Hypothesis> hypothesis = std::static_pointer_cast<Hypothesis>(hypotheses[0]);

  multi_object_tracking::DebugTracking debug_msg;
  debug_msg.header.seq = m_debug_counter++;
  debug_msg.header.stamp = ros::Time::now();
  debug_msg.header.frame_id = m_world_frame;

  debug_msg.velocity.x = hypothesis->getVelocity()(0);
  debug_msg.velocity.y = hypothesis->getVelocity()(1);
  debug_msg.velocity.z = hypothesis->getVelocity()(2);
  debug_msg.velocity_norm = hypothesis->getVelocity().norm();
  debug_msg.position.x = hypothesis->getPosition()(0);
  debug_msg.position.y = hypothesis->getPosition()(1);
  debug_msg.position.z = hypothesis->getPosition()(2);
//  debug_msg.drone_tilt_angle = hypothesis->get_latest_measurement().rotation_angle;

  m_debug_publisher.publish(debug_msg);
}

}