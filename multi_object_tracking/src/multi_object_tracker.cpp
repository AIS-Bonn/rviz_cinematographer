#include <multi_object_tracking/multi_object_tracker.h>


namespace MultiObjectTracker
{

Tracker::Tracker():
m_debug_counter(0)
{
  ros::NodeHandle n("~");
  ros::NodeHandle pub_n;

  m_algorithm = std::make_shared<MultiObjectTrackerAlgorithm>();
  m_transformListener = std::make_shared<tf::TransformListener>();

  m_laser_detection_subscriber = n.subscribe<geometry_msgs::PoseArray>("/object_poses", 30, &Tracker::detectionCallback, this);

  m_hypothesis_object_msg_publisher = pub_n.advertise< object_detection     ::ObjectDetections >( "object_tracks_object_msg_tracked" , 1 );
  m_hypothesis_future_object_msg_publisher = pub_n.advertise< object_detection     ::ObjectDetections >( "object_tracks_object_msg" , 1 );
  m_measurement_marker_publisher   = n.advertise< visualization_msgs::Marker >( n.getNamespace()+ "/measurements", 1 );
  m_hypothesesPublisher          = n.advertise< visualization_msgs::Marker >( n.getNamespace()+ "/object_tracks" , 1 );
  m_track_linePublisher          = n.advertise< visualization_msgs::Marker >( n.getNamespace()+ "/object_tracks_line" , 1 );
  m_measurementCovPublisher      = n.advertise< visualization_msgs::Marker >( n.getNamespace()+ "/measurements_covariances" , 1 );
  m_hypothesisCovPublisher       = n.advertise< visualization_msgs::Marker >( n.getNamespace()+ "/hypotheses_covariances" , 1 );
  m_static_objectsPublisher      = n.advertise< visualization_msgs::Marker >( n.getNamespace()+ "/static_objects" , 1 );
  m_dynamic_objectsPublisher     = n.advertise< visualization_msgs::Marker >( n.getNamespace()+ "/dynamic_objects" , 1 );
  m_debug_publisher              = n.advertise< multi_object_tracking::DebugTracking >( n.getNamespace()+ "/debug" , 1 );
  m_hypotheses_future_publisher  = n.advertise< visualization_msgs::Marker >( n.getNamespace()+ "/future_tracks" , 1 );

  n.param<double>("m_merge_close_hypotheses_distance", m_merge_close_hypotheses_distance, 0.1);
  n.param<double>("m_max_mahalanobis_distance", m_max_mahalanobis_distance,3.75);
  n.param<std::string>("m_world_frame", m_world_frame, "world");
  n.param<double>("m_born_time_threshold", m_born_time_threshold, 0.5);
  n.param<double>("m_future_time", m_future_time, 0.0);
//  n.param<double>("m_spurious_time", m_spurious_time, 4.0);
//  n.param<double>("m_time_start_velocity_decay", m_time_start_velocity_decay, 1.0);
//  n.param<double>("m_time_finish_velocity_decay", m_time_finish_velocity_decay, 4.0);


  m_algorithm->set_merge_close_hypotheses_distance (m_merge_close_hypotheses_distance);
  m_algorithm->m_multi_hypothesis_tracker.set_max_mahalanobis_distance(m_max_mahalanobis_distance);
  // m_algorithm->m_multi_hypothesis_tracker.set_spurious_time(m_spurious_time);
  // m_algorithm->m_multi_hypothesis_tracker.set_time_start_velocity_decay (m_time_start_velocity_decay);
  // m_algorithm->m_multi_hypothesis_tracker.set_time_finish_velocity_decay (m_time_finish_velocity_decay);

}

Tracker::~Tracker() {
}

void Tracker::update() {
  // std::cout << std::endl << "predict without measurement" << '\n';
  m_algorithm->predictWithoutMeasurement();
}

void Tracker::detectionCallback(const geometry_msgs::PoseArray::ConstPtr& msg)
{
  ROS_INFO_STREAM("laser callback");

  std::vector<Measurement> measurements = laser_detections2measurements(msg);

  publish_debug();

  if(!transform_to_frame(measurements, m_world_frame))
    return;

  publish_measurement_markers(measurements);
  publish_measurement_covariance(measurements);

  std::string source = "all";
  m_algorithm->objectDetectionDataReceived(measurements, source);


  // std::vector< unsigned int > assignments = m_algorithm->objectDetectionDataReceived( measurements, source);

  //Full track for first hypothesis
  // std::vector<MultiHypothesisTracker::Hypothesis*> hypotheses = m_algorithm->getHypotheses();
  // MultiObjectHypothesis *hypothesis = (MultiObjectHypothesis *) hypotheses[0];
  // const vnl_vector<double> &mean = hypothesis->getMean();
  // geometry_msgs::Point p;
  // p.x=mean(0);
  // p.y=mean(1);
  // p.z=mean(2);
  // full_track.points.push_back(p);
  // m_track_linePublisher.publish(full_track);
}

// TODO return measurements efficiently
std::vector<Measurement> Tracker::laser_detections2measurements(const geometry_msgs::PoseArray::ConstPtr& msg)
{
  Measurement measurement;
  std::vector<Measurement> measurements;

  for(size_t i = 0; i < msg->poses.size(); i++)
  {
    measurement.pos = vnl_vector<double>(3);
    measurement.pos(0) = msg->poses[i].position.x;
    measurement.pos(1) = msg->poses[i].position.y;
    measurement.pos(2) = msg->poses[i].position.z;

    //TODO: radu: set covariance for the measurement to be dyamic depending on the altitude of the drone
    vnl_matrix<double> measurementCovariance = vnl_matrix<double>(3, 3);
    measurementCovariance.set_identity();
    double measurementStd = 0.03;
    measurementCovariance(0, 0) = measurementStd * measurementStd;
    measurementCovariance(1, 1) = measurementStd * measurementStd;
    measurementCovariance(2, 2) = measurementStd * measurementStd;
    measurement.cov = measurementCovariance;

    measurement.color = 'U'; // for unknown

    measurement.frame = msg->header.frame_id;

    measurement.time = msg->header.stamp.toSec();

    measurements.push_back(measurement);
  }

  return measurements;
}

bool Tracker::transform_to_frame(std::vector<Measurement>& measurements,
                                 const std::string target_frame)
{
  // TODO refactoring to range based
  for(size_t i = 0; i < measurements.size(); i++)
  {
    geometry_msgs::PointStamped mes_in_origin_frame;
    geometry_msgs::PointStamped mes_in_target_frame;
    mes_in_origin_frame.header.frame_id = measurements[i].frame;
    mes_in_origin_frame.point.x = measurements[i].pos(0);
    mes_in_origin_frame.point.y = measurements[i].pos(1);
    mes_in_origin_frame.point.z = measurements[i].pos(2);

    // TODO prob. delete
    mes_in_target_frame.header.frame_id = target_frame;
    try
    {
      m_transformListener->transformPoint(target_frame, mes_in_origin_frame, mes_in_target_frame);
    }
    catch(tf::TransformException& ex)
    {
      ROS_ERROR("Received an exception trying to transform a point from \"%s\" to \"%s\"", mes_in_origin_frame.header.frame_id.c_str(), target_frame.c_str());
      return false;
    }

    measurements[i].pos(0) = mes_in_target_frame.point.x;
    measurements[i].pos(1) = mes_in_target_frame.point.y;
    measurements[i].pos(2) = mes_in_target_frame.point.z;
    measurements[i].frame = target_frame;
  }

  return true;
}

// TODO: check if correct - marker.id , ints for position , frame_id, and so on
visualization_msgs::Marker Tracker::createMarker(int x, int y, int z, float r, float g, float b)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id    = m_world_frame;
  marker.header.stamp       = ros::Time::now();
  marker.ns                 = "multi_object_tracking";
  marker.id                 = 0;
  // marker.type            = visualization_msgs::Marker::SPHERE;
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

void Tracker::publish_measurement_markers(std::vector<Measurement> measurements)
{
  if(measurements.empty())
    return;

  visualization_msgs::Marker mes_marker = createMarker(0, 0, 0, 1.0, 0.0, 0.0); //red marker
  mes_marker.points.resize(measurements.size());
  mes_marker.header.frame_id = m_world_frame;

  for(size_t i = 0; i < measurements.size(); i++) 
  {
    mes_marker.points[i].x = measurements[i].pos(0);
    mes_marker.points[i].y = measurements[i].pos(1);
    mes_marker.points[i].z = measurements[i].pos(2);
    // std::cout << "publish_measurement_markers:: object "<< i << " is " <<  measurements[i].pos << '\n';
  }
  m_measurement_marker_publisher.publish(mes_marker);
}

void Tracker::publish_measurement_covariance(std::vector<Measurement> measurements)
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

void Tracker::publish_hypothesis_covariance(){
  std::vector<MultiHypothesisTracker::Hypothesis*> hypotheses = m_algorithm->getHypotheses();
  double cur= MultiHypothesisTracker::getTimeHighRes();

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



void Tracker::publish_hypotheses() {

  std::vector<MultiHypothesisTracker::Hypothesis*> hypotheses = m_algorithm->getHypotheses();
  visualization_msgs::Marker hypothesis_marker= createMarker();
  double cur= MultiHypothesisTracker::getTimeHighRes();

  // Publish tracks
  for(size_t i = 0; i < hypotheses.size(); ++i)
  {
    MultiObjectHypothesis *hypothesis = (MultiObjectHypothesis *) hypotheses[i];

    const vnl_vector<double> &mean = hypothesis->getMean();
    // std::cout << "publishhypothesis: mean of hypothesis " << i << " is " << mean << '\n';
    // std::cout << "color is-------------------------------------" << hypothesis->getColor() << '\n';
    geometry_msgs::Point p;
    p.x=mean(0);
    p.y=mean(1);
    p.z=mean(2);

    if (!hypothesis->is_picked()   && (cur - hypothesis->get_born_time() > m_born_time_threshold )  ){
      hypothesis_marker.points.push_back(p);
    }
    // m_hypothesesPublisher.publish (createMaker(mean(0),mean(1), mean(2), 0.0, 1.0, 0.0)); //green marker
  }
  m_hypothesesPublisher.publish (hypothesis_marker);

  // 		m_algorithm->unlockHypotheses();


}

void Tracker::publish_hypotheses_object_msg(){
  std::vector<MultiHypothesisTracker::Hypothesis*> hypotheses = m_algorithm->getHypotheses();
  double cur= MultiHypothesisTracker::getTimeHighRes();
  object_detection::ObjectDetections object_detecions;
  object_detection::ObjectDetection object;
  object_detecions.header.stamp=ros::Time::now();
  object_detecions.header.frame_id=m_world_frame;


  // Publish tracks
  for(size_t i = 0; i < hypotheses.size(); ++i)
  {
    MultiObjectHypothesis *hypothesis = (MultiObjectHypothesis *) hypotheses[i];

    const vnl_vector<double> &mean = hypothesis->getMean();
    object.position.x=mean(0);
    object.position.y=mean(1);
    object.position.z=mean(2);
    object.color=hypothesis->getColor();

    if (!hypothesis->is_picked()   && (cur - hypothesis->get_born_time() > m_born_time_threshold )  ){
      object_detecions.object_detections.push_back(object);
    }
  }
  m_hypothesis_object_msg_publisher.publish (object_detecions);

}

void Tracker::publish_hypotheses_future_object_msg(){
  std::vector<MultiHypothesisTracker::Hypothesis*> hypotheses = m_algorithm->getHypotheses();
  double cur= MultiHypothesisTracker::getTimeHighRes();
  object_detection::ObjectDetections object_detecions;
  object_detection::ObjectDetection object;
  object_detecions.header.stamp=ros::Time::now();
  object_detecions.header.frame_id=m_world_frame;


  // Publish tracks
  for(size_t i = 0; i < hypotheses.size(); ++i)
  {
    MultiObjectHypothesis *hypothesis = (MultiObjectHypothesis *) hypotheses[i];
    vnl_vector<double> mean = hypothesis->getMean();

    //Predict a little bit into the future
    mean += hypothesis->get_velocity()* m_future_time;

    object.position.x=mean(0);
    object.position.y=mean(1);
    object.position.z=mean(2);
    object.color=hypothesis->getColor();

    if (!hypothesis->is_picked()   && (cur - hypothesis->get_born_time() > m_born_time_threshold )  ){
      object_detecions.object_detections.push_back(object);
    }
  }
  m_hypothesis_future_object_msg_publisher.publish (object_detecions);

}

void Tracker::publish_hypotheses_future() {

  std::vector<MultiHypothesisTracker::Hypothesis*> hypotheses = m_algorithm->getHypotheses();
  visualization_msgs::Marker hypothesis_marker= createMarker();
  hypothesis_marker.color.r=0.0;
  hypothesis_marker.color.g=0.0;
  hypothesis_marker.color.b=1.0;
  double cur= MultiHypothesisTracker::getTimeHighRes();

  // Publish tracks
  for(size_t i = 0; i < hypotheses.size(); ++i)
  {
    MultiObjectHypothesis *hypothesis = (MultiObjectHypothesis *) hypotheses[i];

    vnl_vector<double> mean = hypothesis->getMean();

    //Predict a little bit into the future
    mean += hypothesis->get_velocity()* m_future_time;


    geometry_msgs::Point p;
    p.x=mean(0);
    p.y=mean(1);
    p.z=mean(2);

    if (!hypothesis->is_picked()   && (cur - hypothesis->get_born_time() > m_born_time_threshold )  ){
      hypothesis_marker.points.push_back(p);
    }
    // m_hypothesesPublisher.publish (createMaker(mean(0),mean(1), mean(2), 0.0, 1.0, 0.0)); //green marker
  }
  m_hypotheses_future_publisher.publish (hypothesis_marker);

  // 		m_algorithm->unlockHypotheses();

}

void Tracker::publish_static_hypotheses(){
  std::vector<MultiHypothesisTracker::Hypothesis*> hypotheses = m_algorithm->getHypotheses();
  visualization_msgs::Marker static_object_marker= createMarker();
  static_object_marker.type = visualization_msgs::Marker::LINE_LIST;
  static_object_marker.color.a = 0.5; // Don't forget to set the alpha!
  double cur= MultiHypothesisTracker::getTimeHighRes();

  // Publish tracks
  for(size_t i = 0; i < hypotheses.size(); ++i){
    MultiObjectHypothesis *hypothesis = (MultiObjectHypothesis *) hypotheses[i];

    if (hypothesis->isStatic() && !hypothesis->is_picked() && (cur - hypothesis->get_born_time() > m_born_time_threshold )    ){
      const vnl_vector<double> &mean = hypothesis->getMean();
      geometry_msgs::Point p;
      p.x=mean(0);
      p.y=mean(1);
      p.z=mean(2);
      static_object_marker.points.push_back(p);

      //push another point for the second point of the line
      p.x=mean(0);
      p.y=mean(1);
      p.z=mean(2)+3;
      static_object_marker.points.push_back(p);
    }

  }
  m_static_objectsPublisher.publish (static_object_marker);

  // 		m_algorithm->unlockHypotheses();

}


void Tracker::publish_dynamic_hypotheses(){
  std::vector<MultiHypothesisTracker::Hypothesis*> hypotheses = m_algorithm->getHypotheses();
  visualization_msgs::Marker dynamic_object_marker= createMarker(0,0,0 , 0.0, 0.5, 0.5);
  dynamic_object_marker.type = visualization_msgs::Marker::LINE_LIST;
  dynamic_object_marker.color.a = 0.5; // Don't forget to set the alpha!
  double cur= MultiHypothesisTracker::getTimeHighRes();

  // Publish tracks
  for(size_t i = 0; i < hypotheses.size(); ++i){
    MultiObjectHypothesis *hypothesis = (MultiObjectHypothesis *) hypotheses[i];

    if (!hypothesis->isStatic() && !hypothesis->is_picked() && (cur - hypothesis->get_born_time() > m_born_time_threshold )    ){
      const vnl_vector<double> &mean = hypothesis->getMean();
      geometry_msgs::Point p;
      p.x=mean(0);
      p.y=mean(1);
      p.z=mean(2);
      dynamic_object_marker.points.push_back(p);

      //push another point for the second point of the line
      p.x=mean(0);
      p.y=mean(1);
      p.z=mean(2)+1.5;
      dynamic_object_marker.points.push_back(p);
    }

  }
  m_dynamic_objectsPublisher.publish (dynamic_object_marker);

  // 		m_algorithm->unlockHypotheses();

}


void Tracker::publish_debug()
{
  std::vector<MultiHypothesisTracker::Hypothesis*> hypotheses = m_algorithm->getHypotheses();
  if(hypotheses.empty())
    return;

  if(hypotheses[0]->get_latest_measurement_time() == 0)
    return;

  MultiObjectHypothesis* hypothesis = (MultiObjectHypothesis*) hypotheses[0];

  multi_object_tracking::DebugTracking debug_msg;
  debug_msg.header.seq = m_debug_counter++;
  debug_msg.header.stamp = ros::Time::now();
  debug_msg.header.frame_id = m_world_frame;

  debug_msg.velocity.x = hypothesis->get_velocity()(0);
  debug_msg.velocity.y = hypothesis->get_velocity()(1);
  debug_msg.velocity.z = hypothesis->get_velocity()(2);
  debug_msg.velocity_norm = hypothesis->get_velocity().two_norm();
  debug_msg.position.x = hypothesis->getMean()(0);
  debug_msg.position.y = hypothesis->getMean()(1);
  debug_msg.position.z = hypothesis->getMean()(2);
  debug_msg.drone_tilt_angle = hypothesis->get_latest_measurement().rotation_angle;

  m_debug_publisher.publish(debug_msg);
}

}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "multi_object_tracking");
  ros::NodeHandle n;
  ros::Rate loopRate(30);

  MultiObjectTracker::Tracker tracker;

  while(n.ok())
  {
    tracker.update();
    ros::spinOnce();
    tracker.publish_hypotheses();
    tracker.publish_hypotheses_future();
    tracker.publish_hypotheses_object_msg();
    tracker.publish_hypotheses_future_object_msg();
    tracker.publish_hypothesis_covariance();
    tracker.publish_static_hypotheses();
    tracker.publish_dynamic_hypotheses();
    // tracker.publish_debug();
    loopRate.sleep();
  }

  return 0;
}
