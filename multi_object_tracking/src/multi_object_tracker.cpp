#include <multi_object_tracking/multi_object_tracker.h>

namespace MultiHypothesisTracker
{

Tracker::Tracker()
: m_multi_hypothesis_tracker(std::make_shared<HypothesisFactory>())
  , m_last_prediction_time(0)
{
  ros::NodeHandle n("~");
  ros::NodeHandle pub_n;

  m_transform_listener = std::make_shared<tf::TransformListener>();

  m_laser_detection_subscriber = n.subscribe<geometry_msgs::PoseArray>("/object_poses", 1, &Tracker::detectionCallback, this);

  n.param<double>("m_merge_close_hypotheses_distance", m_merge_close_hypotheses_distance, 0.1);
  n.param<double>("m_max_mahalanobis_distance", m_max_mahalanobis_distance, 3.75);
  n.param<std::string>("m_world_frame", m_world_frame, "world");

  setMergeDistance(m_merge_close_hypotheses_distance);
  m_multi_hypothesis_tracker.setMaxMahalanobisDistance(m_max_mahalanobis_distance);
}

void Tracker::publish()
{
  m_mot_publisher.publishAll(getHypotheses());
}

void Tracker::detectionCallback(const geometry_msgs::PoseArray::ConstPtr& msg)
{
  ROS_DEBUG_STREAM("Laser detection callback.");

//  double start = getTimeHighRes();

  std::vector<Measurement> measurements;
  convert(msg, measurements);

  if(!transformToFrame(measurements, m_world_frame))
    return;

  m_mot_publisher.publishMeasurementPositions(measurements);
  m_mot_publisher.publishMeasurementsCovariances(measurements);

  objectDetectionDataReceived(measurements);

//  std::cout << std::setprecision(10) << "\n####time for one callback " << (getTimeHighRes() - start) << " " << std::endl;
  publish();
}

void Tracker::convert(const geometry_msgs::PoseArray::ConstPtr &msg,
                      std::vector<Measurement>& measurements)
{
  Measurement measurement;

  for(size_t i = 0; i < msg->poses.size(); i++)
  {
    measurement.pos = Eigen::VectorXf(3,1);
    measurement.pos(0) = static_cast<float>(msg->poses[i].position.x);
    measurement.pos(1) = static_cast<float>(msg->poses[i].position.y);
    measurement.pos(2) = static_cast<float>(msg->poses[i].position.z);

    //TODO: radu: set covariance for the measurement to be dyamic depending on the altitude of the drone
    float measurementStd = 0.03f;
    measurement.cov = Eigen::MatrixXf(3, 3);
    measurement.cov.setIdentity();
    measurement.cov(0, 0) = measurementStd * measurementStd;
    measurement.cov(1, 1) = measurementStd * measurementStd;
    measurement.cov(2, 2) = measurementStd * measurementStd;

    measurement.frame = msg->header.frame_id;
    measurement.time = msg->header.stamp.toSec();

    measurements.push_back(measurement);
  }
}

bool Tracker::transformToFrame(std::vector<Measurement>& measurements,
                               const std::string target_frame)
{
  for(auto& measurement : measurements)
  {
    geometry_msgs::PointStamped mes_in_origin_frame;
    geometry_msgs::PointStamped mes_in_target_frame;
    mes_in_origin_frame.header.frame_id = measurement.frame;
    mes_in_origin_frame.header.stamp.fromSec(measurement.time);
    mes_in_origin_frame.point.x = measurement.pos(0);
    mes_in_origin_frame.point.y = measurement.pos(1);
    mes_in_origin_frame.point.z = measurement.pos(2);

    try
    {
      m_transform_listener->transformPoint(target_frame, mes_in_origin_frame, mes_in_target_frame);
    }
    catch(tf::TransformException& ex)
    {
      ROS_ERROR("Received an exception trying to transform a point from \"%s\" to \"%s\"", mes_in_origin_frame.header.frame_id.c_str(), target_frame.c_str());
      return false;
    }

    measurement.pos(0) = mes_in_target_frame.point.x;
    measurement.pos(1) = mes_in_target_frame.point.y;
    measurement.pos(2) = mes_in_target_frame.point.z;
    measurement.frame = target_frame;
  }

  return true;
}

void Tracker::predict(double prediction_time)
{
  if(m_last_prediction_time > 0)
    m_multi_hypothesis_tracker.predict(prediction_time - m_last_prediction_time);

  m_last_prediction_time = prediction_time;

  m_multi_hypothesis_tracker.deleteSpuriosHypotheses(prediction_time);
}

void Tracker::objectDetectionDataReceived(const std::vector<Measurement>& measurements)
{
  if(measurements.empty())
    return;

  predict(measurements.at(0).time);

  m_multi_hypothesis_tracker.correct(measurements);

  m_multi_hypothesis_tracker.mergeCloseHypotheses(m_merge_distance);
}

const std::vector<std::shared_ptr<Hypothesis>>& Tracker::getHypotheses()
{
  return m_multi_hypothesis_tracker.getHypotheses();
}

}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "multi_object_tracking");

  if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info))
    ros::console::notifyLoggerLevelsChanged();

  MultiHypothesisTracker::Tracker tracker;

  ros::spin();

  return 0;
}
