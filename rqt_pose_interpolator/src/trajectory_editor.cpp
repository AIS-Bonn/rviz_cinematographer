/** @file
 *
 * Simple rqt plugin to edit trajectories.
 *
 * @author Jan Razlaw
 */

#include <rqt_pose_interpolator/trajectory_editor.h>

namespace pose_interpolator {

TrajectoryEditor::TrajectoryEditor()
: rqt_gui_cpp::Plugin()
  , widget_(0)
{
  cam_pose_.orientation.w = 1.0;

  // give QObjects reasonable names
  setObjectName("TrajectoryEditor");
}

void TrajectoryEditor::initPlugin(qt_gui_cpp::PluginContext& context)
{
  ros::NodeHandle ph("~");
  camera_pose_sub_ = ph.subscribe("/rviz/current_camera_pose", 1, &TrajectoryEditor::camPoseCallback, this);
  camera_placement_pub_ = ph.advertise<view_controller_msgs::CameraPlacement>("/rviz/camera_placement", 1);
  view_poses_array_pub_ = ph.advertise<nav_msgs::Path>("/transformed_path", 1);

  //TODO: remove?
  std::string frame_id_param_name = "frame_id";
  std::string test;
  getParam<std::string>(ph, frame_id_param_name, test, "local_map");

  // access standalone command line arguments
  QStringList argv = context.argv();
  // create QWidget
  widget_ = new QWidget();
  // extend the widget with all attributes and children from UI file
  ui_.setupUi(widget_);

  connect(ui_.move_to_start_button, SIGNAL(clicked(bool)), this, SLOT(moveCamToStart()));
  connect(ui_.move_to_end_button, SIGNAL(clicked(bool)), this, SLOT(moveCamToEnd()));
  connect(ui_.set_start_to_cam_button, SIGNAL(clicked(bool)), this, SLOT(setStartToCurrentCam()));
  connect(ui_.set_end_to_cam_button, SIGNAL(clicked(bool)), this, SLOT(setEndToCurrentCam()));
  connect(ui_.frame_text_edit, SIGNAL(textChanged()), this, SLOT(setMarkerFrames()));

  // add widget to the user interface
  context.addWidget(widget_);

  menu_handler_.insert("Remove marker", boost::bind(&TrajectoryEditor::removeMarker, this, _1));
  menu_handler_.insert("Add marker after", boost::bind(&TrajectoryEditor::addMarkerBehind, this, _1));
  menu_handler_.insert("Add marker before", boost::bind(&TrajectoryEditor::addMarkerBefore, this, _1));
  menu_handler_.insert("Publish trajectory", boost::bind(&TrajectoryEditor::publishTrajectory, this, _1));

  // set up markers
  std::string poses_param_name = "poses";
  if(getFullParamName(ph, poses_param_name))
  {
    loadParams(ph, poses_param_name);
  }
  else
  {
    visualization_msgs::InteractiveMarker marker = makeMarker();
    marker.name = "first_marker";
    marker.description = "Marker";
    marker.controls[0].markers[0].color.g = 1.f;
    markers_.emplace_back(TimedMarker(marker, 0.5));
  }

  // connect markers to callback functions
  server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>("trajectory");
  for(const auto& marker : markers_)
  {
    server_->insert(marker.marker, boost::bind( &TrajectoryEditor::processFeedback, this, _1));
    menu_handler_.apply(*server_, marker.marker.name);
  }

  // 'commit' changes and send to all clients
  server_->applyChanges();
}

void TrajectoryEditor::shutdownPlugin()
{
  camera_pose_sub_.shutdown();
  camera_placement_pub_.shutdown();
  view_poses_array_pub_.shutdown();
}

void TrajectoryEditor::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
{
  // TODO save intrinsic configuration, usually using:
  // instance_settings.setValue(k, v)
}

void TrajectoryEditor::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
{
  // TODO restore intrinsic configuration, usually using:
  // v = instance_settings.value(k)
}

void TrajectoryEditor::camPoseCallback(const geometry_msgs::Pose::ConstPtr& cam_pose)
{
  cam_pose_ = geometry_msgs::Pose(*cam_pose);
}

void TrajectoryEditor::updateTrajectory()
{
  visualization_msgs::Marker marker;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.scale.x = 0.1;
  marker.color.r = 1.f;
  marker.color.a = 1.f;

  visualization_msgs::InteractiveMarkerControl control;
  control.always_visible = 1;
  control.markers.push_back(marker);

  visualization_msgs::InteractiveMarker trajectory;
  trajectory.header.frame_id = ui_.frame_text_edit->toPlainText().toStdString();
  trajectory.name = "trajectory";
  trajectory.scale = 1.0;
  trajectory.controls.push_back(control);

  for(const auto& marker : markers_)
  {
    visualization_msgs::InteractiveMarker int_marker;
    server_->get(marker.marker.name, int_marker);
    trajectory.controls.front().markers.front().points.push_back(int_marker.pose.position);
    server_->erase("trajectory");
    server_->insert(trajectory);
    server_->applyChanges();
  }
}

void TrajectoryEditor::safeTrajectoryToFile(const std::string& file_path)
{
  std::ofstream file;
  file.open(file_path, std::ofstream::trunc);
  file << "poses:\n";
  for(const auto& marker : markers_)
  {
    file << std::fixed << std::setprecision(6);
    file << "  -\n";
    file << "    position:\n";
    file << "      x: " << marker.marker.pose.position.x << "\n";
    file << "      y: " << marker.marker.pose.position.y << "\n";
    file << "      z: " << marker.marker.pose.position.z << "\n";
    file << "    orientation:\n";
    file << "      x: " << marker.marker.pose.orientation.x << "\n";
    file << "      y: " << marker.marker.pose.orientation.y << "\n";
    file << "      z: " << marker.marker.pose.orientation.z << "\n";
    file << "      w: " << marker.marker.pose.orientation.w << "\n";
    file << "    transition_time: " << marker.transition_time << "\n";
  }
  file.close();
}

// TODO: evtl move to function that is triggered in gui
void TrajectoryEditor::publishTrajectory(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
  nav_msgs::Path path;
  path.header = feedback->header;

  for(const auto& marker : markers_)
  {
    visualization_msgs::InteractiveMarker int_marker;
    server_->get(marker.marker.name, int_marker);

    geometry_msgs::PoseStamped waypoint;
    waypoint.pose = int_marker.pose;
    waypoint.header = path.header;
    path.poses.push_back(waypoint);
  }

  // TODO: move to own function that is triggered by button
  std::string file_path = ros::package::getPath("rqt_pose_interpolator")  + "/trajectories/example_trajectory.yaml";
  safeTrajectoryToFile(file_path);

  view_poses_array_pub_.publish(path);
}

void TrajectoryEditor::addMarkerBefore(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
  // save marker state
  geometry_msgs::Pose pose_before, pose_behind;
  bool pose_before_initialized = false;
  bool pose_behind_initialized = false;

  // delete all markers from server and safe clicked marker and the one before
  auto searched_element = markers_.end();
  for(auto it = markers_.begin(); it != markers_.end(); ++it)
  {
    server_->get(it->marker.name, it->marker);
    server_->erase(it->marker.name);
    if(it->marker.name == feedback->marker_name)
    {
      searched_element = it;
      pose_behind = it->marker.pose;
      pose_behind_initialized = true;
    }
    else if(!pose_behind_initialized)
    {
      pose_before = it->marker.pose;
      pose_before_initialized = true;
    }
  }

  // initialize new marker between saved - or right beside if first marker selected
  if(searched_element != markers_.end())
  {
    visualization_msgs::InteractiveMarker new_marker = searched_element->marker;
    if(pose_before_initialized && pose_behind_initialized)
    {
      new_marker.pose.position.x = (pose_before.position.x + pose_behind.position.x) / 2.;
      new_marker.pose.position.y = (pose_before.position.y + pose_behind.position.y) / 2.;
      new_marker.pose.position.z = (pose_before.position.z + pose_behind.position.z) / 2.;
      //TODO: slerp on orientations to get mean of them as well
    }
    else
    {
      new_marker.pose.position.x -= 0.5;
    }
    // TODO: replace constant by time specified in gui
    searched_element = markers_.insert(searched_element, TimedMarker(new_marker, 0.5));
  }

  // refill server with member markers
  fillServer(markers_);

  updateTrajectory();
}

void TrajectoryEditor::addMarkerBehind(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
  // save marker state
  geometry_msgs::Pose pose_before, pose_behind;
  bool pose_before_initialized = false;
  bool pose_behind_initialized = false;

  // delete all markers from server and safe clicked marker and the one after
  auto searched_element = markers_.end();
  for(auto it = markers_.begin(); it != markers_.end(); ++it)
  {
    server_->get(it->marker.name, it->marker);
    server_->erase(it->marker.name);
    if(it->marker.name == feedback->marker_name)
    {
      searched_element = it;
      pose_before = it->marker.pose;
      pose_before_initialized = true;
    }
    else if(pose_before_initialized && !pose_behind_initialized)
    {
      pose_behind = it->marker.pose;
      pose_behind_initialized = true;
    }
  }

  // initialize new marker between saved - or right beside if last marker selected
  if(searched_element != markers_.end())
  {
    visualization_msgs::InteractiveMarker new_marker = searched_element->marker;
    ++searched_element;
    if(pose_before_initialized && pose_behind_initialized)
    {
      new_marker.pose.position.x = (pose_before.position.x + pose_behind.position.x) / 2.;
      new_marker.pose.position.y = (pose_before.position.y + pose_behind.position.y) / 2.;
      new_marker.pose.position.z = (pose_before.position.z + pose_behind.position.z) / 2.;
    }
    else
    {
      new_marker.pose.position.x -= 0.5;
    }
    searched_element = markers_.insert(searched_element, TimedMarker(new_marker, 0.5));
  }

  // refill server with member markers
  fillServer(markers_);

  updateTrajectory();
}

void TrajectoryEditor::removeMarker(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
  if(markers_.size() == 1)
  {
    ROS_ERROR("Cannot remove last marker");
    return;
  }

  // delete all markers from server and safe clicked marker
  auto searched_element = markers_.end();
  for(auto it = markers_.begin(); it != markers_.end(); ++it)
  {
    server_->get(it->marker.name, it->marker);
    server_->erase(it->marker.name);
    if(it->marker.name == feedback->marker_name)
      searched_element = it;
  }

  // delete selected marker from member markers
  if(searched_element != markers_.end())
    markers_.erase(searched_element);

  // refill server with member markers
  fillServer(markers_);

  updateTrajectory();
}

void TrajectoryEditor::fillServer(MarkerList& markers)
{
  size_t count = 0;
  for(auto& marker : markers)
  {
    marker.marker.name = std::string("wp") + std::to_string(count);
    marker.marker.description = std::to_string(count) + std::string("\n(right click for options)");
    count++;
    server_->insert(marker.marker, boost::bind( &TrajectoryEditor::processFeedback, this, _1));
    menu_handler_.apply(*server_, marker.marker.name);
  }
}

void TrajectoryEditor::loadParams(const ros::NodeHandle& nh,
                                  const std::string& param_name)
{
  XmlRpc::XmlRpcValue pose_list;
  nh.getParam(param_name, pose_list);
  ROS_ASSERT(pose_list.getType() == XmlRpc::XmlRpcValue::TypeArray);

  for(int i = 0; i < pose_list.size(); ++i)
  {
    ROS_ASSERT(pose_list[i].getType() == XmlRpc::XmlRpcValue::TypeStruct);
    XmlRpc::XmlRpcValue& v = pose_list[i];

    visualization_msgs::InteractiveMarker wp_marker = makeMarker();
    wp_marker.pose.orientation.y = 0.0;
    wp_marker.controls[0].markers[0].color.g = 1.f;

    wp_marker.pose.orientation.w = v["orientation"]["w"];
    wp_marker.pose.orientation.x = v["orientation"]["x"];
    wp_marker.pose.orientation.y = v["orientation"]["y"];
    wp_marker.pose.orientation.z = v["orientation"]["z"];

    wp_marker.pose.position.x = v["position"]["x"];
    wp_marker.pose.position.y = v["position"]["y"];
    wp_marker.pose.position.z = v["position"]["z"];

    wp_marker.name = std::string("wp") + std::to_string(i);
    wp_marker.description = std::to_string(i) + std::string("\n(right click for options)");

    markers_.emplace_back(TimedMarker(wp_marker, v["transition_time"]));
  }
}

visualization_msgs::InteractiveMarker& TrajectoryEditor::getMarkerByName(const std::string& marker_name)
{
  for(auto& marker : markers_)
  {
    if(marker.marker.name == marker_name)
      return marker.marker;
  }

  static visualization_msgs::InteractiveMarker tmp;
  return tmp;
}

void TrajectoryEditor::setStartToCurrentCam()
{
  ui_.messages_label->setText(QString("Message: "));

  if(ui_.start_x_spin_box->maximum() < cam_pose_.position.x ||
     ui_.start_x_spin_box->minimum() > cam_pose_.position.x ||
     ui_.start_y_spin_box->maximum() < cam_pose_.position.y ||
     ui_.start_y_spin_box->minimum() > cam_pose_.position.y ||
     ui_.start_z_spin_box->maximum() < cam_pose_.position.z ||
     ui_.start_z_spin_box->minimum() > cam_pose_.position.z )
  {
    ui_.messages_label->setText(QString("Message: Current position is out of scope.\n\tTry moving closer to the center of the frame."));
    return;
  }
  
  ui_.start_x_spin_box->setValue(cam_pose_.position.x);
  ui_.start_y_spin_box->setValue(cam_pose_.position.y);
  ui_.start_z_spin_box->setValue(cam_pose_.position.z);

  // rviz camera looks into negative z direction
  tf::Vector3 rotated_vector = rotateVector(tf::Vector3(0, 0, -1), cam_pose_.orientation);
  start_look_at_.x = cam_pose_.position.x + ui_.smoothness_spin_box->value() * rotated_vector.x();
  start_look_at_.y = cam_pose_.position.y + ui_.smoothness_spin_box->value() * rotated_vector.y();
  start_look_at_.z = cam_pose_.position.z + ui_.smoothness_spin_box->value() * rotated_vector.z();
}

void TrajectoryEditor::setEndToCurrentCam()
{
  ui_.messages_label->setText(QString("Message: "));

  if(ui_.end_x_spin_box->maximum() < cam_pose_.position.x ||
     ui_.end_x_spin_box->minimum() > cam_pose_.position.x ||
     ui_.end_y_spin_box->maximum() < cam_pose_.position.y ||
     ui_.end_y_spin_box->minimum() > cam_pose_.position.y ||
     ui_.end_z_spin_box->maximum() < cam_pose_.position.z ||
     ui_.end_z_spin_box->minimum() > cam_pose_.position.z )
  {
    ui_.messages_label->setText(QString("Message: Current position is out of scope.\n\tTry moving closer to the center of the frame."));
    return;
  }

  ui_.end_x_spin_box->setValue(cam_pose_.position.x);
  ui_.end_y_spin_box->setValue(cam_pose_.position.y);
  ui_.end_z_spin_box->setValue(cam_pose_.position.z);

  // rviz camera looks into negative z direction
  tf::Vector3 rotated_vector = rotateVector(tf::Vector3(0, 0, -1), cam_pose_.orientation);
  end_look_at_.x = cam_pose_.position.x + ui_.smoothness_spin_box->value() * rotated_vector.x();
  end_look_at_.y = cam_pose_.position.y + ui_.smoothness_spin_box->value() * rotated_vector.y();
  end_look_at_.z = cam_pose_.position.z + ui_.smoothness_spin_box->value() * rotated_vector.z();
}

void TrajectoryEditor::setMarkerFrames()
{
  start_marker_.header.frame_id = ui_.frame_text_edit->toPlainText().toStdString();
  end_marker_.header.frame_id = ui_.frame_text_edit->toPlainText().toStdString();
  server_->erase("start_marker");
  server_->erase("end_marker");
  server_->insert(start_marker_);
  server_->insert(end_marker_);
  server_->applyChanges();
}

view_controller_msgs::CameraPlacement TrajectoryEditor::makeCameraPlacement()
{
  view_controller_msgs::CameraPlacement cp;
  cp.eye.header.stamp = ros::Time::now();
  cp.eye.header.frame_id = ui_.frame_text_edit->toPlainText().toStdString();
  cp.target_frame = ui_.frame_text_edit->toPlainText().toStdString();
  cp.interpolation_mode = view_controller_msgs::CameraPlacement::FPS; // SPHERICAL
  cp.time_from_start = ros::Duration(0);

  cp.up.header = cp.focus.header = cp.eye.header;

  cp.up.vector.x = 0.0;
  cp.up.vector.y = 0.0;
  cp.up.vector.z = 1.0;

  return cp;
}

visualization_msgs::InteractiveMarker TrajectoryEditor::makeMarker(double x, double y, double z)
{
  visualization_msgs::InteractiveMarker marker;
  marker.header.frame_id = ui_.frame_text_edit->toPlainText().toStdString();
  marker.name = "marker";
  marker.description = "Marker";
  marker.scale = 2.22;
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = z;
  marker.pose.orientation.w = 1.0;
  marker.pose.orientation.y = 1.0;

  makeBoxControl(marker);

  visualization_msgs::InteractiveMarkerControl pose_control;
  pose_control.orientation.w = 1;
  pose_control.orientation.x = 1;
  pose_control.orientation.y = 0;
  pose_control.orientation.z = 0;
  pose_control.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;
  pose_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE;
  marker.controls.push_back(pose_control);

  pose_control.orientation.x = 0;
  pose_control.orientation.y = 1;
  pose_control.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;
  pose_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE;
  marker.controls.push_back(pose_control);

  pose_control.orientation.y = 0;
  pose_control.orientation.z = 1;
  pose_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE;
  marker.controls.push_back(pose_control);
  return marker;
}


void TrajectoryEditor::moveCamToStart()
{
  moveCamToStart(ui_.transition_time_spin_box->value());
}

void TrajectoryEditor::moveCamToStart(double transition_time)
{
  view_controller_msgs::CameraPlacement cp = makeCameraPlacement();
  cp.time_from_start = ros::Duration(transition_time);

  if(!ui_.use_up_of_world_radio_button->isChecked())
  {
    // in the cam frame up is the negative x direction
    tf::Vector3 rotated_vector = rotateVector(tf::Vector3(-1, 0, 0), start_marker_.pose.orientation);
    cp.up.vector.x = rotated_vector.x();
    cp.up.vector.y = rotated_vector.y();
    cp.up.vector.z = rotated_vector.z();
  }

  geometry_msgs::Point look_from;
  look_from.x = ui_.start_x_spin_box->value();
  look_from.y = ui_.start_y_spin_box->value();
  look_from.z = ui_.start_z_spin_box->value();
  cp.eye.point = look_from;

  cp.focus.point = start_look_at_;

  camera_placement_pub_.publish(cp);
}

void TrajectoryEditor::moveCamToEnd()
{
  view_controller_msgs::CameraPlacement cp = makeCameraPlacement();
  cp.time_from_start = ros::Duration(ui_.transition_time_spin_box->value());

  if(!ui_.use_up_of_world_radio_button->isChecked())
  {
    // in the cam frame up is the negative x direction 
    tf::Vector3 rotated_vector = rotateVector(tf::Vector3(-1, 0, 0), end_marker_.pose.orientation);
    cp.up.vector.x = rotated_vector.x();
    cp.up.vector.y = rotated_vector.y();
    cp.up.vector.z = rotated_vector.z();
  }

  geometry_msgs::Point look_from;
  look_from.x = ui_.end_x_spin_box->value();
  look_from.y = ui_.end_y_spin_box->value();
  look_from.z = ui_.end_z_spin_box->value();
  cp.eye.point = look_from;

  cp.focus.point = end_look_at_;

  camera_placement_pub_.publish(cp);
}

void TrajectoryEditor::processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
  if(feedback->marker_name == "start_marker")
  {
    start_marker_.pose = feedback->pose;

    ui_.start_x_spin_box->setValue(start_marker_.pose.position.x);
    ui_.start_y_spin_box->setValue(start_marker_.pose.position.y);
    ui_.start_z_spin_box->setValue(start_marker_.pose.position.z);

    tf::Vector3 rotated_vector = rotateVector(tf::Vector3(0, 0, -1), start_marker_.pose.orientation);
    start_look_at_.x = start_marker_.pose.position.x + ui_.smoothness_spin_box->value() * rotated_vector.x();
    start_look_at_.y = start_marker_.pose.position.y + ui_.smoothness_spin_box->value() * rotated_vector.y();
    start_look_at_.z = start_marker_.pose.position.z + ui_.smoothness_spin_box->value() * rotated_vector.z();
  }

  visualization_msgs::InteractiveMarker marker;

  if(feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP
     && server_->get(feedback->marker_name, marker))
  {
    marker.pose = feedback->pose;
    server_->erase( feedback->marker_name );
    server_->insert( marker );
    server_->applyChanges();
    getMarkerByName(feedback->marker_name).pose = feedback->pose;
  }
  updateTrajectory();

}

tf::Vector3 TrajectoryEditor::rotateVector(const tf::Vector3& vector,
                                           const geometry_msgs::Quaternion& quat)
{
  tf::Quaternion rotation;
  tf::quaternionMsgToTF(quat, rotation);
  return tf::quatRotate(rotation, vector);
}

} // namespace
