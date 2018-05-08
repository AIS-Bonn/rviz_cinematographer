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
  , current_marker_(visualization_msgs::InteractiveMarker(), 0.5)
  , interpolation_speed_(3u)
{
  cam_pose_.orientation.w = 1.0;

  // give QObjects reasonable names
  setObjectName("TrajectoryEditor");
}

void TrajectoryEditor::initPlugin(qt_gui_cpp::PluginContext& context)
{
  ros::NodeHandle ph("~");
  camera_pose_sub_ = ph.subscribe("/rviz/current_camera_pose", 1, &TrajectoryEditor::camPoseCallback, this);
  camera_trajectory_pub_ = ph.advertise<rviz_animated_view_controller::CameraTrajectory>("/rviz/camera_trajectory", 1);
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

  connect(ui_.translation_x_spin_box, SIGNAL(valueChanged(double)), this, SLOT(updateCurrentMarker()));
  connect(ui_.translation_y_spin_box, SIGNAL(valueChanged(double)), this, SLOT(updateCurrentMarker()));
  connect(ui_.translation_z_spin_box, SIGNAL(valueChanged(double)), this, SLOT(updateCurrentMarker()));

  connect(ui_.rotation_x_spin_box, SIGNAL(valueChanged(double)), this, SLOT(updateCurrentMarker()));
  connect(ui_.rotation_y_spin_box, SIGNAL(valueChanged(double)), this, SLOT(updateCurrentMarker()));
  connect(ui_.rotation_z_spin_box, SIGNAL(valueChanged(double)), this, SLOT(updateCurrentMarker()));
  connect(ui_.rotation_w_spin_box, SIGNAL(valueChanged(double)), this, SLOT(updateCurrentMarker()));

  connect(ui_.transition_time_spin_box, SIGNAL(valueChanged(double)), this, SLOT(updateCurrentMarker()));

  connect(ui_.move_to_current_button, SIGNAL(clicked(bool)), this, SLOT(moveCamToCurrent()));
  connect(ui_.move_to_prev_button, SIGNAL(clicked(bool)), this, SLOT(moveCamToPrev()));
  connect(ui_.move_to_first_button, SIGNAL(clicked(bool)), this, SLOT(moveCamToFirst()));
  connect(ui_.move_to_next_button, SIGNAL(clicked(bool)), this, SLOT(moveCamToNext()));
  connect(ui_.move_to_last_button, SIGNAL(clicked(bool)), this, SLOT(moveCamToLast()));
  connect(ui_.set_pose_to_cam_button, SIGNAL(clicked(bool)), this, SLOT(setCurrentPoseToCam()));
  connect(ui_.frame_line_edit, SIGNAL(editingFinished()), this, SLOT(setMarkerFrames()));
  connect(ui_.splines_check_box, SIGNAL(stateChanged(int)), this, SLOT(updateTrajectory()));
  connect(ui_.open_file_push_button, SIGNAL(clicked(bool)), this, SLOT(loadTrajectoryFromFile()));
  connect(ui_.save_file_push_button, SIGNAL(clicked(bool)), this, SLOT(saveTrajectoryToFile()));

  // add widget to the user interface
  context.addWidget(widget_);

  menu_handler_.insert("Remove marker", boost::bind(&TrajectoryEditor::removeMarker, this, _1));
  menu_handler_.insert("Add marker after", boost::bind(&TrajectoryEditor::addMarkerBehind, this, _1));
  menu_handler_.insert("Add marker here", boost::bind(&TrajectoryEditor::addMarkerHere, this, _1));
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
    marker.controls[0].markers[0].color.r = 1.f;
    markers_.emplace_back(TimedMarker(std::move(marker), 0.5));
  }

  // connect markers to callback functions
  server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>("trajectory");
  for(const auto& marker : markers_)
  {
    server_->insert(marker.marker, boost::bind(&TrajectoryEditor::processFeedback, this, _1));
    menu_handler_.apply(*server_, marker.marker.name);
  }

  current_marker_ = markers_.back();
  current_marker_.marker.controls[0].markers[0].color.r = 0.f;
  current_marker_.marker.controls[0].markers[0].color.g = 1.f;

  // 'commit' changes and send to all clients
  server_->applyChanges();
}

void TrajectoryEditor::shutdownPlugin()
{
  camera_pose_sub_.shutdown();
  camera_trajectory_pub_.shutdown();
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
  visualization_msgs::Marker line_marker;
  line_marker.type = visualization_msgs::Marker::LINE_STRIP;
  line_marker.scale.x = 0.1;
  line_marker.color.r = 1.f;
  line_marker.color.a = 1.f;

  visualization_msgs::InteractiveMarkerControl control;
  control.always_visible = 1;
  control.markers.push_back(std::move(line_marker));

  visualization_msgs::InteractiveMarker trajectory;
  trajectory.header.frame_id = ui_.frame_line_edit->text().toStdString();
  trajectory.name = "trajectory";
  trajectory.scale = 1.0;
  trajectory.controls.push_back(std::move(control));

  if(ui_.splines_check_box->isChecked())
  {
    std::vector<geometry_msgs::Pose> spline_poses;
    std::vector<double> _;
    splinify(markers_, spline_poses, _, ui_.publish_rate_spin_box->value());
    for(auto& pose : spline_poses)
      trajectory.controls.front().markers.front().points.push_back(std::move(pose.position));

  }
  else
  {
    for(const auto& marker : markers_)
    {
      visualization_msgs::InteractiveMarker int_marker;
      server_->get(marker.marker.name, int_marker);
      trajectory.controls.front().markers.front().points.push_back(int_marker.pose.position);
    }
  }

  server_->erase("trajectory");
  server_->insert(trajectory);
  server_->applyChanges();
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

void TrajectoryEditor::publishTrajectory(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
  nav_msgs::Path path;
  path.header = feedback->header;

  if(ui_.splines_check_box->isChecked())
  {
    std::vector<geometry_msgs::Pose> spline_poses;
    std::vector<double> _;
    splinify(markers_, spline_poses, _, ui_.publish_rate_spin_box->value());
    for(auto& pose : spline_poses)
    {
      geometry_msgs::PoseStamped waypoint;
      waypoint.pose = pose;
      waypoint.header = path.header;
      path.poses.push_back(waypoint);
    }
  }
  else
  {
    for(const auto& marker : markers_)
    {
      visualization_msgs::InteractiveMarker int_marker;
      server_->get(marker.marker.name, int_marker);

      geometry_msgs::PoseStamped waypoint;
      waypoint.pose = int_marker.pose;
      waypoint.header = path.header;
      path.poses.push_back(waypoint);
    }
  }

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
    new_marker.controls[0].markers[0].color.r = 1.f;
    new_marker.controls[0].markers[0].color.g = 0.f;
    if(pose_before_initialized && pose_behind_initialized)
    {
      new_marker.pose.position.x = (pose_before.position.x + pose_behind.position.x) / 2.;
      new_marker.pose.position.y = (pose_before.position.y + pose_behind.position.y) / 2.;
      new_marker.pose.position.z = (pose_before.position.z + pose_behind.position.z) / 2.;

      // Compute the slerp-ed rotation
      tf::Quaternion start_orientation, end_orientation, intermediate_orientation;
      tf::quaternionMsgToTF(pose_before.orientation, start_orientation);
      tf::quaternionMsgToTF(pose_behind.orientation, end_orientation);
      intermediate_orientation = start_orientation.slerp(end_orientation, 0.5);
      tf::quaternionTFToMsg(intermediate_orientation, new_marker.pose.orientation);
    }
    else
    {
      new_marker.pose.position.x -= 0.5;
    }
    searched_element = markers_.insert(searched_element, TimedMarker(std::move(new_marker), searched_element->transition_time));
  }

  // refill server with member markers
  fillServer(markers_);

  updateTrajectory();
}

void TrajectoryEditor::addMarkerHere(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
  // delete all markers from server and safe clicked marker
  auto searched_element = markers_.end();
  for(auto it = markers_.begin(); it != markers_.end(); ++it)
  {
    server_->get(it->marker.name, it->marker);
    server_->erase(it->marker.name);
    if(it->marker.name == feedback->marker_name)
      searched_element = it;
  }

  // initialize new marker
  if(searched_element != markers_.end())
  {
    visualization_msgs::InteractiveMarker new_marker = searched_element->marker;
    new_marker.controls[0].markers[0].color.r = 1.f;
    new_marker.controls[0].markers[0].color.g = 0.f;

    markers_.insert(searched_element, TimedMarker(std::move(new_marker), searched_element->transition_time));
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
    new_marker.controls[0].markers[0].color.r = 1.f;
    new_marker.controls[0].markers[0].color.g = 0.f;
    if(pose_before_initialized && pose_behind_initialized)
    {
      new_marker.pose.position.x = (pose_before.position.x + pose_behind.position.x) / 2.;
      new_marker.pose.position.y = (pose_before.position.y + pose_behind.position.y) / 2.;
      new_marker.pose.position.z = (pose_before.position.z + pose_behind.position.z) / 2.;

      // Compute the slerp-ed rotation
      tf::Quaternion start_orientation, end_orientation, intermediate_orientation;
      tf::quaternionMsgToTF(pose_before.orientation, start_orientation);
      tf::quaternionMsgToTF(pose_behind.orientation, end_orientation);
      intermediate_orientation = start_orientation.slerp(end_orientation, 0.5);
      tf::quaternionTFToMsg(intermediate_orientation, new_marker.pose.orientation);
    }
    else
    {
      new_marker.pose.position.x -= 0.5;
    }
    auto next_element = searched_element;
    next_element++;
    searched_element = markers_.insert(next_element, TimedMarker(std::move(new_marker), searched_element->transition_time));
  }

  // refill server with member markers
  fillServer(markers_);

  updateTrajectory();
}

void TrajectoryEditor::setCurrentTo(TimedMarker& marker)
{
  marker.marker.controls[0].markers[0].color.r = 0.f;
  marker.marker.controls[0].markers[0].color.g = 1.f;
  current_marker_ = marker;

  updatePoseInGUI(marker.marker.pose, marker.transition_time);
}

void TrajectoryEditor::setCurrentFromTo(TimedMarker& old_current,
                                        TimedMarker& new_current)
{
  // update member list
  new_current.marker.controls[0].markers[0].color.r = 0.f;
  new_current.marker.controls[0].markers[0].color.g = 1.f;
  old_current.marker.controls[0].markers[0].color.r = 1.f;
  old_current.marker.controls[0].markers[0].color.g = 0.f;

  // update server
  visualization_msgs::InteractiveMarker int_marker;
  server_->get(new_current.marker.name, int_marker);
  int_marker.controls[0].markers[0].color.r = 0.f;
  int_marker.controls[0].markers[0].color.g = 1.f;
  server_->erase(new_current.marker.name);
  server_->insert(int_marker);

  server_->get(old_current.marker.name, int_marker);
  int_marker.controls[0].markers[0].color.r = 1.f;
  int_marker.controls[0].markers[0].color.g = 0.f;
  server_->erase(old_current.marker.name);
  server_->insert(int_marker);

  server_->applyChanges();

  // update current marker to prev marker
  current_marker_ = new_current;

  updatePoseInGUI(current_marker_.marker.pose, current_marker_.transition_time);
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

  // set previous marker as current
  if(searched_element != markers_.end())
  {
    // if first marker is removed, replace current by second marker
    if(searched_element == markers_.begin())
    {
      setCurrentTo(*(++searched_element));
      --searched_element;
    }
    else
    {
      setCurrentTo(*(--searched_element));
      ++searched_element;
    }
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
    marker.marker.description = std::to_string(count);
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
    wp_marker.controls[0].markers[0].color.r = 1.f;

    wp_marker.pose.orientation.w = v["orientation"]["w"];
    wp_marker.pose.orientation.x = v["orientation"]["x"];
    wp_marker.pose.orientation.y = v["orientation"]["y"];
    wp_marker.pose.orientation.z = v["orientation"]["z"];

    wp_marker.pose.position.x = v["position"]["x"];
    wp_marker.pose.position.y = v["position"]["y"];
    wp_marker.pose.position.z = v["position"]["z"];

    wp_marker.name = std::string("wp") + std::to_string(i);
    wp_marker.description = std::to_string(i);

    markers_.emplace_back(TimedMarker(std::move(wp_marker), v["transition_time"]));
  }
}

TrajectoryEditor::TimedMarker& TrajectoryEditor::getMarkerByName(const std::string& marker_name)
{
  for(auto& marker : markers_)
  {
    if(marker.marker.name == marker_name)
      return marker;
  }

  static TimedMarker tmp = TimedMarker(visualization_msgs::InteractiveMarker(), 0.5);
  return tmp;
}

void TrajectoryEditor::setCurrentPoseToCam()
{
  ui_.messages_label->setText(QString("Message: Right click on markers for options."));

  if(ui_.translation_x_spin_box->maximum() < cam_pose_.position.x ||
     ui_.translation_x_spin_box->minimum() > cam_pose_.position.x ||
     ui_.translation_y_spin_box->maximum() < cam_pose_.position.y ||
     ui_.translation_y_spin_box->minimum() > cam_pose_.position.y ||
     ui_.translation_z_spin_box->maximum() < cam_pose_.position.z ||
     ui_.translation_z_spin_box->minimum() > cam_pose_.position.z )
  {
    ui_.messages_label->setText(QString("Message: Current position is out of scope.\n\tTry moving closer to the center of the frame."));
    return;
  }

  // rotate cam pose around z axis for -90 degrees
  tf::Quaternion cam_orientation;
  tf::quaternionMsgToTF(cam_pose_.orientation, cam_orientation);
  tf::Quaternion rot_around_z_neg_90_deg(0.0, 0.0, -0.707, 0.707);
  geometry_msgs::Pose rotated_cam_pose;
  tf::quaternionTFToMsg(cam_orientation * rot_around_z_neg_90_deg, rotated_cam_pose.orientation);
  rotated_cam_pose.position = cam_pose_.position;

  current_marker_.marker.pose = rotated_cam_pose;

  updatePoseInGUI(rotated_cam_pose, current_marker_.transition_time);

  // update marker pose
  getMarkerByName(current_marker_.marker.name).marker.pose = rotated_cam_pose;

  server_->setPose(current_marker_.marker.name, current_marker_.marker.pose, current_marker_.marker.header);
  server_->applyChanges();
  updateTrajectory();
}

void TrajectoryEditor::setMarkerFrames()
{
  current_marker_.marker.header.frame_id = ui_.frame_line_edit->text().toStdString();

  for(auto& marker : markers_)
  {
    marker.marker.header.frame_id = ui_.frame_line_edit->text().toStdString();
    server_->erase(marker.marker.name);
    server_->insert(marker.marker);
  }
  server_->applyChanges();

  updateTrajectory();
}

void TrajectoryEditor::loadTrajectoryFromFile()
{
  std::string directory_path = ros::package::getPath("rqt_pose_interpolator")  + "/trajectories/";

  QString file_name = QFileDialog::getOpenFileName(widget_, "Open Trajectory", QString(directory_path.c_str()), "All Files (*);;.yaml files (*.yaml)");
  if(file_name == "")
  {
    ROS_ERROR_STREAM("No file specified.");
  }
  else
  {
    YAML::Node trajectory = YAML::LoadFile(file_name.toStdString());
    int count = 0;
    markers_.clear();
    for(const auto& pose : trajectory["poses"])
    {
      visualization_msgs::InteractiveMarker wp_marker = makeMarker();
      wp_marker.pose.orientation.y = 0.0;
      wp_marker.controls[0].markers[0].color.r = 1.f;

      wp_marker.pose.orientation.w = pose["orientation"]["w"].as<double>();
      wp_marker.pose.orientation.x = pose["orientation"]["x"].as<double>();
      wp_marker.pose.orientation.y = pose["orientation"]["y"].as<double>();
      wp_marker.pose.orientation.z = pose["orientation"]["z"].as<double>();

      wp_marker.pose.position.x = pose["position"]["x"].as<double>();
      wp_marker.pose.position.y = pose["position"]["y"].as<double>();
      wp_marker.pose.position.z = pose["position"]["z"].as<double>();

      wp_marker.name = std::string("wp") + std::to_string(count);
      wp_marker.description = std::to_string(count);

      markers_.emplace_back(TimedMarker(std::move(wp_marker), pose["transition_time"].as<double>()));
      count++;
    }

    // green color for last marker
    markers_.back().marker.controls[0].markers[0].color.r = 0.f;
    markers_.back().marker.controls[0].markers[0].color.g = 1.f;

    // connect markers to callback functions
    for(const auto& marker : markers_)
    {
      server_->insert(marker.marker, boost::bind(&TrajectoryEditor::processFeedback, this, _1));
      menu_handler_.apply(*server_, marker.marker.name);
    }

    // 'commit' changes and send to all clients
    server_->applyChanges();

    current_marker_ = markers_.back();

    updatePoseInGUI(current_marker_.marker.pose, current_marker_.transition_time);

    updateTrajectory();
  }
}

void TrajectoryEditor::saveTrajectoryToFile()
{
  std::string directory_path = ros::package::getPath("rqt_pose_interpolator")  + "/trajectories/";

  std::string file_path = QFileDialog::getSaveFileName(widget_, "Save Trajectory", QString(directory_path.c_str()), "All Files (*)").toStdString();
  if(file_path.empty())
  {
    ROS_ERROR_STREAM("No file specified.");
  }
  else
  {
    std::string extension = boost::filesystem::extension(file_path);

    if(extension != ".yaml")
      file_path = file_path + std::string(".yaml");

    safeTrajectoryToFile(file_path);
  }
}

rviz_animated_view_controller::CameraMovement TrajectoryEditor::makeCameraMovement()
{
  rviz_animated_view_controller::CameraMovement cm;
  cm.eye.header.stamp = ros::Time::now();
  cm.eye.header.frame_id = ui_.frame_line_edit->text().toStdString();
  cm.interpolation_speed = rviz_animated_view_controller::CameraMovement::WAVE;
  cm.transition_time = ros::Duration(0);

  cm.up.header = cm.focus.header = cm.eye.header;

  cm.up.vector.x = 0.0;
  cm.up.vector.y = 0.0;
  cm.up.vector.z = 1.0;

  return cm;
}

visualization_msgs::InteractiveMarker TrajectoryEditor::makeMarker(double x, double y, double z)
{
  visualization_msgs::InteractiveMarker marker;
  marker.header.frame_id = ui_.frame_line_edit->text().toStdString();
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

void TrajectoryEditor::convertMarkerToCamMovement(const TimedMarker& marker,
                                                  rviz_animated_view_controller::CameraMovement& cam_movement)
{
  cam_movement = makeCameraMovement();
  cam_movement.transition_time = ros::Duration(marker.transition_time);

  if(!ui_.use_up_of_world_check_box->isChecked())
  {
    // in the cam frame up is the negative x direction
    tf::Vector3 rotated_vector = rotateVector(tf::Vector3(-1, 0, 0), marker.marker.pose.orientation);
    cam_movement.up.vector.x = rotated_vector.x();
    cam_movement.up.vector.y = rotated_vector.y();
    cam_movement.up.vector.z = rotated_vector.z();
  }

  // look from
  cam_movement.eye.point = marker.marker.pose.position;

  // look at
  tf::Vector3 rotated_vector = rotateVector(tf::Vector3(0, 0, -1), marker.marker.pose.orientation);
  cam_movement.focus.point.x = marker.marker.pose.position.x + ui_.smoothness_spin_box->value() * rotated_vector.x();
  cam_movement.focus.point.y = marker.marker.pose.position.y + ui_.smoothness_spin_box->value() * rotated_vector.y();
  cam_movement.focus.point.z = marker.marker.pose.position.z + ui_.smoothness_spin_box->value() * rotated_vector.z();
}

void TrajectoryEditor::moveCamToCurrent()
{
  moveCamToMarker(current_marker_);
}

void TrajectoryEditor::moveCamToFirst()
{
  if(markers_.begin()->marker.name == current_marker_.marker.name)
    return;

  // find current marker
  auto it = ++(markers_.begin());
  for(; it != markers_.end(); ++it)
    if(it->marker.name == current_marker_.marker.name)
      break;

  // fill Camera Trajectory msg with markers and times
  rviz_animated_view_controller::CameraTrajectoryPtr cam_trajectory(new rviz_animated_view_controller::CameraTrajectory());
  cam_trajectory->target_frame = ui_.frame_line_edit->text().toStdString();
  cam_trajectory->allow_free_yaw_axis = !ui_.use_up_of_world_check_box->isChecked();

  if(ui_.splines_check_box->isChecked())
  {
    MarkerList markers;
    auto current = it;
    // TODO: increment current here but check if possible and else push to markers two times
    do
    {
      markers.push_back(*current);
    }
    while(current-- != markers_.begin());

    std::vector<geometry_msgs::Pose> spline_poses;
    std::vector<double> spline_transition_times;
    splinify(markers, spline_poses, spline_transition_times, ui_.publish_rate_spin_box->value(), false, true);
    rviz_animated_view_controller::CameraMovement cam_movement;
    bool first = true;
    for(int i = 0; i < spline_poses.size(); i++)
    {
      cam_movement.eye.point = spline_poses[i].position;
      if(!ui_.use_up_of_world_check_box->isChecked())
      {
        // in the cam frame up is the negative x direction
        tf::Vector3 rotated_vector = rotateVector(tf::Vector3(-1, 0, 0), spline_poses[i].orientation);
        cam_movement.up.vector.x = rotated_vector.x();
        cam_movement.up.vector.y = rotated_vector.y();
        cam_movement.up.vector.z = rotated_vector.z();
      }
      else
      {
        cam_movement.up.vector.z = 1.0;
      }

      tf::Vector3 rotated_vector = rotateVector(tf::Vector3(0, 0, -1), spline_poses[i].orientation);
      cam_movement.focus.point.x = spline_poses[i].position.x + ui_.smoothness_spin_box->value() * rotated_vector.x();
      cam_movement.focus.point.y = spline_poses[i].position.y + ui_.smoothness_spin_box->value() * rotated_vector.y();
      cam_movement.focus.point.z = spline_poses[i].position.z + ui_.smoothness_spin_box->value() * rotated_vector.z();

      cam_movement.interpolation_speed = (first) ? rviz_animated_view_controller::CameraMovement::RISING : rviz_animated_view_controller::CameraMovement::FULL;
      first = false;

      cam_movement.transition_time = ros::Duration(spline_transition_times[i]);

      cam_trajectory->trajectory.push_back(cam_movement);
    }
    cam_trajectory->trajectory.back().interpolation_speed = rviz_animated_view_controller::CameraMovement::DECLINING;
  }
  else
  {
    rviz_animated_view_controller::CameraMovement cam_movement;
    bool first = true;
    auto previous = it;
    do
    {
      previous--;
      convertMarkerToCamMovement(*previous, cam_movement);

      // set interpolation speed first rising, then full speed, then declining
      cam_movement.interpolation_speed = (first) ? rviz_animated_view_controller::CameraMovement::RISING : rviz_animated_view_controller::CameraMovement::FULL;
      if(previous == markers_.begin())
      {
        // if the whole trajectory is between the first two markers, use WAVE, else decline
        if(first)
          cam_movement.interpolation_speed = rviz_animated_view_controller::CameraMovement::WAVE;
        else
          cam_movement.interpolation_speed = rviz_animated_view_controller::CameraMovement::DECLINING;
      }

      cam_trajectory->trajectory.push_back(cam_movement);
      first = false;
    }
    while(previous != markers_.begin());
  }

  setCurrentFromTo(*it, *(markers_.begin()));

  // publish cam trajectory
  camera_trajectory_pub_.publish(cam_trajectory);
}

void TrajectoryEditor::moveCamToPrev()
{
  if(markers_.begin()->marker.name == current_marker_.marker.name)
    return;

  auto it = ++(markers_.begin()), prev_marker = ++(markers_.begin());
  for(; it != markers_.end(); ++it)
  {
    // find current marker
    if(it->marker.name == current_marker_.marker.name)
    {
      prev_marker = it;
      --prev_marker;
      break;
    }
  }

  setCurrentFromTo(*it, *prev_marker);

  moveCamToMarker(current_marker_);
}

void TrajectoryEditor::moveCamToNext()
{
  // do nothing if current is already the last marker
  auto last_marker = markers_.end();
  last_marker--;
  if(last_marker->marker.name == current_marker_.marker.name)
    return;

  // find iterator to current marker
  auto it = markers_.begin(), next_marker = markers_.begin();
  for(; it != markers_.end(); ++it)
  {
    if(it->marker.name == current_marker_.marker.name)
    {
      next_marker = it;
      next_marker++;
      break;
    }
  }

  setCurrentFromTo(*it, *next_marker);

  moveCamToMarker(current_marker_);
}

void TrajectoryEditor::moveCamToLast()
{
  // do nothing if current is already the last marker
  auto last_marker = markers_.end();
  last_marker--;
  if(last_marker->marker.name == current_marker_.marker.name)
    return;

  // find current marker
  auto it = markers_.begin();
  for(; it != markers_.end(); ++it)
    if(it->marker.name == current_marker_.marker.name)
      break;

  // fill Camera Trajectory msg with markers and times
  rviz_animated_view_controller::CameraTrajectoryPtr cam_trajectory(new rviz_animated_view_controller::CameraTrajectory());
  cam_trajectory->target_frame = ui_.frame_line_edit->text().toStdString();
  cam_trajectory->allow_free_yaw_axis = !ui_.use_up_of_world_check_box->isChecked();

  rviz_animated_view_controller::CameraMovement cam_movement;
  bool first = true;
  auto next = it;
  for(++next; next != markers_.end(); next++)
  {
    convertMarkerToCamMovement(*next, cam_movement);

    // set interpolation speed first rising, then full speed, then declining
    cam_movement.interpolation_speed = (first) ? rviz_animated_view_controller::CameraMovement::RISING : rviz_animated_view_controller::CameraMovement::FULL;
    if(next == last_marker)
    {
      // if the whole trajectory is between the last two markers, use WAVE, else decline
      if(first)
        cam_movement.interpolation_speed = rviz_animated_view_controller::CameraMovement::WAVE;
      else
        cam_movement.interpolation_speed = rviz_animated_view_controller::CameraMovement::DECLINING;
    }

    cam_trajectory->trajectory.push_back(cam_movement);
    first = false;
  }

  setCurrentFromTo(*it, *(--next));

  // publish cam trajectory
  camera_trajectory_pub_.publish(cam_trajectory);
}

void TrajectoryEditor::moveCamToMarker(const TimedMarker& marker)
{
  rviz_animated_view_controller::CameraTrajectoryPtr cam_trajectory(new rviz_animated_view_controller::CameraTrajectory());
  cam_trajectory->target_frame = ui_.frame_line_edit->text().toStdString();
  cam_trajectory->allow_free_yaw_axis = !ui_.use_up_of_world_check_box->isChecked();

  rviz_animated_view_controller::CameraMovement cp = makeCameraMovement();
  cp.transition_time = ros::Duration(marker.transition_time);
  cp.interpolation_speed = rviz_animated_view_controller::CameraMovement::WAVE;

  if(!ui_.use_up_of_world_check_box->isChecked())
  {
    // in the cam frame up is the negative x direction
    tf::Vector3 rotated_vector = rotateVector(tf::Vector3(-1, 0, 0), marker.marker.pose.orientation);
    cp.up.vector.x = rotated_vector.x();
    cp.up.vector.y = rotated_vector.y();
    cp.up.vector.z = rotated_vector.z();
  }

  // look from
  cp.eye.point = marker.marker.pose.position;

  // look at
  tf::Vector3 rotated_vector = rotateVector(tf::Vector3(0, 0, -1), marker.marker.pose.orientation);
  cp.focus.point.x = marker.marker.pose.position.x + ui_.smoothness_spin_box->value() * rotated_vector.x();
  cp.focus.point.y = marker.marker.pose.position.y + ui_.smoothness_spin_box->value() * rotated_vector.y();
  cp.focus.point.z = marker.marker.pose.position.z + ui_.smoothness_spin_box->value() * rotated_vector.z();

  cam_trajectory->trajectory.push_back(cp);

  camera_trajectory_pub_.publish(cam_trajectory);
}

void TrajectoryEditor::updatePoseInGUI(const geometry_msgs::Pose& pose,
                                       double transition_time)
{
  setValueQuietly(ui_.translation_x_spin_box, pose.position.x);
  setValueQuietly(ui_.translation_y_spin_box, pose.position.y);
  setValueQuietly(ui_.translation_z_spin_box, pose.position.z);

  setValueQuietly(ui_.rotation_x_spin_box, pose.orientation.x);
  setValueQuietly(ui_.rotation_y_spin_box, pose.orientation.y);
  setValueQuietly(ui_.rotation_z_spin_box, pose.orientation.z);
  setValueQuietly(ui_.rotation_w_spin_box, pose.orientation.w);

  setValueQuietly(ui_.transition_time_spin_box, transition_time);
}

void TrajectoryEditor::setValueQuietly(QDoubleSpinBox* spin_box, double value)
{
  bool old_block_state = spin_box->blockSignals(true);
  spin_box->setValue(value);
  spin_box->blockSignals(old_block_state);
}

void TrajectoryEditor::processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
  // update markers
  visualization_msgs::InteractiveMarker marker;
  if(feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP
     && server_->get(feedback->marker_name, marker))
  {
    current_marker_.marker.name = feedback->marker_name;
    current_marker_.marker.pose = feedback->pose;
    current_marker_.transition_time = getMarkerByName(feedback->marker_name).transition_time;

    updatePoseInGUI(feedback->pose, getMarkerByName(feedback->marker_name).transition_time);

    // update marker pose
    marker.pose = feedback->pose;
    getMarkerByName(feedback->marker_name).marker.pose = feedback->pose;

    // change color of current marker to green
    marker.controls[0].markers[0].color.r = 0.f;
    marker.controls[0].markers[0].color.g = 1.f;

    // change color of all markers back to red
    for(const auto& marker : markers_)
    {
      visualization_msgs::InteractiveMarker int_marker;
      server_->get(marker.marker.name, int_marker);
      int_marker.controls[0].markers[0].color.r = 1.f;
      int_marker.controls[0].markers[0].color.g = 0.f;
      server_->erase(marker.marker.name);
      server_->insert(int_marker);
      server_->applyChanges();
    }

    // update server
    server_->erase(feedback->marker_name);
    server_->insert(marker);
    server_->applyChanges();
  }
  updateTrajectory();
}

void TrajectoryEditor::updateCurrentMarker()
{
  current_marker_.marker.pose.position.x = ui_.translation_x_spin_box->value();
  current_marker_.marker.pose.position.y = ui_.translation_y_spin_box->value();
  current_marker_.marker.pose.position.z = ui_.translation_z_spin_box->value();

  current_marker_.marker.pose.orientation.x = ui_.rotation_x_spin_box->value();
  current_marker_.marker.pose.orientation.y = ui_.rotation_y_spin_box->value();
  current_marker_.marker.pose.orientation.z = ui_.rotation_z_spin_box->value();
  current_marker_.marker.pose.orientation.w = ui_.rotation_w_spin_box->value();

  current_marker_.transition_time = ui_.transition_time_spin_box->value();

  getMarkerByName(current_marker_.marker.name).marker.pose = current_marker_.marker.pose;
  getMarkerByName(current_marker_.marker.name).transition_time = current_marker_.transition_time;

  server_->setPose(current_marker_.marker.name, current_marker_.marker.pose, current_marker_.marker.header);
  server_->applyChanges();

  updateTrajectory();
}

tf::Vector3 TrajectoryEditor::rotateVector(const tf::Vector3& vector,
                                           const geometry_msgs::Quaternion& quat)
{
  tf::Quaternion rotation;
  tf::quaternionMsgToTF(quat, rotation);
  return tf::quatRotate(rotation, vector);
}

// TODO: split up funtion into prepareSpline and sampleSpline
void TrajectoryEditor::splinify(const MarkerList& markers,
                                std::vector<geometry_msgs::Pose>& spline_poses,
                                std::vector<double>& spline_transition_times,
                                double frequency,
                                bool duplicate_start,
                                bool duplicate_end)
{
  // put all markers positions into a vector. First and last double.
  double total_transition_time = 0.0;
  std::vector<Vector3> spline_points;
  Vector3 position;
  bool first = true;
  for(const auto& marker : markers)
  {
    position[0] = static_cast<float>(marker.marker.pose.position.x);
    position[1] = static_cast<float>(marker.marker.pose.position.y);
    position[2] = static_cast<float>(marker.marker.pose.position.z);
    spline_points.push_back(position);

    // when moving from marker A to B we only consider the transition time of B
    if(!first)
      total_transition_time += marker.transition_time;

    if(first)
    {
      first = false;
      if(duplicate_start)
        spline_points.push_back(position);
    }
  }
  if(duplicate_end)
    spline_points.push_back(position);

  // TODO split here
  UniformCRSpline<Vector3> spline(spline_points);
  // TODO and here

  // rate to sample from spline and get points
  double rate = 1.0 / frequency;
  float max_t = spline.getMaxT();
  auto current_marker = markers.begin();
  auto next_marker = ++(markers.begin());
  int current_marker_id = 0;
  double total_length = spline.totalLength();
  for(double i = 0.f; i <= max_t; i += rate)
  {
    // get position of spline
    auto interpolated_position = spline.getPosition(i);
    geometry_msgs::Pose pose;
    pose.position.x = interpolated_position[0];
    pose.position.y = interpolated_position[1];
    pose.position.z = interpolated_position[2];


    // i from 0 to 1 corresponds to the spline between the first and the second marker
    // we have to maintain iterators for slerp
    if(current_marker_id != (int)std::floor(i))
    {
      current_marker_id++;
      current_marker++;
      next_marker++;
    }
    // get slerped orientation
    tf::Quaternion start_orientation, end_orientation, intermediate_orientation;
    tf::quaternionMsgToTF(current_marker->marker.pose.orientation, start_orientation);
    tf::quaternionMsgToTF(next_marker->marker.pose.orientation, end_orientation);
    intermediate_orientation = start_orientation.slerp(end_orientation, fmod(i, 1.0));
    tf::quaternionTFToMsg(intermediate_orientation, pose.orientation);

    spline_poses.push_back(pose);

    double local_length = spline.arcLength(std::max(i - rate, 0.0), i);
    double transition_time = total_transition_time * local_length / total_length;
    spline_transition_times.push_back(transition_time);
  }
}

} // namespace
