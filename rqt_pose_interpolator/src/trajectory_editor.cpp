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
  , current_marker_name_("")
{
  //cam_pose_.orientation.w = 1.0;

  // give QObjects reasonable names
  setObjectName("TrajectoryEditor");
}

void TrajectoryEditor::initPlugin(qt_gui_cpp::PluginContext& context)
{
  ros::NodeHandle ph("~");
  camera_trajectory_pub_ = ph.advertise<rviz_animated_view_controller::CameraTrajectory>("/rviz/camera_trajectory", 1);
  view_poses_array_pub_ = ph.advertise<nav_msgs::Path>("/transformed_path", 1, true);

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
  connect(ui_.append_cam_pose, SIGNAL(clicked(bool)), this, SLOT(appendCamPoseToTrajectory()));
  connect(ui_.set_pose_to_cam_button, SIGNAL(clicked(bool)), this, SLOT(setCurrentPoseToCam()));
  connect(ui_.frame_line_edit, SIGNAL(editingFinished()), this, SLOT(setMarkerFrames()));
  connect(ui_.splines_check_box, SIGNAL(stateChanged(int)), this, SLOT(updateTrajectory()));
  connect(ui_.marker_size_increase, SIGNAL(clicked(bool)), this, SLOT(increaseMarkerScale()));
  connect(ui_.marker_size_decrease, SIGNAL(clicked(bool)), this, SLOT(decreaseMarkerScale()));

  connect(ui_.open_file_push_button, SIGNAL(clicked(bool)), this, SLOT(loadTrajectoryFromFile()));
  connect(ui_.save_file_push_button, SIGNAL(clicked(bool)), this, SLOT(saveTrajectoryToFile()));

  // add widget to the user interface
  context.addWidget(widget_);

  menu_handler_.insert("Remove marker", boost::bind(&TrajectoryEditor::removeMarker, this, _1));
  menu_handler_.insert("Add marker after", boost::bind(&TrajectoryEditor::addMarkerBehind, this, _1));
  menu_handler_.insert("Add marker here", boost::bind(&TrajectoryEditor::addMarkerHere, this, _1));
  menu_handler_.insert("Add marker before", boost::bind(&TrajectoryEditor::addMarkerBefore, this, _1));

  // set up markers
  std::string poses_param_name = "poses";
  if(getFullParamName(ph, poses_param_name))
  {
    loadParams(ph, poses_param_name);
  }
  else
  {
    visualization_msgs::InteractiveMarker marker_0 = makeMarker();
    marker_0.name = "0";
    marker_0.description = "0";
    marker_0.controls[0].markers[0].color.g = 1.f;
    markers_.emplace_back(TimedMarker(std::move(marker_0), 2.5));
    visualization_msgs::InteractiveMarker marker_1 = makeMarker(2.0, 0.0, 1.0);
    marker_1.name = "1";
    marker_1.description = "1";
    marker_1.controls[0].markers[0].color.r = 1.f;
    markers_.emplace_back(TimedMarker(std::move(marker_1), 2.5));
  }

  setCurrentTo(markers_.front());

  // connect markers to callback functions
  server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>("trajectory");
  updateServer(markers_);
  updateTrajectory();

  camera_pose_sub_ = ph.subscribe("/rviz/current_camera_pose", 1, &TrajectoryEditor::camPoseCallback, this);
}

void TrajectoryEditor::shutdownPlugin()
{
  // create empty path to "erase" previous path on shutdown
  nav_msgs::Path path;
  path.header = markers_.front().marker.header;

  markers_.clear();
  server_->clear();

  camera_pose_sub_.shutdown();
  camera_trajectory_pub_.shutdown();

  view_poses_array_pub_.publish(path);
  usleep(1000); // sleep for a millisecond to give the publisher some time
  view_poses_array_pub_.shutdown();
}

void TrajectoryEditor::saveSettings(qt_gui_cpp::Settings& plugin_settings,
                                    qt_gui_cpp::Settings& instance_settings) const
{
  // TODO save intrinsic configuration, usually using:
  //instance_settings.setValue(k, v)
}

void TrajectoryEditor::restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
                                       const qt_gui_cpp::Settings& instance_settings)
{
  // TODO restore intrinsic configuration, usually using:
  // v = instance_settings.value(k)
}

void TrajectoryEditor::camPoseCallback(const geometry_msgs::Pose::ConstPtr& cam_pose)
{
  cam_pose_ = geometry_msgs::Pose(*cam_pose);
  server_->applyChanges();
}

void TrajectoryEditor::updateTrajectory()
{
  if(markers_.size() < 2)
    return;

  nav_msgs::Path path;
  path.header = markers_.front().marker.header;

  if(ui_.splines_check_box->isChecked())
  {
    std::vector<geometry_msgs::Pose> spline_poses;
    markersToSplinedPoses(markers_, spline_poses, ui_.publish_rate_spin_box->value());
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

void TrajectoryEditor::addMarkerBefore(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
  geometry_msgs::Pose pose_before, clicked_pose;
  bool pose_before_initialized = false;
  bool clicked_pose_initialized = false;

  // delete all markers from server and safe iterator to clicked marker and the marker before that in the trajectory
  auto clicked_element = markers_.end();
  for(auto it = markers_.begin(); it != markers_.end(); ++it)
  {
    if(it->marker.name == feedback->marker_name)
    {
      clicked_element = it;
      clicked_pose = it->marker.pose;
      clicked_pose_initialized = true;
    }
    else if(!clicked_pose_initialized)
    {
      pose_before = it->marker.pose;
      pose_before_initialized = true;
    }
  }

  colorizeMarkersRed();

  // initialize new marker between clicked and previous - or right beside clicked if first marker selected
  if(clicked_element != markers_.end())
  {
    visualization_msgs::InteractiveMarker new_marker = clicked_element->marker;
    new_marker.controls[0].markers[0].color.r = 0.f;
    new_marker.controls[0].markers[0].color.g = 1.f;
    if(pose_before_initialized && clicked_pose_initialized)
    {
      new_marker.pose.position.x = (pose_before.position.x + clicked_pose.position.x) / 2.;
      new_marker.pose.position.y = (pose_before.position.y + clicked_pose.position.y) / 2.;
      new_marker.pose.position.z = (pose_before.position.z + clicked_pose.position.z) / 2.;

      // Compute the slerp-ed rotation
      tf::Quaternion start_orientation, end_orientation, intermediate_orientation;
      tf::quaternionMsgToTF(pose_before.orientation, start_orientation);
      tf::quaternionMsgToTF(clicked_pose.orientation, end_orientation);
      intermediate_orientation = start_orientation.slerp(end_orientation, 0.5);
      tf::quaternionTFToMsg(intermediate_orientation, new_marker.pose.orientation);
    }
    else
    {
      new_marker.pose.position.x -= 0.5;
    }
    clicked_element = markers_.insert(clicked_element, TimedMarker(std::move(new_marker), clicked_element->transition_time));
  }

  current_marker_name_ = feedback->marker_name;

  // update server with updated member markers
  server_->clear();
  updateServer(markers_);

  updateTrajectory();
}

void TrajectoryEditor::addMarkerHere(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
  // delete all markers from server and safe clicked marker
  auto clicked_element = markers_.end();
  for(auto it = markers_.begin(); it != markers_.end(); ++it)
    if(it->marker.name == feedback->marker_name)
      clicked_element = it;

  colorizeMarkersRed();

  // initialize new marker at the position of the clicked marker
  if(clicked_element != markers_.end())
  {
    visualization_msgs::InteractiveMarker new_marker = clicked_element->marker;
    new_marker.controls[0].markers[0].color.r = 0.f;
    new_marker.controls[0].markers[0].color.g = 1.f;

    markers_.insert(clicked_element, TimedMarker(std::move(new_marker), clicked_element->transition_time));
  }

  current_marker_name_ = feedback->marker_name;

  // update server with updated member markers
  server_->clear();
  updateServer(markers_);

  updateTrajectory();
}

void TrajectoryEditor::addMarkerBehind(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
  geometry_msgs::Pose clicked_pose, pose_behind;
  bool clicked_pose_initialized = false;
  bool pose_behind_initialized = false;

  // delete all markers from server and safe iterator to clicked marker and the one after in trajectory
  auto clicked_element = markers_.end();
  for(auto it = markers_.begin(); it != markers_.end(); ++it)
  {
    if(it->marker.name == feedback->marker_name)
    {
      clicked_element = it;
      clicked_pose = it->marker.pose;
      clicked_pose_initialized = true;
    }
    else if(clicked_pose_initialized && !pose_behind_initialized)
    {
      pose_behind = it->marker.pose;
      pose_behind_initialized = true;
    }
  }

  colorizeMarkersRed();

  // initialize new marker between clicked and next marker - or right beside the clicked if last marker selected
  if(clicked_element != markers_.end())
  {
    visualization_msgs::InteractiveMarker new_marker = clicked_element->marker;
    new_marker.controls[0].markers[0].color.r = 0.f;
    new_marker.controls[0].markers[0].color.g = 1.f;
    if(clicked_pose_initialized && pose_behind_initialized)
    {
      new_marker.pose.position.x = (clicked_pose.position.x + pose_behind.position.x) / 2.;
      new_marker.pose.position.y = (clicked_pose.position.y + pose_behind.position.y) / 2.;
      new_marker.pose.position.z = (clicked_pose.position.z + pose_behind.position.z) / 2.;

      // Compute the slerp-ed rotation
      tf::Quaternion start_orientation, end_orientation, intermediate_orientation;
      tf::quaternionMsgToTF(clicked_pose.orientation, start_orientation);
      tf::quaternionMsgToTF(pose_behind.orientation, end_orientation);
      intermediate_orientation = start_orientation.slerp(end_orientation, 0.5);
      tf::quaternionTFToMsg(intermediate_orientation, new_marker.pose.orientation);
    }
    else
    {
      new_marker.pose.position.x -= 0.5;
    }
    clicked_element = markers_.insert(std::next(clicked_element), TimedMarker(std::move(new_marker), clicked_element->transition_time));
  }

  // name of the new marker will be the one of the clicked marker incremented by 1
  current_marker_name_ = std::to_string(std::stoi(feedback->marker_name) + 1);

  // update server with updated member markers
  server_->clear();
  updateServer(markers_);

  updateTrajectory();
}

void TrajectoryEditor::setCurrentTo(TimedMarker& marker)
{
  marker.marker.controls[0].markers[0].color.r = 0.f;
  marker.marker.controls[0].markers[0].color.g = 1.f;
  current_marker_name_ = marker.marker.name;

  updatePoseInGUI(marker.marker.pose, marker.transition_time);
}

void TrajectoryEditor::setCurrentFromTo(TimedMarker& old_current,
                                        TimedMarker& new_current)
{
  // update current marker
  current_marker_name_ = new_current.marker.name;

  updatePoseInGUI(new_current.marker.pose, new_current.transition_time);

  // update member list
  new_current.marker.controls[0].markers[0].color.r = 0.f;
  new_current.marker.controls[0].markers[0].color.g = 1.f;
  old_current.marker.controls[0].markers[0].color.r = 1.f;
  old_current.marker.controls[0].markers[0].color.g = 0.f;

  server_->clear();
  updateServer(markers_);
}

void TrajectoryEditor::removeMarker(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
  if(markers_.size() < 3)
  {
    ROS_ERROR("Cannot remove last two markers.");
    return;
  }

  // delete all markers from server and safe clicked marker
  auto searched_element = markers_.end();
  for(auto it = markers_.begin(); it != markers_.end(); ++it)
    if(it->marker.name == feedback->marker_name)
      searched_element = it;

  colorizeMarkersRed();

  // set previous marker as current
  if(searched_element != markers_.end())
  {
    // if first marker is removed, replace current by second marker
    if(searched_element == markers_.begin())
    {
      setCurrentTo(*(std::next(searched_element)));
      // second element becomes first and gets first's name - all following numbered continuously
      current_marker_name_ = markers_.front().marker.name;
    }
    else
      setCurrentTo(*(std::prev(searched_element)));
  }

  // delete selected marker from member markers
  if(searched_element != markers_.end())
    markers_.erase(searched_element);

  server_->clear();
  updateServer(markers_);

  updateTrajectory();
}

void TrajectoryEditor::updateServer(MarkerList& markers)
{
  size_t count = 0;
  for(auto& marker : markers)
  {
    marker.marker.name = std::to_string(count);
    marker.marker.description = std::to_string(count);
    count++;
    server_->insert(marker.marker, boost::bind(&TrajectoryEditor::processFeedback, this, _1));
    menu_handler_.apply(*server_, marker.marker.name);
  }

  server_->applyChanges();
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

    wp_marker.name = std::to_string(i);
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

bool TrajectoryEditor::isCamWithinBounds()
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
    return false;
  }
  return true;
}

void TrajectoryEditor::rvizCamToMarkerOrientation(const geometry_msgs::Pose& rviz_cam_pose,
                                                  geometry_msgs::Pose& marker_pose)
{
  // rotate cam pose around z axis for -90 degrees
  tf::Quaternion cam_orientation;
  tf::quaternionMsgToTF(cam_pose_.orientation, cam_orientation);
  tf::Quaternion rot_around_z_neg_90_deg(0.0, 0.0, -0.707, 0.707);
  tf::quaternionTFToMsg(cam_orientation * rot_around_z_neg_90_deg, marker_pose.orientation);
  marker_pose.position = cam_pose_.position;
}

void TrajectoryEditor::appendCamPoseToTrajectory()
{
  if(!isCamWithinBounds())
    return;

  // rotate cam pose around z axis for -90 degrees
  geometry_msgs::Pose rotated_cam_pose;
  rvizCamToMarkerOrientation(cam_pose_, rotated_cam_pose);

  // create new marker
  visualization_msgs::InteractiveMarker new_marker = makeMarker();
  new_marker.pose.orientation.y = 0.0;
  new_marker.name = std::to_string((int)markers_.size());
  new_marker.description = std::to_string((int)markers_.size());

  // set cam pose as marker pose
  new_marker.pose = rotated_cam_pose;

  markers_.emplace_back(TimedMarker(std::move(new_marker), 0.5));

  colorizeMarkersRed();

  // set new marker as current marker
  setCurrentTo(markers_.back());

  updateServer(markers_);
  updateTrajectory();
}

void TrajectoryEditor::setCurrentPoseToCam()
{
  if(!isCamWithinBounds())
    return;

  // rotate cam pose around z axis for -90 degrees
  geometry_msgs::Pose rotated_cam_pose;
  rvizCamToMarkerOrientation(cam_pose_, rotated_cam_pose);

  // update marker pose
  getMarkerByName(current_marker_name_).marker.pose = rotated_cam_pose;
  server_->setPose(current_marker_name_, rotated_cam_pose, markers_.front().marker.header);
  updatePoseInGUI(rotated_cam_pose, getMarkerByName(current_marker_name_).transition_time);

  updateTrajectory();
}

void TrajectoryEditor::setMarkerFrames()
{
  for(auto& marker : markers_)
    marker.marker.header.frame_id = ui_.frame_line_edit->text().toStdString();

  updateServer(markers_);
  updateTrajectory();
}

void TrajectoryEditor::increaseMarkerScale()
{
  updateMarkerScales(1.1f);
}

void TrajectoryEditor::decreaseMarkerScale()
{
  updateMarkerScales(0.9f);
}

void TrajectoryEditor::updateMarkerScale(TimedMarker& marker, float scale_factor)
{
  marker.marker.scale *= scale_factor;
  marker.marker.controls[0].markers[0].scale.x *= scale_factor;
  marker.marker.controls[0].markers[0].scale.y *= scale_factor;
  marker.marker.controls[0].markers[0].scale.z *= scale_factor;
  marker.marker.controls[0].markers[1].scale.x *= scale_factor;
  marker.marker.controls[0].markers[1].scale.y *= scale_factor;
  marker.marker.controls[0].markers[1].scale.z *= scale_factor;
}

void TrajectoryEditor::updateMarkerScales(float scale_factor)
{
  for(auto& marker : markers_)
    updateMarkerScale(marker, scale_factor);

  updateServer(markers_);
}

void TrajectoryEditor::loadTrajectoryFromFile()
{
  std::string directory_path = ros::package::getPath("rqt_pose_interpolator")  + "/trajectories/";

  QString file_name = QFileDialog::getOpenFileName(widget_, "Open Trajectory", QString(directory_path.c_str()), "All Files (*);;.yaml files (*.yaml);;.txt files (*.txt)");
  if(file_name == "")
  {
    ROS_ERROR_STREAM("No file specified.");
    return;
  }

  std::string extension = boost::filesystem::extension(file_name.toStdString());

  if(extension == ".yaml")
  {
    YAML::Node trajectory = YAML::LoadFile(file_name.toStdString());
    int count = 0;
    markers_.clear();
    for(const auto& pose : trajectory["poses"])
    {
      visualization_msgs::InteractiveMarker wp_marker = makeMarker();
      wp_marker.controls[0].markers[0].color.r = 1.f;

      wp_marker.pose.orientation.w = pose["orientation"]["w"].as<double>();
      wp_marker.pose.orientation.x = pose["orientation"]["x"].as<double>();
      wp_marker.pose.orientation.y = pose["orientation"]["y"].as<double>();
      wp_marker.pose.orientation.z = pose["orientation"]["z"].as<double>();

      wp_marker.pose.position.x = pose["position"]["x"].as<double>();
      wp_marker.pose.position.y = pose["position"]["y"].as<double>();
      wp_marker.pose.position.z = pose["position"]["z"].as<double>();

      wp_marker.name = std::to_string(count);
      wp_marker.description = std::to_string(count);

      markers_.emplace_back(TimedMarker(std::move(wp_marker), pose["transition_time"].as<double>()));
      count++;
    }
  }
  else if(extension == ".txt")
  {
    int count = 0;
    markers_.clear();
    std::ifstream infile(file_name.toStdString());
    std::string line;
    double prev_pose_time = 0.0;
    while(std::getline(infile, line))
    {
      if(boost::starts_with(line, "#"))
        continue;

      std::vector<std::string> pose_strings;
      boost::split(pose_strings, line, boost::is_any_of(" "), boost::algorithm::token_compress_on);

      if(pose_strings.size() != 8)
      {
        ROS_ERROR_STREAM("Line: " << line << " contains the wrong number of parameters. Format is: timestamp tx ty tz qx qy qz qw. Number of parameters are " << (int)pose_strings.size());
        continue;
      }

      visualization_msgs::InteractiveMarker wp_marker = makeMarker();
      wp_marker.controls[0].markers[0].color.r = 1.f;

      double transition_time = 0.0;
      if(count > 0)
        transition_time = boost::lexical_cast<double>(pose_strings.at(0)) - prev_pose_time;

      wp_marker.pose.position.x = boost::lexical_cast<double>(pose_strings.at(1));
      wp_marker.pose.position.y = boost::lexical_cast<double>(pose_strings.at(2));
      wp_marker.pose.position.z = boost::lexical_cast<double>(pose_strings.at(3));

      wp_marker.pose.orientation.x = boost::lexical_cast<double>(pose_strings.at(4));
      wp_marker.pose.orientation.y = boost::lexical_cast<double>(pose_strings.at(5));
      wp_marker.pose.orientation.z = boost::lexical_cast<double>(pose_strings.at(6));
      wp_marker.pose.orientation.w = boost::lexical_cast<double>(pose_strings.at(7));

      wp_marker.name = std::to_string(count);
      wp_marker.description = std::to_string(count);

      markers_.emplace_back(TimedMarker(std::move(wp_marker), transition_time));

      prev_pose_time = boost::lexical_cast<double>(pose_strings.at(0));
      count++;
    }
  }
  else
  {
    ROS_ERROR_STREAM("Specified file is neither .yaml nor .txt file.\n File name is: " << file_name.toStdString());
    return;
  }

  // first marker is current marker
  markers_.front().marker.controls[0].markers[0].color.r = 0.f;
  markers_.front().marker.controls[0].markers[0].color.g = 1.f;
  current_marker_name_ = markers_.front().marker.name;

  server_->clear();
  updateServer(markers_);
  updatePoseInGUI(markers_.front().marker.pose, markers_.front().transition_time);
  updateTrajectory();
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
  marker.scale = 2.22f;
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = z;
  marker.pose.orientation.w = M_SQRT1_2;
  marker.pose.orientation.y = M_SQRT1_2;

  makeBoxControl(marker);

  visualization_msgs::InteractiveMarkerControl pose_control;
  pose_control.orientation.w = M_SQRT1_2;
  pose_control.orientation.x = M_SQRT1_2;
  pose_control.orientation.y = 0;
  pose_control.orientation.z = 0;
  pose_control.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;
  pose_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE;
  marker.controls.push_back(pose_control);

  pose_control.orientation.x = 0;
  pose_control.orientation.y = M_SQRT1_2;
  pose_control.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;
  pose_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE;
  marker.controls.push_back(pose_control);

  pose_control.orientation.y = 0;
  pose_control.orientation.z = M_SQRT1_2;
  pose_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE;
  marker.controls.push_back(pose_control);
  return marker;
}

void TrajectoryEditor::colorizeMarkersRed()
{
  for(auto& marker : markers_)
  {
    marker.marker.controls[0].markers[0].color.r = 1.f;
    marker.marker.controls[0].markers[0].color.g = 0.f;
  }
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
  moveCamToMarker(current_marker_name_);
}

void TrajectoryEditor::moveCamToFirst()
{
  // do nothing if current is already the first marker
  if(markers_.begin()->marker.name == current_marker_name_)
    return;

  // find current marker
  auto it = std::next(markers_.begin());
  for(; it != markers_.end(); ++it)
    if(it->marker.name == current_marker_name_)
      break;

  // fill Camera Trajectory msg with markers and times
  rviz_animated_view_controller::CameraTrajectoryPtr cam_trajectory(new rviz_animated_view_controller::CameraTrajectory());
  cam_trajectory->target_frame = ui_.frame_line_edit->text().toStdString();
  cam_trajectory->allow_free_yaw_axis = !ui_.use_up_of_world_check_box->isChecked();

  if(ui_.splines_check_box->isChecked())
  {
    MarkerList markers;
    auto current = it;
    // used spline type uses first an last point but doesn't interpolate between them
    // therefore we add the marker before the current - or itself if there is none before
    if(current == std::prev(markers_.end()))
      markers.push_back(*current);
    else
      markers.push_back(*std::next(current));

    // then we add all other markers
    do
    {
      markers.push_back(*current);
    }
    while(current-- != markers_.begin());
    // and the last one a second time
    markers.push_back(*(markers_.begin()));

    markersToSplinedCamTrajectory(markers,
                                  cam_trajectory,
                                  ui_.publish_rate_spin_box->value(),
                                  ui_.smooth_velocity_check_box->isChecked());
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
  // do nothing if current is already the first marker
  if(markers_.begin()->marker.name == current_marker_name_)
    return;

  // find current marker
  auto it = std::next(markers_.begin()), prev_marker = markers_.begin();
  for(; it != markers_.end(); ++it, ++prev_marker)
    if(it->marker.name == current_marker_name_)
      break;

  setCurrentFromTo(*it, *prev_marker);

  moveCamToMarker(current_marker_name_);
}

void TrajectoryEditor::moveCamToNext()
{
  // do nothing if current is already the last marker
  if(std::prev(markers_.end())->marker.name == current_marker_name_)
    return;

  // find iterator to current marker
  auto it = markers_.begin(), next_marker = std::next(markers_.begin());
  for(; it != markers_.end(); ++it, ++next_marker)
    if(it->marker.name == current_marker_name_)
      break;

  setCurrentFromTo(*it, *next_marker);

  moveCamToMarker(current_marker_name_);
}

void TrajectoryEditor::moveCamToLast()
{
  // do nothing if current is already the last marker
  if(std::prev(markers_.end())->marker.name == current_marker_name_)
    return;

  // find current marker
  auto it = markers_.begin();
  for(; it != markers_.end(); ++it)
    if(it->marker.name == current_marker_name_)
      break;

  // fill Camera Trajectory msg with markers and times
  rviz_animated_view_controller::CameraTrajectoryPtr cam_trajectory(new rviz_animated_view_controller::CameraTrajectory());
  cam_trajectory->target_frame = ui_.frame_line_edit->text().toStdString();
  cam_trajectory->allow_free_yaw_axis = !ui_.use_up_of_world_check_box->isChecked();

  if(ui_.splines_check_box->isChecked())
  {
    MarkerList markers;
    auto current = it;
    // used spline type uses first an last point but doesn't interpolate between them
    // therefore we add the marker before the current - or itself if there is none before
    if(current == markers_.begin())
      markers.push_back(*current);
    else
      markers.push_back(*std::prev(current));

    // then we add all other markers
    for(; current != markers_.end(); current++)
      markers.push_back(*current);

    // and the last one a second time
    markers.push_back(*(std::prev(markers_.end())));

    markersToSplinedCamTrajectory(markers,
                                  cam_trajectory,
                                  ui_.publish_rate_spin_box->value(),
                                  ui_.smooth_velocity_check_box->isChecked());
  }
  else
  {
    rviz_animated_view_controller::CameraMovement cam_movement;
    bool first = true;
    auto next = it;
    for(++next; next != markers_.end(); next++)
    {
      convertMarkerToCamMovement(*next, cam_movement);

      // set interpolation speed first rising, then full speed, then declining
      cam_movement.interpolation_speed = (first) ? rviz_animated_view_controller::CameraMovement::RISING : rviz_animated_view_controller::CameraMovement::FULL;
      if(next == std::prev(markers_.end()))
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
  }

  setCurrentFromTo(*it, *(std::prev(markers_.end())));

  // publish cam trajectory
  camera_trajectory_pub_.publish(cam_trajectory);
}

void TrajectoryEditor::moveCamToMarker(const std::string& marker_name)
{
  TrajectoryEditor::TimedMarker marker = getMarkerByName(marker_name);

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
    current_marker_name_ = feedback->marker_name;

    updatePoseInGUI(feedback->pose, getMarkerByName(feedback->marker_name).transition_time);

    // update marker pose
    marker.pose = feedback->pose;
    getMarkerByName(feedback->marker_name).marker.pose = feedback->pose;

    colorizeMarkersRed();
    // change color of current marker to green
    getMarkerByName(feedback->marker_name).marker.controls[0].markers[0].color.r = 0.f;
    getMarkerByName(feedback->marker_name).marker.controls[0].markers[0].color.g = 1.f;

    server_->clear();
    updateServer(markers_);

    updateTrajectory();
  }
}

void TrajectoryEditor::updateCurrentMarker()
{
  geometry_msgs::Pose pose;
  pose.position.x = ui_.translation_x_spin_box->value();
  pose.position.y = ui_.translation_y_spin_box->value();
  pose.position.z = ui_.translation_z_spin_box->value();

  pose.orientation.x = ui_.rotation_x_spin_box->value();
  pose.orientation.y = ui_.rotation_y_spin_box->value();
  pose.orientation.z = ui_.rotation_z_spin_box->value();
  pose.orientation.w = ui_.rotation_w_spin_box->value();

  getMarkerByName(current_marker_name_).marker.pose = pose;
  getMarkerByName(current_marker_name_).transition_time = ui_.transition_time_spin_box->value();

  server_->setPose(current_marker_name_, pose, markers_.front().marker.header);
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

void TrajectoryEditor::markersToSplinedCamTrajectory(const MarkerList& markers,
                                                     rviz_animated_view_controller::CameraTrajectoryPtr trajectory,
                                                     double frequency,
                                                     bool smooth_velocity)
{
  // put all markers positions into a vector. First and last double.
  double total_transition_time = 0.0;
  std::vector<Vector3> input_eye_positions;
  std::vector<Vector3> input_focus_positions;
  std::vector<Vector3> input_up_directions;
  std::vector<double> transition_times;
  Vector3 position;
  bool first = true;
  std::string prev_marker_name;
  for(const auto& marker : markers)
  {
    position[0] = static_cast<float>(marker.marker.pose.position.x);
    position[1] = static_cast<float>(marker.marker.pose.position.y);
    position[2] = static_cast<float>(marker.marker.pose.position.z);
    input_eye_positions.push_back(position);

    tf::Vector3 rotated_vector = rotateVector(tf::Vector3(0, 0, -ui_.smoothness_spin_box->value()), marker.marker.pose.orientation);
    position[0] = position[0] + static_cast<float>(rotated_vector.x());
    position[1] = position[1] + static_cast<float>(rotated_vector.y());
    position[2] = position[2] + static_cast<float>(rotated_vector.z());
    input_focus_positions.push_back(position);

    if(!ui_.use_up_of_world_check_box->isChecked())
    {
      // in the cam frame up is the negative x direction
      tf::Vector3 rotated_vector = rotateVector(tf::Vector3(-1, 0, 0), marker.marker.pose.orientation);
      position[0] = static_cast<float>(rotated_vector.x());
      position[1] = static_cast<float>(rotated_vector.y());
      position[2] = static_cast<float>(rotated_vector.z());
    }
    else
    {
      position[0] = 0;
      position[1] = 0;
      position[2] = 1;
    }
    input_up_directions.push_back(position);


    // first because - when moving from marker A to B we only consider the transition time of B
    // check for equal names because - UniformCRSpline needs to be fed with start marker two times (end as well)
    // this case needs to be handled to prevent errors
    if(!first && prev_marker_name != marker.marker.name)
    {
      if(smooth_velocity)
        total_transition_time += marker.transition_time;
      else
        transition_times.push_back(marker.transition_time / frequency);
    }

    first = false;
    prev_marker_name = marker.marker.name;
  }

  UniformCRSpline<Vector3> eye_spline(input_eye_positions);
  UniformCRSpline<Vector3> focus_spline(input_focus_positions);
  UniformCRSpline<Vector3> up_spline(input_up_directions);

  // rate to sample from spline and get points
  rviz_animated_view_controller::CameraMovement cam_movement = makeCameraMovement();
  double rate = 1.0 / frequency;
  float max_t = eye_spline.getMaxT();
  double total_length = eye_spline.totalLength();
  first = true;
  bool last_run = false;
  for(double t = 0.f; t <= max_t;)
  {
    // get position of spline
    auto interpolated_position = eye_spline.getPosition(t);
    auto interpolated_focus = focus_spline.getPosition(t);
    auto interpolated_up = up_spline.getPosition(t);

    cam_movement.eye.point.x = interpolated_position[0];
    cam_movement.eye.point.y = interpolated_position[1];
    cam_movement.eye.point.z = interpolated_position[2];
    cam_movement.focus.point.x = interpolated_focus[0];
    cam_movement.focus.point.y = interpolated_focus[1];
    cam_movement.focus.point.z = interpolated_focus[2];

    if(!ui_.use_up_of_world_check_box->isChecked())
    {
      cam_movement.up.vector.x = interpolated_up[0];
      cam_movement.up.vector.y = interpolated_up[1];
      cam_movement.up.vector.z = interpolated_up[2];
    }
    // else is not necessary - up is already set to default in makeCameraMovement

    cam_movement.interpolation_speed = (first) ? rviz_animated_view_controller::CameraMovement::RISING : rviz_animated_view_controller::CameraMovement::FULL;

    double transition_time = 0.0;
    if(smooth_velocity)
    {
      double local_length = eye_spline.arcLength(std::max(t - rate, 0.0), t);
      transition_time = total_transition_time * local_length / total_length;
    }
    else
      transition_time = transition_times[(int)std::floor(std::max(t - rate, 0.0))];

    cam_movement.transition_time = ros::Duration(transition_time);

    trajectory->trajectory.push_back(cam_movement);

    ROS_DEBUG_STREAM("t " << t << " max_t " << max_t);

    if(last_run)
      break;

    t += rate;
    if(t > max_t)
    {
      last_run = true;
      t = max_t;
    }

    first = false;
  }
}

void TrajectoryEditor::markersToSplinedPoses(const MarkerList& markers,
                                             std::vector<geometry_msgs::Pose>& spline_poses,
                                             double frequency,
                                             bool duplicate_ends)
{
  // put all markers positions into a vector. First and last double.
  std::vector<Vector3> spline_points;
  Vector3 position;
  bool first = true;
  for(const auto& marker : markers)
  {
    position[0] = static_cast<float>(marker.marker.pose.position.x);
    position[1] = static_cast<float>(marker.marker.pose.position.y);
    position[2] = static_cast<float>(marker.marker.pose.position.z);
    spline_points.push_back(position);

    if(first && duplicate_ends)
      spline_points.push_back(position);

    first = false;
  }
  if(duplicate_ends)
    spline_points.push_back(position);

  UniformCRSpline<Vector3> spline(spline_points);

  // rate to sample from spline and get points
  double rate = 1.0 / frequency;
  float max_t = spline.getMaxT();
  auto current_marker = markers.begin();
  auto next_marker = std::next(markers.begin());
  int current_marker_id = 0;
  bool last_run = false;
  for(double i = 0.f; i <= max_t;)
  {
    // get position of spline
    auto interpolated_position = spline.getPosition(i);
    geometry_msgs::Pose pose;
    pose.position.x = interpolated_position[0];
    pose.position.y = interpolated_position[1];
    pose.position.z = interpolated_position[2];

    // i from 0 to 1 corresponds to the spline between the first and the second marker
    // we have to maintain iterators for slerp
    if(current_marker_id != (int)std::floor(i) && !last_run)
    {
      current_marker_id++;
      current_marker++;
      next_marker++;
    }

    // get slerped orientation
    tf::Quaternion start_orientation, end_orientation, intermediate_orientation;
    tf::quaternionMsgToTF(current_marker->marker.pose.orientation, start_orientation);
    tf::quaternionMsgToTF(next_marker->marker.pose.orientation, end_orientation);
    double slerp_factor = fmod(i, 1.0);
    if(last_run)
      slerp_factor = 1.0;
    intermediate_orientation = start_orientation.slerp(end_orientation, slerp_factor);
    tf::quaternionTFToMsg(intermediate_orientation, pose.orientation);

    spline_poses.push_back(pose);

    if(last_run)
      break;

    i += rate;
    if(i >= max_t)
    {
      last_run = true;
      i = max_t;
    }
  }
}

} // namespace
