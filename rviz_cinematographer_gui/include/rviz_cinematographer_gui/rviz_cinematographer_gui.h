/** @file
 *
 * Simple rqt plugin to edit trajectories.
 *
 * @author Jan Razlaw
 */

#ifndef RVIZ_CINEMATOGRAPHER_GUI_H
#define RVIZ_CINEMATOGRAPHER_GUI_H

#include <fstream>
#include <sstream>
#include <string>
#include <unistd.h>
#include <signal.h>

#include <ros/ros.h>
#include <ros/package.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>

#include <rviz_cinematographer_msgs/CameraMovement.h>
#include <rviz_cinematographer_msgs/CameraTrajectory.h>
#include <rviz_cinematographer_msgs/Record.h>
#include <rviz_cinematographer_msgs/Finished.h>

#include <std_msgs/Empty.h>

#include <nav_msgs/Path.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <rqt_gui_cpp/plugin.h>

#include <QWidget>
#include <QFileDialog>

#include <rviz_cinematographer_gui/utils.h>
#include <ui_rviz_cinematographer_gui.h>

#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/thread.hpp>
#include <yaml-cpp/yaml.h>

#include <spline_library/splines/natural_spline.h>
#include <spline_library/splines/uniform_cr_spline.h>
#include <spline_library/vector.h>


namespace rviz_cinematographer_gui
{

/**
 * @brief Manipulates the rviz camera.
 */
class RvizCinematographerGUI : public rqt_gui_cpp::Plugin
{

Q_OBJECT
public:
  struct InteractiveMarkerWithTime
  {
    InteractiveMarkerWithTime(visualization_msgs::InteractiveMarker&& input_marker,
                              const double transition_time,
                              const double wait_time = 0.0)
      : marker(input_marker)
        , transition_time(transition_time)
        , wait_time(wait_time)
    {
    }

    visualization_msgs::InteractiveMarker marker;
    double transition_time;
    double wait_time;
  };

  typedef InteractiveMarkerWithTime TimedMarker;
  typedef std::list<TimedMarker> MarkerList;


  /** @brief Constructor. */
  RvizCinematographerGUI();
  ~RvizCinematographerGUI() = default;

  /**
   * @brief Sets up subscribers and publishers and connects GUI to functions.
   *
   * @param context     the plugin context.
   */
  virtual void initPlugin(qt_gui_cpp::PluginContext& context);

  /**
   * @brief Shuts down the subscribers and publishers.
   */
  virtual void shutdownPlugin();

  /**
   * @brief Saves settings. TODO.
   *
   * @param plugin_settings     plugin-specific settings
   * @param instance_settings   instance-specific settings
   */
  virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings,
                            qt_gui_cpp::Settings& instance_settings) const;

  /**
   * @brief Restores settings. TODO.
   *
   * @param plugin_settings     plugin-specific settings
   * @param instance_settings   instance-specific settings
   */
  virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
                               const qt_gui_cpp::Settings& instance_settings);

  /**
   * @brief Saves the current camera pose to #cam_pose_.
   *
   * @param[in] cam_pose    pointer to current camera pose.
   */
  void camPoseCallback(const geometry_msgs::Pose::ConstPtr& cam_pose);

Q_SIGNALS:
  void updateRequested();

public slots:
  /** @brief Moves rviz camera to currently selected pose.*/
  void moveCamToCurrent();
  /** @brief Moves rviz camera to the pose before the selected one.*/
  void moveCamToPrev();
  /** @brief Moves rviz camera subsequently to the first pose in the trajectory.*/
  void moveCamToFirst();
  /** @brief Moves rviz camera to the pose after the selected one.*/
  void moveCamToNext();
  /** @brief Moves rviz camera subsequently to the last pose in the trajectory.*/
  void moveCamToLast();
  /** @brief Update selected marker with values from GUI.*/
  void updateCurrentMarker();
  /** @brief Appends the current pose of the rviz camera to the trajectory.*/
  void appendCamPoseToTrajectory();
  /** @brief Sets selected pose to the current pose of the rviz camera.*/
  void setCurrentPoseToCam();
  /** @brief Reconstructs trajectory from current markers. */
  void updateTrajectory();
  /** @brief Sets the frame_id of the markers.*/
  void setMarkerFrames();
  /** @brief Increase the scale of the markers.*/
  void increaseMarkerScale();
  /** @brief Decrease the scale of the markers.*/
  void decreaseMarkerScale();
  /** @brief Show/Hide interactive marker controls.*/
  void showInteractiveMarkerControls();
  /** @brief Loads a series of markers from a file.*/
  void loadTrajectoryFromFile();
  /** @brief Saves poses and transition times of interactive markers to a file.*/
  void saveTrajectoryToFile();
  /** @brief Opens file explorer to specify the path of the recorded video.*/
  void setVideoOutputPath();

private:
  /**
   * @brief Creates a CameraMovement hull.
   * @return CameraMovement.
   */
  rviz_cinematographer_msgs::CameraMovement makeCameraMovement();

  /**
   * @brief Creates an InteractiveMarker hull.
   *
   * @param[in] x   x position of marker.
   * @param[in] y   y position of marker.
   * @param[in] z   z position of marker.
   * @return InteractiveMarker.
   */
  visualization_msgs::InteractiveMarker makeMarker(double x = 0.0,
                                                   double y = 0.0,
                                                   double z = 0.0);

  /**
   * @brief Colorize all markers in red.
   */
  void colorizeMarkersRed();

  /** @brief Updates the scale of the provided marker.
   *
   * @param[in, out]    marker          marker that is updated
   * @param[in]         scale_factor    factor used to update the marker scale
   */
  void updateMarkerScale(TimedMarker& marker,
                         float scale_factor);

  /** @brief Updates the scales of the markers.
   *
   * @param[in]     scale_factor   factor used to update the markers' scales
   */
  void updateMarkerScales(float scale_factor);

  /**
   * @brief Checks if current position of rviz camera is within bounds of spin boxes.
   *
   * @return true, if camera is within bounds.
   */
  bool isCamWithinBounds();

  /**
   * @brief Rotates rviz_cam_pose around z-axis for -90 degrees.
   *
   * @param[in]     rviz_cam_pose   camera orientation as defined by rviz.
   * @param[out]    marker_pose     camera orientation as defined by markers.
   */
  void rvizCamToMarkerOrientation(const geometry_msgs::Pose& rviz_cam_pose,
                                  geometry_msgs::Pose& marker_pose);

  /**
   * @brief Fills a CameraMovement message with the values of a TimedMarker.
   *
   * @param[in]     marker          marker.
   * @param[out]    cam_movement    message.
   */
  void convertMarkerToCamMovement(const TimedMarker& marker,
                                  rviz_cinematographer_msgs::CameraMovement& cam_movement);

  /**
   * @brief Moves rviz camera to marker pose by publishing a CameraTrajectory message.
   *
   * @param[in] marker_name         name of marker.
   * @param[in] transition_time     (optional) provide time needed to get to marker - default: transition_time of marker.
   */
  void moveCamToMarker(const std::string& marker_name,
                       double transition_time = -1.0);

  /**
   * @brief Updates members using pose of currently moved interactive marker.
   *
   * @param[in] feedback    feedback the interaction with the interactive marker generates.
   */
  void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);

  /**
   * @brief Rotates a vector by a quaternion.
   *
   * @param[in] vector  input vector.
   * @param[in] quat    input rotation.
   * @return the rotated vector.
   */
  tf::Vector3 rotateVector(const tf::Vector3& vector,
                           const geometry_msgs::Quaternion& quat);

  /**
   * @brief Safes poses and times of markers to yaml file.
   *
   * @param[in] file_path   path to the file.
   */
  void safeTrajectoryToFile(const std::string& file_path);

  /**
   * @brief Adds a marker between the selected marker and the one before in the trajectory.
   *
   * @param[in] feedback    feedback from selected marker.
   */
  void addMarkerBefore(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);

  /**
   * @brief Adds a marker at the pose of the selected marker - useful to define pauses.
   *
   * @param[in] feedback    feedback from selected marker.
   */
  void addMarkerHere(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);

  /**
   * @brief Adds a marker between the selected marker and the next one in the trajectory.
   *
   * @param[in] feedback    feedback from selected marker.
   */
  void addMarkerBehind(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);

  /**
   * @brief Removes the selected marker.
   *
   * @param[in] feedback    feedback from selected marker.
   */
  void removeClickedMarker(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);

  /**
   * @brief Removes the marker called marker_name.
   *
   * @param[in] marker_name    name of marker that should be removed.
   */
  void removeMarker(const std::string& marker_name);

  /**
   * @brief Removes the current marker.
   *
   * @param[in] empty    empty msgs needed as this is a callback function.
   */
  void removeCurrentMarker(const std_msgs::EmptyConstPtr& empty);

  /**
   * @brief Saves markers in server.
   *
   * @param[in] markers     markers.
   */
  void updateServer(MarkerList& markers);

  /**
   * @brief Load marker poses from a file.
   *
   * @param[in] nh          node handle.
   * @param[in] param_name  root name of the parameters.
   */
  void loadParams(const ros::NodeHandle& nh,
                  const std::string& param_name);

  /**
   * @brief Gets marker with specified name.
   *
   * @param[in] marker_name name of marker.
   * @return marker with marker_name.
   */
  InteractiveMarkerWithTime& getMarkerByName(const std::string& marker_name);

  /**
   * @brief Sets the #current_marker_ to the provided input.
   *
   * Additionally updates the GUI elements and sets the color of the input marker to green.
   *
   * @param[in] marker  input marker.
   */
  void setCurrentTo(TimedMarker& marker);

  /**
   * @brief Resets the #current_marker_ and dependencies from old_current to new_current.
   *
   * Dependencies are the marker member, the marker server and the GUI.
   *
   * @param[in,out] old_current  marker that was current before.
   * @param[in,out] new_current  marker that is current now.
   */
  void setCurrentFromTo(TimedMarker& old_current,
                        TimedMarker& new_current);

  /**
   * @brief Sets the pose in the GUI to the provided input values.
   *
   * @param[in] pose            pose
   * @param[in] transition_time transition time
   */
  void updatePoseInGUI(const geometry_msgs::Pose& pose,
                       double transition_time = 0.5);

  /**
   * @brief Sets the value of the spin_box to the value without triggering a signal.
   *
   * @param[in,out] spin_box    the updated spin box.
   * @param[in]     value       the value the spin box is set to.
   */
  void setValueQuietly(QDoubleSpinBox* spin_box,
                       double value);

  /**
   * @brief Interpolate markers using a spline and safe that spline in spline_poses.
   *
   * @param[in]     markers         markers to be interpolated.
   * @param[out]    spline_poses    constructed spline.
   * @param[in]     frequency       resolution - number of spline points between each marker pair.
   * @param[in]     duplicate_ends  flag if first and last marker should be duplicated for spline - some types of splines don't interpolate between the the end points and the ones next to them.
   */
  void markersToSplinedPoses(const MarkerList& markers,
                             std::vector<geometry_msgs::Pose>& spline_poses,
                             double frequency,
                             bool duplicate_ends = true);

  /**
   * @brief Interpolate markers using a spline and safe that spline as the points of a CameraTrajectory.
   *
   * @param[in]     markers         markers to be interpolated.
   * @param[out]    trajectory      resulting trajectory.
   */
  void markersToSplinedCamTrajectory(const MarkerList& markers,
                                     rviz_cinematographer_msgs::CameraTrajectoryPtr trajectory);

  /**
   * @brief Generates trajectories for eye positions, focus positions and up directories, needed for spline generation.
   *
   * @param[in]     markers                   markers defining trajectory.
   * @param[out]    input_eye_positions       camera positions at trajectory points.
   * @param[out]    input_focus_positions     positions of camera focus at trajectory points.
   * @param[out]    input_up_directions       cameras up directions at trajectory points.
   */
  void prepareSpline(const MarkerList& markers,
                     std::vector<Vector3>& input_eye_positions,
                     std::vector<Vector3>& input_focus_positions,
                     std::vector<Vector3>& input_up_directions);

  /**
   * @brief Computes transition times for each step within the spline.
   *
   * @param[in]     markers                 markers defining trajectory.
   * @param[out]    transition_times        transition times between spline points.
   * @param[out]    total_transition_time   sum of all transition times.
   */
  void computeTransitionTimes(const MarkerList& markers,
                              std::vector<double>& transition_times,
                              double& total_transition_time);

  /**
   * @brief Convert spline to CameraTrajectory.
   *
   * @param[in]     eye_spline              spline of camera positions.
   * @param[in]     focus_spline            spline of camera focus points.
   * @param[in]     up_spline               spline of camera up positions.
   * @param[in]     transition_times        transition time between spline points.
   * @param[in]     total_transition_time   overall transition time.
   * @param[out]    trajectory              resulting camera trajectory.
   */
  void splineToCamTrajectory(const UniformCRSpline<Vector3>& eye_spline,
                             const UniformCRSpline<Vector3>& focus_spline,
                             const UniformCRSpline<Vector3>& up_spline,
                             const std::vector<double>& transition_times,
                             const double total_transition_time, 
                             rviz_cinematographer_msgs::CameraTrajectoryPtr trajectory);

  /** @brief Call service to record current trajectory. */
  void publishRecordParams();

  /** @brief Listen to the message that the recording is over. */
  void recordFinishedCallback(const rviz_cinematographer_msgs::Finished::ConstPtr& record_finished);

  /** @brief Starts video recorder nodelet. */
  void videoRecorderThread();

  /** @brief Ui object - connection to GUI. */
  Ui::rviz_cinematographer_gui ui_;
  /** @brief Widget. */
  QWidget* widget_;

  /** @brief Publishes camera trajectory messages. */
  ros::Publisher camera_trajectory_pub_;
  /** @brief Publishes the trajectory that is defined by the markers. */
  ros::Publisher view_poses_array_pub_;
  /** @brief Publishes the parameters for a recording. */
  ros::Publisher record_params_pub_;

  /** @brief Subscribes to the camera pose. */
  ros::Subscriber camera_pose_sub_;
  /** @brief Subscribes to listen when the recording is over. */
  ros::Subscriber record_finished_sub_;
  /** @brief Subscribes to delete marker msgs. */
  ros::Subscriber delete_marker_sub_;

  /** @brief Starts video recorder nodelet. */
  boost::shared_ptr<boost::thread> video_recorder_thread_;

  /** @brief Connects markers to callbacks. */
  interactive_markers::MenuHandler menu_handler_;
  /** @brief Stores markers - needed for #menu_handler. */
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;

  /** @brief Current camera pose. */
  geometry_msgs::Pose cam_pose_;

  /** @brief Name of currently selected marker. */
  std::string current_marker_name_;

  /** @brief Currently maintained list of TimedMarkers. */
  MarkerList markers_;

  /** @brief True if recorder was destructed. */
  bool recorder_running_;
};

} // namespace

#endif //RVIZ_CINEMATOGRAPHER_GUI_H
