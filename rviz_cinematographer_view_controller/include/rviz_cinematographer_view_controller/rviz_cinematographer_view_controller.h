/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Original Author: Adam Leeper
 * Adapted by Jan Razlaw
 */

#ifndef RVIZ_CINEMATOGRAPHER_VIEW_CONTROLLER_H
#define RVIZ_CINEMATOGRAPHER_VIEW_CONTROLLER_H

#include "rviz/view_controller.h"
#include "rviz/view_manager.h"
#include "rviz/render_panel.h"
#include "rviz/load_resource.h"
#include "rviz/uniform_string_stream.h"
#include "rviz/display_context.h"
#include "rviz/viewport_mouse_event.h"
#include "rviz/frame_manager.h"
#include "rviz/geometry.h"
#include "rviz/ogre_helpers/shape.h"
#include "rviz/properties/float_property.h"
#include "rviz/properties/vector_property.h"
#include "rviz/properties/bool_property.h"
#include "rviz/properties/tf_frame_property.h"
#include "rviz/properties/editable_enum_property.h"
#include "rviz/properties/ros_topic_property.h"

#include <ros/subscriber.h>
#include <ros/ros.h>
#include <ros/package.h>

#include <rviz_cinematographer_msgs/CameraMovement.h>
#include <rviz_cinematographer_msgs/CameraTrajectory.h>
#include <rviz_cinematographer_msgs/Record.h>
#include <rviz_cinematographer_msgs/Finished.h>
#include <rviz_cinematographer_msgs/Wait.h>
#include <std_msgs/Empty.h>

#include <nav_msgs/Odometry.h>

#include <OGRE/OgreVector3.h>
#include <OGRE/OgreQuaternion.h>
#include <OGRE/OgreRenderWindow.h>
#include <OGRE/OgreViewport.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreCamera.h>

#include <boost/circular_buffer.hpp>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

namespace rviz {
  class SceneNode;
  class Shape;
  class BoolProperty;
  class FloatProperty;
  class VectorProperty;
  class TfFrameProperty;
  class EditableEnumProperty;
  class RosTopicProperty;
}

namespace rviz_cinematographer_view_controller
{

/** @brief An un-constrained "flying" camera, specified by an eye point, focus point, and up vector. */
class CinematographerViewController : public rviz::ViewController
{
Q_OBJECT
public:

  struct OgreCameraMovement
  {
    OgreCameraMovement(){};

    OgreCameraMovement(const Ogre::Vector3& eye,
                       const Ogre::Vector3& focus,
                       const Ogre::Vector3& up,
                       const ros::Duration& transition_duration,
                       const uint8_t interpolation_speed)
      : eye(eye)
        , focus(focus)
        , up(up)
        , transition_duration(transition_duration)
        , interpolation_speed(interpolation_speed)
    {
    }

    Ogre::Vector3 eye;
    Ogre::Vector3 focus;
    Ogre::Vector3 up;

    ros::Duration transition_duration;
    uint8_t interpolation_speed;
  };

  typedef boost::circular_buffer<OgreCameraMovement> BufferCamMovements;

  CinematographerViewController();
  virtual ~CinematographerViewController();

  /** @brief Do subclass-specific initialization. Called by
   * ViewController::initialize after context_ and camera_ are set.
   *
   * This version sets up the attached_scene_node, focus shape, and camera movements buffer. */
  void onInitialize() override;

  /** @brief Called by activate(). */
  void onActivate() override;

  /** @brief Applies a translation to the focus and eye points.
   *
   * @param[in] x   x-part of translation.
   * @param[in] y   y-part of translation.
   * @param[in] z   z-part of translation.
   */
  void move_focus_and_eye(float x,
                          float y,
                          float z);

  /** @brief Applies a translation to only the eye point.
   *
   * @param[in] x   x-part of translation.
   * @param[in] y   y-part of translation.
   * @param[in] z   z-part of translation.
   */
  void move_eye(float x,
                float y,
                float z);

  /** @brief Applies a body-fixed-axes sequence of rotations; only accurate for small angles.
   *
   * @param[in] yaw   yaw-part of rotation.
   * @param[in] pitch pitch-part of rotation.
   * @param[in] roll  roll-part of rotation.
   */
  void yaw_pitch_roll(float yaw,
                      float pitch,
                      float roll);

  /** @brief Handles user mouse events.
   *
   * @param[in] evt     the event that occured.
   */
  void handleMouseEvent(rviz::ViewportMouseEvent& evt) override;

  /** @brief Handles keyboard events.
   *
   * Sends a msg that the delete button was pressed.
   *
   * @param[in] event     the event that occured.
   */
  void handleKeyEvent(QKeyEvent* event, rviz::RenderPanel* panel) override;

  /** @brief Move the focus point to the point provided.
   *
   * Point is assumed to be in the Rviz Fixed Frame.
   *
   * @param[in] point   new focus point.
   */
  void lookAt(const Ogre::Vector3& point) override;

  /** @brief Publishes the current camera pose as an Odometry msg.
   *
   * @param[in] position    position of to be published odometry msg.
   * @param[in] velocity    velocity of to be published odometry msg.
   */
  void publishOdometry(const Ogre::Vector3& position,
                       const Ogre::Vector3& velocity);

  /** @brief Resets the camera parameters to a sane value. */
  void reset() override;

  /** @brief Configure the settings of this view controller to give,
   * as much as possible, a similar view as that given by the source_view.
   *
   * @param[in] source_view     must return a valid Ogre::Camera* from getCamera(). */
  void mimic(ViewController* source_view) override;

  /** @brief Called by ViewManager when this ViewController is being made current.
   *
   * This gives ViewController subclasses an opportunity to implement
   * a smooth transition from a previous viewpoint to the new viewpoint.
   *
   * @param[in] previous_view   the previous "current" view, and will not be NULL.
   */
  void transitionFrom(ViewController* previous_view) override;


protected Q_SLOTS:
  /** @brief Called when Target Frame property changes while view is active.
   *
   * Purpose is to change values in the view controller (like
   * a position offset) such that the actual viewpoint does not
   * change. Calls updateTargetSceneNode() and onTargetFrameChanged(). */
  virtual void updateAttachedFrame();

  /** @brief Called when distance property is changed; computes new eye position. */
  virtual void onDistancePropertyChanged();

  /** @brief Called focus property is changed; computes new distance. */
  virtual void onFocusPropertyChanged();

  /** @brief Called when eye property is changed; computes new distance. */
  virtual void onEyePropertyChanged();

  /** @brief Called when up vector property is changed; sets according camera parameter. */
  virtual void onUpPropertyChanged();

  /** @brief Called when camera trajectory topic is changed; updates subscriber to camera trajectories. */
  void updateTopics();

protected:  //methods
  /** @brief Called at 30Hz by ViewManager::update() while this view is active.
   *
   * Performs camera motion if required.
   *
   * @param[in] dt      time difference to last call of update function.
   * @param[in] ros_dt  time difference in ros time to last call of update function.
   */
  void update(float dt, float ros_dt) override;

  /** @brief Connects the signals/slots for position properties. */
  void connectPositionProperties();

  /** @brief Disconnects the signals/slots for position properties. */
  void disconnectPositionProperties();

  /** @brief Changes properties to nullify the change in attached frame.
   *
   * @param[in] old_reference_position      position in old frame.
   * @param[in] old_reference_orientation   orientation in old frame.
   *
   * @see updateAttachedFrame()
   */
  virtual void onAttachedFrameChanged(const Ogre::Vector3& old_reference_position,
                                      const Ogre::Quaternion& old_reference_orientation);

  /** @brief Update the position of the attached_scene_node_ in the current frame. */
  void updateAttachedSceneNode();

  /** @brief Initiate camera motion from incoming CameraTrajectory.
   *
   * @param[in] ct_ptr  incoming CameraTrajectory msg.
   */
  void cameraTrajectoryCallback(const rviz_cinematographer_msgs::CameraTrajectoryConstPtr& ct_ptr);

  /** @brief Transforms the camera movement into the attached frame.
   *
   * @param[in,out] cm  camera movement that should be transformed into attached frame.
   */
  void transformCameraMovementToAttachedFrame(rviz_cinematographer_msgs::CameraMovement& cm);

  /** @brief Set eye, focus and up property from provided source_camera.
   *
   * @param[in] source_camera   camera to take the parameters from.
   */
  void setPropertiesFromCamera(Ogre::Camera* source_camera);

  /** @brief Begins a camera movement animation to the given goal points.
   *
   * @param[in] eye                     position of camera
   * @param[in] focus                   focus point of camera
   * @param[in] up                      vector of camera pointing up
   * @param[in] transition_duration     duration needed for transition
   * @param[in] interpolation_speed     the interpolation speed profile
   */
  void beginNewTransition(const Ogre::Vector3& eye,
                          const Ogre::Vector3& focus,
                          const Ogre::Vector3& up,
                          ros::Duration transition_duration,
                          uint8_t interpolation_speed = rviz_cinematographer_msgs::CameraMovement::WAVE);

  /** @brief Cancels any currently active camera movement. */
  void cancelTransition();

  /** @brief Updates the Ogre camera properties from the view controller properties. */
  void updateCamera();

  /** @brief Publishes the camera pose. */
  void publishCameraPose();

  /** @brief Sets parameters requested with service call.
   *
   * @params[in] record_params  specifies that a record should be made and the parameters that should be used.
   */
  void setRecord(const rviz_cinematographer_msgs::Record::ConstPtr& record_params);

  /** @brief Sets the duration the rendering has to wait for.
   *
   * @params[in] wait_duration  duration to wait for.
   */
  void setWaitDuration(const rviz_cinematographer_msgs::Wait::ConstPtr& wait_duration);

  Ogre::Vector3 fixedFrameToAttachedLocal(const Ogre::Vector3& v) { return reference_orientation_.Inverse() * (v - reference_position_); }
  Ogre::Vector3 attachedLocalToFixedFrame(const Ogre::Vector3& v) { return reference_position_ + (reference_orientation_ * v); }

  /** @brief Return the distance between camera and focal point. */
  float getDistanceFromCameraToFocalPoint();

  /** @brief Convert the relative progress in time to the corresponding relative progress in space wrt. the interpolation speed profile.
   *
   * @params[in] relative_progress_in_time  the relative progress in time.
   * @params[in] interpolation_speed        speed profile.
   */
  float computeRelativeProgressInSpace(double relative_progress_in_time, uint8_t interpolation_speed);

  /** @brief Publish the rendered image that is visible to the user in rviz. */
  void publishViewImage();

protected:    //members

  ros::NodeHandle nh_;

  rviz::BoolProperty* mouse_enabled_property_;            ///< If True, most user changes to camera state are disabled.
  rviz::EditableEnumProperty* interaction_mode_property_; ///< Select between Orbit or FPS control style.
  rviz::BoolProperty* fixed_up_property_;                 ///< If True, "up" is fixed to ... up.

  rviz::FloatProperty* distance_property_;                ///< The camera's distance from the focal point
  rviz::VectorProperty* eye_point_property_;              ///< The position of the camera.
  rviz::VectorProperty* focus_point_property_;            ///< The point around which the camera "orbits".
  rviz::VectorProperty* up_vector_property_;              ///< The up vector for the camera.
  rviz::FloatProperty* default_transition_duration_property_; ///< A default time for any animation requests.

  rviz::RosTopicProperty* camera_trajectory_topic_property_;

  rviz::FloatProperty* transition_velocity_property_;     ///< The current velocity of the animated camera.
  
  rviz::FloatProperty* window_width_property_;            ///< The width of the rviz visualization window in pixels.
  rviz::FloatProperty* window_height_property_;           ///< The height of the rviz visualization window in pixels.
    
  rviz::TfFrameProperty* attached_frame_property_;
  Ogre::SceneNode* attached_scene_node_;

  Ogre::Quaternion reference_orientation_;    ///< Used to store the orientation of the attached frame relative to <Fixed Frame>
  Ogre::Vector3 reference_position_;          ///< Used to store the position of the attached frame relative to <Fixed Frame>

  // Variables used during animation
  bool animate_;
  ros::WallTime transition_start_time_;
  BufferCamMovements cam_movements_buffer_;

  std::shared_ptr<rviz::Shape> focal_shape_;    ///< A small ellipsoid to show the focus point.
  bool dragging_;         ///< A flag indicating the dragging state of the mouse.

  QCursor interaction_disabled_cursor_;         ///< A cursor for indicating mouse interaction is disabled.

  ros::Subscriber trajectory_sub_;
  ros::Subscriber record_params_sub_;
  ros::Subscriber wait_duration_sub_;

  ros::Publisher placement_pub_;
  ros::Publisher odometry_pub_;
  ros::Publisher finished_rendering_trajectory_pub_;
  ros::Publisher delete_pub_;
  image_transport::Publisher image_pub_;

  bool render_frame_by_frame_;
  int target_fps_;
  int recorded_frames_counter_;

  bool do_wait_;
  float wait_duration_;
};

}  // namespace rviz_cinematographer_view_controller

#endif // RVIZ_CINEMATOGRAPHER_VIEW_CONTROLLER_H
