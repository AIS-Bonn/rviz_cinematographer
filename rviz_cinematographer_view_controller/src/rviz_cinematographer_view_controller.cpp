/*
 * Copyright (c) 2009, Willow Garage, Inc.
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
 */

#include "rviz_cinematographer_view_controller/rviz_cinematographer_view_controller.h"

namespace rviz_cinematographer_view_controller
{
using namespace rviz;

// Strings for selecting control mode styles
static const std::string MODE_ORBIT = "Orbit";
static const std::string MODE_FPS = "FPS";

// Limits to prevent orbit controller singularity, but not currently used.
static const Ogre::Radian PITCH_LIMIT_LOW  = Ogre::Radian(0.02);
static const Ogre::Radian PITCH_LIMIT_HIGH = Ogre::Radian(Ogre::Math::PI - 0.02);

// Some convenience functions for Ogre / geometry_msgs conversions
static inline Ogre::Vector3 vectorFromMsg(const geometry_msgs::Point &m) { return Ogre::Vector3(m.x, m.y, m.z); }
static inline Ogre::Vector3 vectorFromMsg(const geometry_msgs::Vector3 &m) { return Ogre::Vector3(m.x, m.y, m.z); }
static inline geometry_msgs::Point pointOgreToMsg(const Ogre::Vector3 &o)
{
  geometry_msgs::Point m;
  m.x = o.x; m.y = o.y; m.z = o.z;
  return m;
}

static inline geometry_msgs::Vector3 vectorOgreToMsg(const Ogre::Vector3 &o)
{
  geometry_msgs::Vector3 m;
  m.x = o.x; m.y = o.y; m.z = o.z;
  return m;
}

// -----------------------------------------------------------------------------


CinematographerViewController::CinematographerViewController()
: nh_("")
  , animate_(false)
  , dragging_(false)
  , do_record_(false)
  , target_fps_(60)
  , recorded_frames_counter_(0)
{
  interaction_disabled_cursor_ = makeIconCursor("package://rviz/icons/forbidden.svg");

  mouse_enabled_property_ = new BoolProperty("Mouse Enabled", true, "Enables mouse control of the camera.", this);

  interaction_mode_property_ = new EditableEnumProperty("Control Mode", QString::fromStdString(MODE_ORBIT), "Select the style of mouse interaction.", this);
  interaction_mode_property_->addOptionStd(MODE_ORBIT);
  interaction_mode_property_->addOptionStd(MODE_FPS);
  interaction_mode_property_->setStdString(MODE_ORBIT);

  fixed_up_property_        = new BoolProperty("Maintain Vertical Axis", true, "If enabled, the camera is not allowed to roll side-to-side.", this);
  attached_frame_property_  = new TfFrameProperty("Target Frame", TfFrameProperty::FIXED_FRAME_STRING, "TF frame the camera is attached to.", this, NULL, true);
  eye_point_property_       = new VectorProperty("Eye", Ogre::Vector3( 5, 5, 10 ), "Position of the camera.", this);
  focus_point_property_     = new VectorProperty("Focus", Ogre::Vector3::ZERO, "Position of the focus/orbit point.", this);
  up_vector_property_       = new VectorProperty("Up", Ogre::Vector3::UNIT_Z, "The vector which maps to \"up\" in the camera image plane.", this);
  distance_property_        = new FloatProperty("Distance", getDistanceFromCameraToFocalPoint(), "The distance between the camera position and the focus point.", this);
  distance_property_->setMin(0.01);

  default_transition_time_property_ = new FloatProperty("Transition Time", 0.5, "The default time to use for camera transitions.", this);
  camera_trajectory_topic_property_ = new RosTopicProperty("Trajectory Topic", "/rviz/camera_trajectory", QString::fromStdString(ros::message_traits::datatype<rviz_cinematographer_msgs::CameraTrajectory>()), "Topic for CameraTrajectory messages", this, SLOT(updateTopics()));

  // TODO: latch?
  placement_pub_ = nh_.advertise<geometry_msgs::Pose>("/rviz/current_camera_pose", 1);
  odometry_pub_ = nh_.advertise<nav_msgs::Odometry>("/rviz/trajectory_odometry", 1);

  image_transport::ImageTransport it(nh_);
  image_pub_ = it.advertise("/rviz/view_image", 1);
}

CinematographerViewController::~CinematographerViewController()
{
  context_->getSceneManager()->destroySceneNode(attached_scene_node_);
}

void CinematographerViewController::updateTopics()
{
  trajectory_sub_ = nh_.subscribe<rviz_cinematographer_msgs::CameraTrajectory>
                        (camera_trajectory_topic_property_->getStdString(), 1,
                         boost::bind(&CinematographerViewController::cameraTrajectoryCallback, this, _1));
}

void CinematographerViewController::onInitialize()
{
  attached_frame_property_->setFrameManager(context_->getFrameManager());

  camera_->detachFromParent();
  attached_scene_node_ = context_->getSceneManager()->getRootSceneNode()->createChildSceneNode();
  attached_scene_node_->attachObject(camera_);
  camera_->setProjectionType(Ogre::PT_PERSPECTIVE);

  focal_shape_ = std::make_shared<rviz::Shape>(Shape::Sphere, context_->getSceneManager(), attached_scene_node_);
  focal_shape_->setScale(Ogre::Vector3(0.05f, 0.05f, 0.01f));
  focal_shape_->setColor(1.0f, 1.0f, 0.0f, 0.5f);
  focal_shape_->getRootNode()->setVisible(false);

  const unsigned long buffer_capacity = 100;
  cam_movements_buffer_ = BufferCamMovements(buffer_capacity);
}

void CinematographerViewController::onActivate()
{
  updateAttachedSceneNode();

  // Before activation, changes to target frame property should have
  // no side-effects.  After activation, changing target frame
  // property has the side effect (typically) of changing an offset
  // property so that the view does not jump.  Therefore we make the
  // signal/slot connection from the property here in onActivate()
  // instead of in the constructor.
  connect(attached_frame_property_, SIGNAL(changed()), this, SLOT(updateAttachedFrame()));
  connect(fixed_up_property_,       SIGNAL(changed()), this, SLOT(onUpPropertyChanged()));
  connectPositionProperties();

  updateTopics();
}

void CinematographerViewController::connectPositionProperties()
{
  connect( distance_property_,    SIGNAL( changed() ), this, SLOT( onDistancePropertyChanged() ), Qt::UniqueConnection);
  connect( eye_point_property_,   SIGNAL( changed() ), this, SLOT( onEyePropertyChanged() ),      Qt::UniqueConnection);
  connect( focus_point_property_, SIGNAL( changed() ), this, SLOT( onFocusPropertyChanged() ),    Qt::UniqueConnection);
  connect( up_vector_property_,   SIGNAL( changed() ), this, SLOT( onUpPropertyChanged() ),       Qt::UniqueConnection);
}

void CinematographerViewController::disconnectPositionProperties()
{
  disconnect( distance_property_,    SIGNAL( changed() ), this, SLOT( onDistancePropertyChanged() ));
  disconnect( eye_point_property_,   SIGNAL( changed() ), this, SLOT( onEyePropertyChanged() ));
  disconnect( focus_point_property_, SIGNAL( changed() ), this, SLOT( onFocusPropertyChanged() ));
  disconnect( up_vector_property_,   SIGNAL( changed() ), this, SLOT( onUpPropertyChanged() ));
}

void CinematographerViewController::onEyePropertyChanged()
{
  distance_property_->setFloat(getDistanceFromCameraToFocalPoint());
}

void CinematographerViewController::onFocusPropertyChanged()
{
  distance_property_->setFloat(getDistanceFromCameraToFocalPoint());
}

void CinematographerViewController::onDistancePropertyChanged()
{
  disconnectPositionProperties();
  Ogre::Vector3 new_eye_position = focus_point_property_->getVector() + distance_property_->getFloat()* camera_->getOrientation().zAxis();
  eye_point_property_->setVector(new_eye_position);
  connectPositionProperties();
}

void CinematographerViewController::onUpPropertyChanged()
{
  disconnect( up_vector_property_,   SIGNAL( changed() ), this, SLOT( onUpPropertyChanged() ));
  if(fixed_up_property_->getBool()){
    up_vector_property_->setVector(Ogre::Vector3::UNIT_Z);
    camera_->setFixedYawAxis(true, reference_orientation_ * Ogre::Vector3::UNIT_Z);
  }
  else {
    // force orientation to match up vector; first call doesn't actually change the quaternion
    camera_->setFixedYawAxis(true, reference_orientation_ * up_vector_property_->getVector());
    camera_->setDirection( reference_orientation_ * (focus_point_property_->getVector() - eye_point_property_->getVector()));
    // restore normal behavior
    camera_->setFixedYawAxis(false);
  }
  connect( up_vector_property_,   SIGNAL( changed() ), this, SLOT( onUpPropertyChanged() ),       Qt::UniqueConnection);
}

void CinematographerViewController::updateAttachedFrame()
{
  Ogre::Vector3 old_position = attached_scene_node_->getPosition();
  Ogre::Quaternion old_orientation = attached_scene_node_->getOrientation();

  updateAttachedSceneNode();

  onAttachedFrameChanged( old_position, old_orientation );
}

void CinematographerViewController::updateAttachedSceneNode()
{
  // TODO: this can be wildly refactored
  Ogre::Vector3 new_reference_position;
  Ogre::Quaternion new_reference_orientation;

  bool queue = false;
  if( context_->getFrameManager()->getTransform( attached_frame_property_->getFrameStd(), ros::Time(),
                                                 new_reference_position, new_reference_orientation ))
  {
    attached_scene_node_->setPosition( new_reference_position );
    attached_scene_node_->setOrientation( new_reference_orientation );
    reference_position_ = new_reference_position;
    reference_orientation_ = new_reference_orientation;
    queue = true;
  }
  if(queue) context_->queueRender();
}

void CinematographerViewController::onAttachedFrameChanged(const Ogre::Vector3& old_reference_position, const Ogre::Quaternion& old_reference_orientation)
{
  Ogre::Vector3 fixed_frame_focus_position = old_reference_orientation*focus_point_property_->getVector() + old_reference_position;
  Ogre::Vector3 fixed_frame_eye_position = old_reference_orientation*eye_point_property_->getVector() + old_reference_position;
  Ogre::Vector3 new_focus_position = fixedFrameToAttachedLocal(fixed_frame_focus_position);
  Ogre::Vector3 new_eye_position = fixedFrameToAttachedLocal(fixed_frame_eye_position);
  Ogre::Vector3 new_up_vector = reference_orientation_.Inverse()*old_reference_orientation*up_vector_property_->getVector();

  //Ogre::Quaternion new_camera_orientation = reference_orientation_.Inverse()*old_reference_orientation*getOrientation();

  focus_point_property_->setVector(new_focus_position);
  eye_point_property_->setVector(new_eye_position);
  up_vector_property_->setVector(fixed_up_property_->getBool() ? Ogre::Vector3::UNIT_Z : new_up_vector);
  distance_property_->setFloat( getDistanceFromCameraToFocalPoint());

  // force orientation to match up vector; first call doesn't actually change the quaternion
  camera_->setFixedYawAxis(true, reference_orientation_ * up_vector_property_->getVector());
  camera_->setDirection( reference_orientation_ * (focus_point_property_->getVector() - eye_point_property_->getVector()));
}

float CinematographerViewController::getDistanceFromCameraToFocalPoint()
{
    return (eye_point_property_->getVector() - focus_point_property_->getVector()).length();
}

void CinematographerViewController::reset()
{
  eye_point_property_->setVector(Ogre::Vector3(5, 5, 10));
    focus_point_property_->setVector(Ogre::Vector3::ZERO);
    up_vector_property_->setVector(Ogre::Vector3::UNIT_Z);
    distance_property_->setFloat( getDistanceFromCameraToFocalPoint());
    mouse_enabled_property_->setBool(true);
    interaction_mode_property_->setStdString(MODE_ORBIT);


  // Hersh says: why is the following junk necessary?  I don't know.
  // However, without this you need to call reset() twice after
  // switching from TopDownOrtho to FPS.  After the first call the
  // camera is in the right position but pointing the wrong way.
  updateCamera();
  camera_->lookAt( 0, 0, 0 );
  setPropertiesFromCamera( camera_ );
}

void CinematographerViewController::handleMouseEvent(ViewportMouseEvent& event)
{
  if( !mouse_enabled_property_->getBool() )
  {
    setCursor( interaction_disabled_cursor_ );
    setStatus( "<b>Mouse interaction is disabled. You can enable it by checking the \"Mouse Enabled\" check-box in the Views panel." );
    return;
  }
  else if ( event.shift() )
  {
    setStatus( "TODO: Fix me! <b>Left-Click:</b> Move X/Y.  <b>Right-Click:</b>: Move Z." );
  }
  else if ( event.control() )
  {
    setStatus( "TODO: Fix me! <b>Left-Click:</b> Move X/Y.  <b>Right-Click:</b>: Move Z." );
  }
  else
  {
    setStatus( "TODO: Fix me! <b>Left-Click:</b> Rotate.  <b>Middle-Click:</b> Move X/Y.  <b>Right-Click:</b>: Zoom.  <b>Shift</b>: More options." );
  }

  float distance = distance_property_->getFloat();
  int32_t diff_x = 0;
  int32_t diff_y = 0;
  bool moved = false;

  if( event.type == QEvent::MouseButtonPress )
  {
    focal_shape_->getRootNode()->setVisible(true);
    moved = true;
    dragging_ = true;
    cancelTransition();  // Stop any automated movement
  }
  else if( event.type == QEvent::MouseButtonRelease )
  {
    focal_shape_->getRootNode()->setVisible(false);
    moved = true;
    dragging_ = false;
  }
  else if( dragging_ && event.type == QEvent::MouseMove )
  {
    diff_x = event.x - event.last_x;
    diff_y = event.y - event.last_y;
    moved = true;
  }

  // regular left-button drag
  if( event.left() && !event.shift() )
  {
    setCursor( Rotate3D );
    yaw_pitch_roll( -diff_x*0.005, -diff_y*0.005, 0 );
  }
  // middle or shift-left drag
  else if( event.middle() || ( event.shift() && event.left() ))
  {
    setCursor( MoveXY );
    if(interaction_mode_property_->getStdString() == MODE_ORBIT)  // Orbit style
    {
        float fovY = camera_->getFOVy().valueRadians();
        float fovX = 2.0f * atan( tan( fovY / 2.0f ) * camera_->getAspectRatio() );

        int width = camera_->getViewport()->getActualWidth();
        int height = camera_->getViewport()->getActualHeight();

        move_focus_and_eye( -((float)diff_x / (float)width) * distance * tan( fovX / 2.0f ) * 2.0f,
              ((float)diff_y / (float)height) * distance * tan( fovY / 2.0f ) * 2.0f,
              0.0f );
    }
    else if(interaction_mode_property_->getStdString() == MODE_FPS)  // Orbit style
    {
      move_focus_and_eye( diff_x*0.01, -diff_y*0.01, 0.0f );
    }
  }
  else if( event.right() )
  {
    if( event.shift() ||  (interaction_mode_property_->getStdString() == MODE_FPS) )
    {
      setCursor( MoveZ );
      move_focus_and_eye(0.0f, 0.0f, diff_y * 0.01 * distance);
    }
    else
    {
      setCursor( Zoom );
      move_eye( 0, 0, diff_y * 0.01 * distance );
    }
  }
  else
  {
    setCursor( event.shift() ? MoveXY : Rotate3D );
  }

  if ( event.wheel_delta != 0 )
  {
    int diff = event.wheel_delta;

    if( event.shift() )
    {
      move_focus_and_eye(0, 0, -diff * 0.001 * distance );
    }
    else if(event.control())
    {
      yaw_pitch_roll(0, 0, diff*0.001 );
    }
    else
    {
      move_eye( 0, 0, -diff * 0.001 * distance );
    }
    moved = true;
  }

  if(event.type == QEvent::MouseButtonPress && event.left() && event.control() && event.shift())
  {
    bool was_orbit = (interaction_mode_property_->getStdString() == MODE_ORBIT);
    interaction_mode_property_->setStdString(was_orbit ? MODE_FPS : MODE_ORBIT );
  }

  if (moved)
  {
    publishCameraPose();
    context_->queueRender();
  }
}

//void CinematographerViewController::setUpVectorPropertyModeDependent( const Ogre::Vector3 &vector )
//{
//  if(fixed_up_property_->getBool())
//  {
//    //up_vector_property_->setVector(Ogre::Vector3::UNIT_Z);
//  }
//  else {
//    up_vector_property_->setVector(vector);
//  }
//}


void CinematographerViewController::setPropertiesFromCamera( Ogre::Camera* source_camera )
{
  disconnectPositionProperties();
  Ogre::Vector3 direction = source_camera->getOrientation() * Ogre::Vector3::NEGATIVE_UNIT_Z;
  eye_point_property_->setVector( source_camera->getPosition() );
  focus_point_property_->setVector( source_camera->getPosition() + direction*distance_property_->getFloat());
  if(fixed_up_property_->getBool())
    up_vector_property_->setVector(Ogre::Vector3::UNIT_Z);
  else
    up_vector_property_->setVector(source_camera->getOrientation().yAxis());

  //setUpVectorPropertyModeDependent(source_camera->getOrientation().yAxis());
  connectPositionProperties();
}

void CinematographerViewController::mimic( ViewController* source_view )
{
  QVariant target_frame = source_view->subProp( "Target Frame" )->getValue();
    if( target_frame.isValid() )
    {
      attached_frame_property_->setValue( target_frame );
    }

    Ogre::Camera* source_camera = source_view->getCamera();
    Ogre::Vector3 position = source_camera->getPosition();
    Ogre::Quaternion orientation = source_camera->getOrientation();

    if( source_view->getClassId() == "rviz/Orbit" )
    {
        distance_property_->setFloat( source_view->subProp( "Distance" )->getValue().toFloat() );
    }
    else
    {
        distance_property_->setFloat( position.length() );
    }
    interaction_mode_property_->setStdString( MODE_ORBIT );

    Ogre::Vector3 direction = orientation * (Ogre::Vector3::NEGATIVE_UNIT_Z * distance_property_->getFloat() );
    focus_point_property_->setVector( position + direction );
    eye_point_property_->setVector(position);
    updateCamera();
}

void CinematographerViewController::transitionFrom( ViewController* previous_view )
{
  CinematographerViewController* fvc = dynamic_cast<CinematographerViewController*>(previous_view);
  if(fvc)
  {
    Ogre::Vector3 new_eye =   eye_point_property_->getVector();
    Ogre::Vector3 new_focus = focus_point_property_->getVector();
    Ogre::Vector3 new_up =    up_vector_property_->getVector();

    eye_point_property_->setVector(fvc->eye_point_property_->getVector());
    focus_point_property_->setVector(fvc->focus_point_property_->getVector());
    up_vector_property_->setVector(fvc->up_vector_property_->getVector());

    beginNewTransition(new_eye,
                       new_focus,
                       new_up,
                       ros::Duration(default_transition_time_property_->getFloat()));
  }
}

void CinematographerViewController::beginNewTransition(const Ogre::Vector3 &eye,
                                                const Ogre::Vector3 &focus,
                                                const Ogre::Vector3 &up,
                                                ros::Duration transition_time,
                                                uint8_t interpolation_speed)
{
  // if jump was requested, perform as usual but prevent division by zero
  if(ros::Duration(transition_time).isZero())
    transition_time = ros::Duration(0.001);

  // if the buffer is empty we set the first element in it to the current camera pose
  if(cam_movements_buffer_.empty())
  {
    transition_start_time_ = ros::WallTime::now();

    cam_movements_buffer_.push_back(std::move(OgreCameraMovement(eye_point_property_->getVector(),
                                                                 focus_point_property_->getVector(),
                                                                 up_vector_property_->getVector(),
                                                                 ros::Duration(0.001),
                                                                 interpolation_speed))); // interpolation_speed doesn't make a difference for very short times
  }

  if(cam_movements_buffer_.full())
    cam_movements_buffer_.set_capacity(cam_movements_buffer_.capacity() + 20);

  cam_movements_buffer_.push_back(std::move(OgreCameraMovement(eye, focus, up, transition_time, interpolation_speed)));

  animate_ = true;
}

void CinematographerViewController::cancelTransition()
{
  animate_ = false;
  cam_movements_buffer_.clear();
}

void CinematographerViewController::cameraTrajectoryCallback(const rviz_cinematographer_msgs::CameraTrajectoryConstPtr &ct_ptr)
{
  rviz_cinematographer_msgs::CameraTrajectory ct = *ct_ptr;

  if(ct.trajectory.empty())
    return;

  // Handle control parameters
  mouse_enabled_property_->setBool(!ct.interaction_disabled);
  fixed_up_property_->setBool(!ct.allow_free_yaw_axis);
  if(ct.mouse_interaction_mode != rviz_cinematographer_msgs::CameraTrajectory::NO_CHANGE)
  {
    std::string name = "";
    if(ct.mouse_interaction_mode == rviz_cinematographer_msgs::CameraTrajectory::ORBIT)
      name = MODE_ORBIT;
    else if(ct.mouse_interaction_mode == rviz_cinematographer_msgs::CameraTrajectory::FPS)
      name = MODE_FPS;
    interaction_mode_property_->setStdString(name);
  }

  for(auto& cam_movement : ct.trajectory)
  {
    transformCameraMovementToAttachedFrame(cam_movement);

    if(ct.target_frame != "")
    {
      attached_frame_property_->setStdString(ct.target_frame);
      updateAttachedFrame();
    }

    Ogre::Vector3 eye = vectorFromMsg(cam_movement.eye.point);
    Ogre::Vector3 focus = vectorFromMsg(cam_movement.focus.point);
    Ogre::Vector3 up = vectorFromMsg(cam_movement.up.vector);
    beginNewTransition(eye, focus, up, cam_movement.transition_time, cam_movement.interpolation_speed);
  }
}

void CinematographerViewController::transformCameraMovementToAttachedFrame(rviz_cinematographer_msgs::CameraMovement &cm)
{
  Ogre::Vector3 position_fixed_eye, position_fixed_focus, position_fixed_up; // position_fixed_attached;
  Ogre::Quaternion rotation_fixed_eye, rotation_fixed_focus, rotation_fixed_up; // rotation_fixed_attached;

  context_->getFrameManager()->getTransform(cm.eye.header.frame_id, ros::Time(0), position_fixed_eye, rotation_fixed_eye);
  context_->getFrameManager()->getTransform(cm.focus.header.frame_id,  ros::Time(0), position_fixed_focus, rotation_fixed_focus);
  context_->getFrameManager()->getTransform(cm.up.header.frame_id,  ros::Time(0), position_fixed_up, rotation_fixed_up);
  //context_->getFrameManager()->getTransform(attached_frame_property_->getStdString(),  ros::Time(0), position_fixed_attached, rotation_fixed_attached);

  Ogre::Vector3 eye = vectorFromMsg(cm.eye.point);
  Ogre::Vector3 focus = vectorFromMsg(cm.focus.point);
  Ogre::Vector3 up = vectorFromMsg(cm.up.vector);

  eye = fixedFrameToAttachedLocal(position_fixed_eye + rotation_fixed_eye*eye);
  focus = fixedFrameToAttachedLocal(position_fixed_focus + rotation_fixed_focus*focus);
  up = reference_orientation_.Inverse()*rotation_fixed_up*up;
  //up = rotation_fixed_up*up;

  cm.eye.point = pointOgreToMsg(eye);
  cm.focus.point = pointOgreToMsg(focus);
  cm.up.vector = vectorOgreToMsg(up);
  cm.eye.header.frame_id = attached_frame_property_->getStdString();
  cm.focus.header.frame_id = attached_frame_property_->getStdString();
  cm.up.header.frame_id = attached_frame_property_->getStdString();
}


// We must assume that this point is in the Rviz Fixed frame since it came from Rviz...
void CinematographerViewController::lookAt( const Ogre::Vector3& point )
{
  if( !mouse_enabled_property_->getBool() ) return;

  Ogre::Vector3 new_point = fixedFrameToAttachedLocal(point);

  beginNewTransition(eye_point_property_->getVector(),
                     new_point,
                     up_vector_property_->getVector(),
                     ros::Duration(default_transition_time_property_->getFloat()));

  //  // Just for easily testing the other movement styles:
  //  orbitCameraTo(point);
  //  moveCameraWithFocusTo(point);
}

void CinematographerViewController::orbitCameraTo( const Ogre::Vector3& point)
{
  beginNewTransition(point,
                     focus_point_property_->getVector(),
                     up_vector_property_->getVector(),
                     ros::Duration(default_transition_time_property_->getFloat()));
}

void CinematographerViewController::moveEyeWithFocusTo( const Ogre::Vector3& point)
{
  beginNewTransition(point,
                     focus_point_property_->getVector() + (point - eye_point_property_->getVector()),
                     up_vector_property_->getVector(),
                     ros::Duration(default_transition_time_property_->getFloat()));
}

void CinematographerViewController::publishOdometry(const Ogre::Vector3& position,
                                             const Ogre::Vector3& velocity)
{
  nav_msgs::Odometry odometry;
  odometry.header.frame_id = attached_frame_property_->getFrameStd();
  odometry.header.stamp = ros::Time::now();
  odometry.pose.pose.position.x = position.x;
  odometry.pose.pose.position.y = position.y;
  odometry.pose.pose.position.z = position.z;
  odometry.twist.twist.linear.x = velocity.x; //This is allo velocity and therefore not ROS convention!
  odometry.twist.twist.linear.y = velocity.y; //This is allo velocity and therefore not ROS convention!
  odometry.twist.twist.linear.z = velocity.z; //This is allo velocity and therefore not ROS convention!

  Ogre::Quaternion cam_orientation = getOrientation();
  Ogre::Quaternion rot_around_y_pos_90_deg(0.707f, 0.0f, 0.707f, 0.0f);
  cam_orientation = cam_orientation * rot_around_y_pos_90_deg;
  odometry.pose.pose.orientation.x = cam_orientation.x;
  odometry.pose.pose.orientation.y = cam_orientation.y;
  odometry.pose.pose.orientation.z = cam_orientation.z;
  odometry.pose.pose.orientation.w = cam_orientation.w;
  odometry_pub_.publish(odometry);
}

void CinematographerViewController::update(float dt, float ros_dt)
{
  updateAttachedSceneNode();

  // there has to be at least two positions in the buffer - start and goal
  if(animate_ && cam_movements_buffer_.size() > 1)
  {
    auto start = cam_movements_buffer_.begin();
    auto goal = ++(cam_movements_buffer_.begin());

    double fraction = 0.0;
    if(do_record_)
    {
      fraction = counter_ / (target_fps_ * goal->transition_time.toSec());
      counter_++;
    }
    else
    {
      ros::WallDuration time_from_start = ros::WallTime::now() - transition_start_time_;
      fraction = time_from_start.toSec()/goal->transition_time.toSec();
    }

    // make sure we get all the way there before turning off
    if(fraction >= 1.0)
    {
      fraction = 1.0;
      animate_ = false;
    }

    float progress = 0.0f;
    switch(goal->interpolation_speed)
    {
      case rviz_cinematographer_msgs::CameraMovement::RISING:
        progress = 1.f - static_cast<float>(cos(fraction * M_PI_2));
        break;
      case rviz_cinematographer_msgs::CameraMovement::DECLINING:
        progress = static_cast<float>(-cos(fraction * M_PI_2 + M_PI_2));
        break;
      case rviz_cinematographer_msgs::CameraMovement::FULL:
        progress = static_cast<float>(fraction);
        break;
      case rviz_cinematographer_msgs::CameraMovement::WAVE:
      default:
        progress = 0.5f * ( 1.f - static_cast<float>(cos(fraction * M_PI)));
        break;
    }

    Ogre::Vector3 new_position = start->eye + progress*(goal->eye - start->eye);
    Ogre::Vector3 new_focus = start->focus + progress*(goal->focus - start->focus);
    Ogre::Vector3 new_up = start->up + progress*(goal->up - start->up);

    if(odometry_pub_.getNumSubscribers() != 0)
      publishOdometry(new_position, (new_position - eye_point_property_->getVector()) / ros_dt);

    disconnectPositionProperties();
    eye_point_property_->setVector(new_position);
    focus_point_property_->setVector(new_focus);
    up_vector_property_->setVector(new_up);
    distance_property_->setFloat(getDistanceFromCameraToFocalPoint());
    connectPositionProperties();

    // This needs to happen so that the camera orientation will update properly when fixed_up_property == false
    camera_->setFixedYawAxis(true, reference_orientation_ * up_vector_property_->getVector());
    camera_->setDirection(reference_orientation_ * (focus_point_property_->getVector() - eye_point_property_->getVector()));

    publishCameraPose();

    unsigned int height = context_->getViewManager()->getRenderPanel()->getRenderWindow()->getHeight();
    unsigned int width = context_->getViewManager()->getRenderPanel()->getRenderWindow()->getWidth();
    cv::Size img_size((int)width, (int)height);

    Ogre::PixelFormat format = Ogre::PF_BYTE_BGR;
    auto outBytesPerPixel = Ogre::PixelUtil::getNumElemBytes(format);
    auto data = new unsigned char [width * height * outBytesPerPixel];
    Ogre::Box extents(0, 0, width, height);
    Ogre::PixelBox pb(extents, format, data);

    cv::Mat image_rgb(height, width, CV_8UC3, data);

    // TODO: replace with sensor msgs
//    sensor_msgs::ImagePtr ros_image = sensor_msgs::ImagePtr(new sensor_msgs::Image());;
//    ros_image->header.frame_id = attached_frame_property_->getStdString();
//    ros_image->header.stamp = ros::Time::now();
//    ros_image->height = height;
//    ros_image->width = width;
//    ros_image->encoding = sensor_msgs::image_encodings::BGR8;
//    ros_image->is_bigendian = false;
//    ros_image->step = static_cast<unsigned int>(width * outBytesPerPixel);
//    size_t size = width * outBytesPerPixel * height;
//    ros_image->data.resize(size);

    context_->getViewManager()->getRenderPanel()->getRenderWindow()->copyContentsToMemory(pb, Ogre::RenderTarget::FB_AUTO);
//    memcpy((char*)(&ros_image->data[0]), data, size);

//    image_pub_.publish(ros_image);

    if(do_record_ && !output_video_.isOpened())
      if(!output_video_.open("/tmp/raw_video.avi", cv::VideoWriter::fourcc('D', 'I', 'V', 'X'), target_fps_, img_size, true))
        ROS_ERROR("Could not open the output video for write.");

    if(output_video_.isOpened())
      output_video_.write(image_rgb);

    //cv::imwrite("/tmp/Gray_Image" + std::to_string(counter_++) + ".jpg", image_rgb);


    // if current movement is over
    if(!animate_)
    {
      // delete current start element in buffer
      cam_movements_buffer_.pop_front();

      // if there are still movements to perform
      if(cam_movements_buffer_.size() > 1)
      {
        // reset animate to perform the next movement
        animate_ = true;
        counter_ = 0;
        // update the transition start time with the time the transition should have taken
        transition_start_time_ += ros::WallDuration(cam_movements_buffer_.front().transition_time.toSec());
      }
      else
      {
        // clean up
        cam_movements_buffer_.clear();

        if(output_video_.isOpened())
          output_video_.release();
      }
    }
  }

  updateCamera();
}

void CinematographerViewController::updateCamera()
{
  camera_->setPosition( eye_point_property_->getVector() );
  camera_->setFixedYawAxis(fixed_up_property_->getBool(), reference_orientation_ * up_vector_property_->getVector());
  camera_->setDirection( reference_orientation_ * (focus_point_property_->getVector() - eye_point_property_->getVector()));
  //camera_->setDirection( (focus_point_property_->getVector() - eye_point_property_->getVector()));
  focal_shape_->setPosition( focus_point_property_->getVector() );
}

void CinematographerViewController::publishCameraPose()
{
  geometry_msgs::Pose cam_pose;
  cam_pose.position.x = camera_->getPosition().x;
  cam_pose.position.y = camera_->getPosition().y;
  cam_pose.position.z = camera_->getPosition().z;
  cam_pose.orientation.w = camera_->getOrientation().w;
  cam_pose.orientation.x = camera_->getOrientation().x;
  cam_pose.orientation.y = camera_->getOrientation().y;
  cam_pose.orientation.z = camera_->getOrientation().z;
  placement_pub_.publish(cam_pose);
}

void CinematographerViewController::yaw_pitch_roll( float yaw, float pitch, float roll )
{
  Ogre::Quaternion old_camera_orientation = camera_->getOrientation();
  Ogre::Radian old_pitch = old_camera_orientation.getPitch(false);// - Ogre::Radian(Ogre::Math::HALF_PI);

  if(fixed_up_property_->getBool()) yaw = cos(old_pitch.valueRadians() - Ogre::Math::HALF_PI)*yaw; // helps to reduce crazy spinning!

  Ogre::Quaternion yaw_quat, pitch_quat, roll_quat;
  yaw_quat.FromAngleAxis( Ogre::Radian( yaw ), Ogre::Vector3::UNIT_Y );
  pitch_quat.FromAngleAxis( Ogre::Radian( pitch ), Ogre::Vector3::UNIT_X );
  roll_quat.FromAngleAxis( Ogre::Radian( roll ), Ogre::Vector3::UNIT_Z );
  Ogre::Quaternion orientation_change = yaw_quat * pitch_quat * roll_quat;
  Ogre::Quaternion new_camera_orientation = old_camera_orientation * orientation_change;
  Ogre::Radian new_pitch = new_camera_orientation.getPitch(false);// - Ogre::Radian(Ogre::Math::HALF_PI);

  if( fixed_up_property_->getBool() &&
      ((new_pitch > PITCH_LIMIT_HIGH && new_pitch > old_pitch) || (new_pitch < PITCH_LIMIT_LOW && new_pitch < old_pitch)) )
  {
    orientation_change = yaw_quat * roll_quat;
    new_camera_orientation = old_camera_orientation * orientation_change;
  }

//  Ogre::Radian new_roll = new_camera_orientation.getRoll(false);
//  Ogre::Radian new_yaw = new_camera_orientation.getYaw(false);
  //ROS_INFO("old_pitch: %.3f, new_pitch: %.3f", old_pitch.valueRadians(), new_pitch.valueRadians());

  camera_->setOrientation( new_camera_orientation );
  if( interaction_mode_property_->getStdString() == MODE_ORBIT )
  {
    // In orbit mode the focal point stays fixed, so we need to compute the new camera position.
    Ogre::Vector3 new_eye_position = focus_point_property_->getVector() + distance_property_->getFloat()* new_camera_orientation.zAxis();
    eye_point_property_->setVector(new_eye_position);
    camera_->setPosition(new_eye_position);
    setPropertiesFromCamera(camera_);
  }
  else
  {
    // In FPS mode the camera stays fixed, so we can just apply the rotations and then rely on the property update to set the new focal point.
    setPropertiesFromCamera(camera_);
  }
}

Ogre::Quaternion CinematographerViewController::getOrientation()  // Do we need this?
{
  return camera_->getOrientation();
}

void CinematographerViewController::move_focus_and_eye( float x, float y, float z )
{
  Ogre::Vector3 translate( x, y, z );
  eye_point_property_->add( getOrientation() * translate );
  focus_point_property_->add( getOrientation() * translate );
}

void CinematographerViewController::move_eye( float x, float y, float z )
{
  Ogre::Vector3 translate( x, y, z );
  // Only update the camera position if it won't "pass through" the origin
  Ogre::Vector3 new_position = eye_point_property_->getVector() + getOrientation() * translate;
  if( (new_position - focus_point_property_->getVector()).length() > distance_property_->getMin() )
    eye_point_property_->setVector(new_position);
  distance_property_->setFloat(getDistanceFromCameraToFocalPoint());
}



} // end namespace rviz

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( rviz_cinematographer_view_controller::CinematographerViewController, rviz::ViewController )
