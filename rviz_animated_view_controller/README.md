# General

Based on [rviz_animated_view_controller](https://github.com/UTNuclearRoboticsPublic/rviz_animated_view_controller) package - a modification of the official ros package to work with ros kinetic.

# Differences

**Messages** :

Replaced CameraPlacement msgs by own messages.

CameraMovement is a subset of CameraPlacement consisting of the target camera pose, the transition time and the interpolation speed - the last was added to add more flexibility concerning the velocity of the camera.

CameraTrajectory consists of a vector of CameraMovements + interaction parameters + target_frame and yaw axis parameter. All of the latter were part of the CameraPlacement message.

**Publishing** :

Everytime the camera is moved in rviz, the camera pose is published.

Additionally Odometry msgs are published when the camera movement is triggered using the messages described above.

**Funcionality** :

Using the CameraTrajectory msgs one can either move the camera the usual way by providing just one CameraMovement in the vector or move the camera along a trajectory specified by several CameraMovements.

**Remark** :

If you want wo switch from the ros rvi_animated_view_controller to the one provided here, you just have to switch from CameraPlacement to the new message type CameraTrajectory.

It provides the same fields - except for interpolation_mode which wasn't used and was "replaced" by interpolation speed.

No previously available functionality was harmed.