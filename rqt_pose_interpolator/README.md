# General

Small collection of rqt plugins to manipulate the camera in combination with the adapted [rviz_animated_view_controller](https://git.ais.uni-bonn.de/razlaw/thesis/tree/master/rviz_animated_view_controller) package.

# Plugins

### pose_interpolator:

Lightweight tool to quickly **generate simple tracking shots**.

This plugin basicly offers a subset of the functionality of the trajectory_editor, while being easy to use.

Manipulate two interactive markers with your mouse to set a start and end pose or set each marker individually to the current rviz camera pose.

More information to each button/option is displayed when hovering with the mouse over it.

### trajectory_editor:

A more sophisticated tool to **create longer camera trajectories**.

This plugin offers a whole variaty of options to edit and create camera trajectories using several interactive markers and options to control the camera speed, roll angle, trajectory smoothness and more.

Markers can be interpolated using splines (thanks to https://github.com/ejmahler/SplineLibrary).

More information to each button/option is displayed when hovering with the mouse over it. Right-clicks on the interactive markers offer more options edit the trajectory.
