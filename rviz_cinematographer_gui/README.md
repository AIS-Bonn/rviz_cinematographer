# General

An rqt plugin serving as the front-end for the *Rviz Cinematographer*.  
Create and edit trajectories to move the rviz camera along and record what is visualized in rviz directly to a video file.

![trajectory_editor](readme/trajectory_editor.png)

# Install

Clone or download this repository into your ros catkin workspace.  
Make/Build this package - induces build of other packages. 

# Getting Started

I recommend to launch the provided launch file to get a feeling for this tool without any distractions. 

```
$ roslaunch rviz_cinematographer_gui rviz_cinematographer_gui.launch
```

The tutorial below introduces most features. 

Alternatively, you can start the provided plugin inside rqt at any time to create camera trajectories within already running rviz visualizations.  
Some [remarks](https://git.ais.uni-bonn.de/razlaw/trajectory_editor/tree/master/rqt_pose_interpolator/README.md#remarks) for this use case are added after the tutorial section. 

# Tutorial

This plugin offers a whole variety of options to edit and create camera trajectories using several interactive markers.  
Additionally it provides a set of options to control the camera speed, roll angle, trajectory smoothness and more.  

Hover with your mouse over a button in the plugin to get more information.   

##### Camera Control:

The five buttons always visible on top are used to control the rviz camera.  
Pressing one of them will move the camera with respect to the currently active marker colored in green.  
Left-click on a marker to select the clicked as the active marker.   

| Button | Functionality |
| -------- | -------- |
| ![alt text](icons/first.png) | Move the camera within the trajectory from the active to the first marker |
| ![alt text](icons/prev.png ) | Move the camera to the marker previous to the active |
| Move to Current Pose | Move the camera to the currently active marker |
| ![alt text](icons/next.png ) | Move the camera to the marker after the active |
| ![alt text](icons/last.png ) | Move the camera within the trajectory from the active to the last marker |

Activate the *Record* to record a video of the next camera movement initiated by the five buttons above.  
More details to recording videos are provided [below](https://git.ais.uni-bonn.de/razlaw/trajectory_editor/tree/master/rqt_pose_interpolator/README.md#recording-parameters). 

![rviz_cinematographer_gui](readme/rviz_cinematographer_gui.png)

Remark:  
Rviz doesn't allow to alter the view using the mouse buttons or wheel when hovering over an interactive marker.  
To not get stuck, it's helpful to hold the right-click and move the mouse to zoom out.

### Tabs

In the following, the functionalities organized in the tabs are explained in detail. 

##### Edit Pose:

This tab can be used to edit the marker poses of the trajectory.  
The easiest way to edit a trajectory is by moving the camera within rviz to the pose you like and pressing the *Append at Cam Pose*-button.
This adds a marker at the selected pose to the end of the trajectory.  
Changing the pose of the active marker can be done similarly by pressing the *Set to Cam Pose*-button.  

Alternatively, the pose of and the transition time to the active marker can be tuned using the spin boxes. 

The third option is to manipulate the interactive markers directly using your mouse.  

![tab_edit_pose](readme/tab_edit_pose.png)

###### Adding and Removing Markers:

To edit the number of markers defining the trajectory right-click on an interactive marker.

![trajectory_editor_right_click](readme/trajectory_editor_right_click.png)

Use *Delete* on your keyboard to remove the active marker.  

---

##### Trajectory Parameters:

| Parameter | Functionality |
| -------- | -------- |
| Use Up of World   | If disabled, the camera is not allowed to perform roll motions |
| Spline | If enabled, interpolates poses using a spline |
| Smooth Cam Velocity | Combine with spline to use trajectories' total transition time to move with a smooth velocity to first or last marker | 
| Distance to Focus Point | The distance between the eye point and the focus point of the camera |
| Publishing Rate | Smoothness of spline |
| Marker Size | In- or decrease the markers' size |
| Frame |  The frame_id of the interactive markers |

![trajectory_editor_parameters](readme/trajectory_editor_parameters.png)

Thanks to https://github.com/ejmahler/SplineLibrary for providing an easy to use spline library.

---

##### Recording Parameters:

| Parameter | Functionality |
| -------- | -------- |
| Record to | Directly specify the path and name of the output video file - or use button to navigate in a file explorer |
| Frames Per Second | Specify the FPS for the output video |
| Compress Video | Toggle between codecs - divx for compressed output and FFV1 for raw videos | 
| Add Watermark | If checked, a watermark is added at the bottom right of the video |

![tab_recording_parameters](readme/tab_recording_parameters.png)

---

##### Save/Open Trajectory:

Save your trajectory using the *Save As..*-button and load existing ones using the *Open*-button.  
Additionally one trajectory can be specified in the launch file to be loaded on initialization.

# Remarks

You have the option to create a camera trajectory within an already running rviz instance.   
For this, just run your application and visualization in rviz as usual and additionally start rqt by 

```
$ rqt
```

Click on "Plugins -> Visualization -> Rviz Cinematographer".

**Caveat:** Be aware, that rviz should **not** be started as an rqt plugin, because this experimental version of rviz crashes every time a *Path* message is received.

Now you have to add the *InteractiveMarkers* and the *Path* displays.  
Make sure the frame specified in the Rviz Cinematographer plugin is present in your tf tree, otherwise you will not be able to see the markers. 

Additionally you have to select the rviz_cinematographer_view_controller in the Rviz *Views* in order to be able to move the camera using the plugin.

![rviz_views](readme/rviz_view.png)

If you want to record a video using the Rviz Cinematographer, you addionally have to run an instance of the *video_recorder* by

```
$ roslaunch video_recorder video_recorder.launch
```

I would recommend to use the *Fixed Frame* as the Target Frame for the view. 

---

All of these steps were taken care of when starting the *Rviz Cinematographer* using the provided launch file

```
$ roslaunch rviz_cinematographer_gui rviz_cinematographer_gui.launch
```

