# Rviz Cinematographer

An rqt plugin to create and edit trajectories for the rviz camera and record its views in a video.

An example trajectory generated in about 3 minutes:

![Example](readme/output.gif)

Visualized [Model Data](https://grabcad.com/library/office-building-9).

# Further information

- [Instructions](rviz_cinematographer_gui)
- [Details - Package Structure](readme)
- [Details - Rviz View Controller](rviz_cinematographer_view_controller)
- [Details - Video Recorder](video_recorder)
 
# Remark

The recorded video will contain a watermark in the bottom right corner.  
Feel free to deactivate it in the GUI.  
If you do so, please mention the *Rviz Cinematographer* in a comment somewhere around your video.  
Your viewers might also be interested in using this tool.

# License

Rviz Cinematographer is licensed under BSD-3.  
This repository includes an adapted version of the [rviz_animated_view_controller](https://github.com/UTNuclearRoboticsPublic/rviz_animated_view_controller) package which is a modification of the official ros [rviz_animated_view_controller](https://github.com/ros-visualization/rviz_animated_view_controller) package for ros kinetic.  
Both of the ladder are licensed under BSD-2.
