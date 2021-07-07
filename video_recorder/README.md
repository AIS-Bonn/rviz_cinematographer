# General

Subscribes to images and starts to generate a video on receiving a *record*-message, optionally adding a watermark.

# Messages

#### Inputs:  

1. **Topic** : /rviz/view_image  
   **Type** : sensor_msgs::Image    
   **Purpose** : The image input.

2. **Topic** : /rviz/record  
   **Type** : rviz_cinematographer_msgs::Record  
   **Purpose** : Parameters for the output video + Starts recording.  
   Frame Rate, Codec, Output File Name, Add Watermark Flag.  

3. **Topic** : /rviz/finished_animation    
   **Type** : std_msgs::Bool    
   **Purpose** : If true, indicates that the input image stream ended.  

#### Outputs:

1. **Topic** : /video_recorder/record_finished  
   **Type** : std_msgs::Bool  
   **Purpose** : If true, indicates that the input stream was fully processed.  

2. **Topic** : /video_recorder/wait_duration  
   **Type** : std_msgs::Duration  
   **Purpose** : The approximate time it takes to process most of the queue buffering the input images.    
   Is send if processing the images takes more time than generating and queueing.  
