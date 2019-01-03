# Indicates that recording should be started, if true
bool do_record 

# Path to the resulting recorded video 
string path_to_output

# Desired frames per second for recorded video
int8 frames_per_second

# If true, the video is recorded using the divx codec; if false, raw video is recorded with ffv1 codec
bool compress

# If true, a watermark is added to the recorded video
bool add_watermark
