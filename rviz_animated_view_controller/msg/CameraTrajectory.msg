# Array of CameraMovements defining a trajectory.
CameraMovement[] trajectory

# Sets this as the camera attached (fixed) frame before movement.
# An empty string will leave the attached frame unchanged.
string target_frame

# A flag indicating if the camera yaw axis is fixed to +Z of the camera attached_frame
# (defaults to false)
bool allow_free_yaw_axis

# The interaction style that should be activated when movement is done.
uint8 mouse_interaction_mode
uint8 NO_CHANGE = 0 # Leaves the control style unchanged
uint8 ORBIT = 1 # Activates the Orbit-style controller
uint8 FPS   = 2 # Activates the FPS-style controller

# A flag to enable or disable user interaction
# (defaults to false so that interaction is enabled)
bool interaction_disabled

