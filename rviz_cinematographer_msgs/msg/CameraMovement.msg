# The interpolation speed profile to use during this step
uint8 interpolation_speed
uint8 RISING    = 0 # Speed of the camera rises smoothly - resembles the first quarter of a sinus wave.
uint8 DECLINING = 1 # Speed of the camera declines smoothly - resembles the second quarter of a sinus wave.
uint8 FULL      = 2 # Camera is always at full speed - depending of transition time.
uint8 WAVE      = 3 # RISING and DECLINING concatenated in one movement.

# Defines how long the transition should take.
# A negative value will disable the pose command altogether.
duration transition_duration

# The frame-relative point for the camera.
geometry_msgs/PointStamped eye

# The frame-relative point for the focus (or pivot for an Orbit controller).
geometry_msgs/PointStamped focus

# The frame-relative vector that maps to "up" in the view plane.
# The zero-vector will default to +Z in the view controller's "Target Frame".
geometry_msgs/Vector3Stamped up
