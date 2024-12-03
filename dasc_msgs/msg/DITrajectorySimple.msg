std_msgs/Header header

# from the start time in the header, construct a trajectory
float64 dt # timestep between these messages, in seconds
float64[] times
DIState[] states
DIAcc[] accelerations # or control input