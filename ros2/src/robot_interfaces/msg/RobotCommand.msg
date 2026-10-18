# Structured robot action command (LLM/Agent -> execution layer)
#
# action: move | stop | rotate | navigate (extensible: search, grasp, ...)
string action

# Linear velocity (m/s); for move: linear_x positive = forward, negative = backward
float32 linear_x
float32 linear_y

# Angular velocity (rad/s); for rotate: positive = counter-clockwise (left)
float32 angular_z

# Duration (seconds). 0 = keep executing until next command or stop
float32 duration

# Navigation goal for action=navigate (future Nav2, e.g. "kitchen")
string goal

# Reserved extension parameters (JSON string)
string params

# Task source (optional, for logging / experiments)
string source

# Correlation token echoed by RobotStatus for this command.
string command_id
