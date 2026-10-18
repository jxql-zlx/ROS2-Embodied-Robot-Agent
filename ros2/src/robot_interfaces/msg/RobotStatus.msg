# Execution-layer status feedback (control node -> agent / logs)
#
# state: idle | running | done | stopped | error
string action
string state

# Remaining execution time (s); -1 = running indefinitely
float32 remaining

# Human-readable message
string message

builtin_interfaces/Time stamp

# Command correlation token copied from RobotCommand.
string command_id
