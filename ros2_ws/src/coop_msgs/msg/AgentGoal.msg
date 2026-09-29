# Where the planner wants an agent to be: a world position and a camera
# heading, re-issued every planning cycle. `land` ends the agent's mission.

builtin_interfaces/Time stamp
uint8 agent
float32[3] position             # world frame (ENU) [m]
float32 yaw                     # camera heading, ENU [rad]
bool land
float32 score                   # the planner's value for this spot (hidden area it covers, m^2)
