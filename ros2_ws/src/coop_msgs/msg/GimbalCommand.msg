# What the planner wants an agent's gimbal camera to look at: a world point,
# usually a track's position, re-issued every planning cycle. With `active`
# false the gimbal returns to its stow pointing (straight down).

builtin_interfaces/Time stamp
uint8 agent
float32[3] target               # world frame (ENU) [m]
uint32 track_id                 # the track being looked at, 0 = none
bool active
