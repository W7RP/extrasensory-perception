# Where an agent's gimbal camera is pointing now. The gimbal is stabilised in
# the world: yaw and pitch are ENU angles, independent of the airframe's
# attitude. Read by the detector (the zoom camera's pointing), the device (its
# picture-in-picture label) and the planner.

builtin_interfaces/Time stamp
uint8 agent
float32 yaw                     # optical axis heading, ENU [rad]
float32 pitch_down              # below the horizon [rad]
float32 hfov                    # [rad]
float32 vfov                    # [rad]
uint32 track_id                 # the track it was commanded to look at, 0 = none
bool on_target                  # within a fraction of the field of view of the command
