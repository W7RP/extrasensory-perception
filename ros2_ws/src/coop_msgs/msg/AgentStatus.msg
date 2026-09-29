# One agent's own view of itself: where it believes it is and what its camera
# covers. Sent to the device (its minimap and fog of war) and read by the
# planner that positions the agents. Everything here is the agent's own
# estimate, never ground truth.

builtin_interfaces/Time stamp
uint8 agent
float32[3] position             # world frame (ENU) [m], from the agent's navigation
float32 yaw                     # heading of the camera's optical axis, ENU [rad]
float32 camera_pitch_down       # [rad]
float32 hfov                    # [rad]
float32 vfov                    # [rad]
float32 max_range_m
uint8 detections                # detections in its latest frame
