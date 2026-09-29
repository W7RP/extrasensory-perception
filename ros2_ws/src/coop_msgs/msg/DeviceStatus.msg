# The ground device's pose, sent up to the agents so they can position
# themselves around it. The one message that flows from the device to the air.

builtin_interfaces/Time stamp
float32[3] position             # world frame (ENU) [m], at the device's feet
float32 yaw                     # heading of the device's camera, ENU [rad]
