# One detection of an entity of interest, already placed in the shared world
# frame by the agent that made it. This is what any per-agent detector
# publishes, synthetic or camera-based: a camera detector projects its image
# box to a world position with its own pose estimate before filling this in.
#
# Kept small on purpose (Phase 4 pushes these through a bandwidth-limited link).
# Measured CDR size: 44 bytes per detection; a DetectionArray is 24 bytes
# empty, 68 with one detection (a std_msgs/Header alone would add a string).

uint8 CLASS_UNKNOWN=0
uint8 CLASS_PERSON=1
uint8 CLASS_VEHICLE=2
uint8 CLASS_OBJECT=3

uint8 class_id            # CLASS_*
float32 confidence        # [0, 1]
float32[3] position       # world frame: Gazebo ENU (x east, y north, z up) [m]
float32[6] covariance     # position covariance, upper triangle xx xy xz yy yz zz [m^2]
