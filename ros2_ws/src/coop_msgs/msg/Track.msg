# One track: the fused estimate of one entity of interest.

uint8 STATUS_TENTATIVE=0        # born, not yet confirmed by enough detections
uint8 STATUS_CONFIRMED=1        # confirmed, and updated recently
uint8 STATUS_COASTING=2         # confirmed, but no detection for a while: predicted only

uint32 id                       # unique per producer, never reused
uint8 class_id                  # Detection.CLASS_*
uint8 status                    # STATUS_*
uint8 source_mask               # bit (n-1) set: agent n contributed a detection recently
builtin_interfaces/Time last_update   # time of the newest detection fused into it
float32[3] position             # world frame [m], predicted to TrackArray.stamp
float32[3] velocity             # world frame [m/s]
float32[6] covariance           # position covariance, upper triangle xx xy xz yy yz zz [m^2]
