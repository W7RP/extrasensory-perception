# SIMULATION AND EVALUATION ONLY: never produced by a real detector and never
# sent over the link. Whether an agent's sensor could see an entity at this
# instant according to ground truth, before detection dropout is applied.

builtin_interfaces/Time stamp
uint8 agent
string entity
bool visible                    # in_fov && in_range && !occluded
bool in_fov
bool in_range
bool occluded
float32 range_m

# Imaging-model cameras only (Phase 4; zero for the Phase 1-3 range model).
# For the camera that sees the entity best.
uint8 camera                    # index into the detector's camera list
float32 pixels                  # pixels across the entity's critical dimension
float32 p_detect                # detection probability this frame
