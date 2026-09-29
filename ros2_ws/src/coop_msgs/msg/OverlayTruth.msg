# SIMULATION AND EVALUATION ONLY: how well one frame of the device's
# see-through overlay matched the truth. Published by device_view per camera
# frame; never sent over a link, and nothing it contains feeds the overlay.

builtin_interfaces/Time stamp   # the camera frame's capture time
string entity
bool in_frustum                 # the true entity box projects into the image (occluded or not)
bool hidden                     # the device has no line of sight to the entity's centre
bool drawn                      # a track matched to the entity (within 2 m) was drawn
uint32 track_id                 # the matched track (0 if none)
uint8 agents_seeing             # agents contributing to that track recently
float32 age_s                   # frame time - the track's last detection time
float32 world_error_m           # horizontal error of the drawn track
float32 pixel_error             # drawn box centre vs true box centre in the image [px]
float32 iou                     # 2D boxes of the drawn and the true 3D box, intersection over union
