# Everything one agent detected in one sensor frame. An empty array is a valid
# and useful message: "I looked and saw nothing". The frame is implicit: every
# position in this message set is in the world frame (see Detection.msg), which
# saves a frame_id string per message.

builtin_interfaces/Time stamp   # sensor sample time, simulation clock
uint8 source_agent              # agent id, 1-based
uint32 frame_seq                # per-agent frame counter: gaps show frames lost on a link
Detection[<=16] detections
