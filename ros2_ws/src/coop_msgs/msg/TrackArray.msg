# The full track picture from one producer at one instant. Every track is
# predicted to `stamp`, so a consumer can use it directly.
# Measured CDR size: 20 bytes empty + 64 bytes per track.

builtin_interfaces/Time stamp   # tracks predicted to this time, simulation clock
uint8 producer                  # 0 = fusion node / device; n = agent n's local tracker
Track[<=32] tracks
