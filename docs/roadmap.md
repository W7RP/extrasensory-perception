# Roadmap: Phases 2-4 (plan only)

## The goal

The agents in the air collect what they see and get it to a device on the
ground as fast as possible, so the device can **see through walls**. From the
device's own camera, an entity standing behind a building is highlighted
where it actually is, drawn over the wall: an "ESP" view like the ones in
games, but built from real sensor data that travels over a real link.

The project is both halves of that:

- **the capability**: the see-through overlay on the device's camera view,
  as a box, an outline, or a skeleton for a person;
- **the measurement**: how quickly and how accurately that overlay follows
  the real entity, and how that holds up as the link between agents and device
  degrades.

Phase 1 built the foundation: two agents, a perfect link, one fused track.
Nothing below is implemented yet.

## Phase 2: the device

A ground-side node that consumes the picture, with a pose and a view of its own.

- **Device node** (`device` package, C++20): its own pose source (scripted
  ground route, or its own ESKF-style estimate), subscribing to what the agents
  send instead of to `/fusion/tracks` directly.
- **Its own view**: the device runs the same visibility model as the detector
  (FOV, range, ray-cast occlusion from the world file), so for every track it
  knows whether it could see the entity itself.
- **The see-through overlay** (the headline visual): the device gets a
  simulated camera, and every track it holds is projected into that image
  with the device's pose and the camera intrinsics, drawn over whatever
  hides it. Three levels, each more than the last:
  1. **box**: a 3D box of the entity's class size at the track's position,
     facing its direction of travel, projected to the image;
  2. **outline**: the silhouette of a class template (a person model) placed
     and oriented the same way, so it reads as a person, not a rectangle;
  3. **skeleton**, for a person: body keypoints measured by the agents, not a
     template. The synthetic detector gets them from an animated Gazebo actor
     in place of today's static shape; a camera detector would run pose
     estimation. They travel as a separate, lower-priority message: 17
     keypoints are about 200 bytes per person, three times a whole
     one-detection frame (68 bytes).

  The overlay also shows how much to trust it: it grows with the track's
  covariance, and fades and changes style as the data gets older (seen now,
  seen by an agent a moment ago, coasting on prediction). In simulation a
  thin ground-truth outline can be drawn next to it, so the error is visible
  frame by frame. Output: an annotated image topic, viewable in RViz or
  `rqt_image_view`, and recordable as video.
- **Where fusion runs**: two configurations to compare, both with the Phase 1
  tracker: agents send detections and the device fuses them (centralised), or
  agents run local trackers and send tracks, which the device fuses (track-to-
  track, with the cross-correlation that implies; covariance intersection is
  the conservative first step). The message set already carries both
  (`DetectionArray`, `TrackArray.producer`).
- **Navigation bias**: Phase 1 found that the agents' navigation errors, not
  detector noise, dominate the fused error. A per-agent bias estimate from
  jointly observed entities (registration) is the natural fix, and belongs
  here before the link makes it harder to see.
- **Evaluation**: the Phase 1 metrics from the device's point of view, plus
  "fraction of time the device holds a track on an entity it cannot see", and
  the overlay's error in the image (pixels, and overlap of the drawn box with
  the true one).

## Phase 3: the link (the headline result)

A link model node between every agent and the device: messages go in, some
come out later.

- **Bandwidth caps** per link, with priority queues: new and updated tracks of
  confirmed entities before empty frames, newer before older, with message
  sizes taken from the real CDR encoding (Phase 1 measured them: 68 bytes for a
  one-detection frame, 64 bytes per track).
- **Latency and jitter**: a base delay plus a random component, preserving or
  deliberately breaking ordering.
- **Loss**: dependent on range and on whether the agent has line of sight to
  the device (the same ray casting), plus outages (scripted and random
  bursts).
- **What the receiver must handle**: out-of-sequence measurements (the Phase 1
  reorder buffer refuses them today and counts them; the options are
  retrodiction or a longer hold), frame gaps (`frame_seq` already counts
  them), and stale tracks (age-aware prediction).
- **Graceful degradation of the overlay**: position and status first,
  keypoints only when bandwidth allows, so a worse link turns a skeleton into
  an outline and then a coasting box, rather than dropping the entity.
- **Headline curves**: end-to-end latency (the entity moves -> the overlay on
  the device reflects it; and first sighting by any agent -> first overlay)
  and position error, each as a function of link quality (bandwidth, loss
  rate, latency), with the Phase 1 perfect link as the reference point. The
  matching visual: side-by-side overlay videos at several link qualities,
  showing the highlight lag and blur as the link gets worse.

## Phase 4: more agents, planned coverage

- **N agents** (4-6): the simulator, parameters and message set are already
  per-agent; the limits to watch are Gazebo's real-time factor with N flow
  cameras and link contention.
- **Coverage-aware patrol planning**: route agents to minimise the time the
  entity is visible to nobody (and, with Phase 3, to keep a link to the
  device), instead of Phase 1's scripted routes.
- **Evaluation**: continuity and latency against the number of agents and the
  planner, on the Phase 3 link.

## Beyond

- Replace the synthetic detector with a camera detector (e.g. a small YOLO
  model on a simulated RGB camera, plus a pose-estimation model for the
  skeleton overlay). It has to publish the same
  `/agent_<n>/detections` contract: one `DetectionArray` per frame, even when
  empty, stamped with the capture time, world positions with full covariances.
- Several entities, clutter and false detections, where the swappable
  associator (nearest neighbour vs Hungarian today) starts to matter.
