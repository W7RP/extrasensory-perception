# Roadmap: Phases 2-4 (plan only)

Phase 1 built the foundation: two agents, a perfect link, one fused track.
The project's question is what happens to that picture on its way to a mobile
device over a degraded link: how late it arrives and how wrong it is when it
gets there. The later phases build towards that measurement. Nothing below is
implemented yet.

## Phase 2: the device

A ground-side node that consumes the picture, with a pose and a view of its own.

- **Device node** (`device` package, C++20): its own pose source (scripted
  ground route, or its own ESKF-style estimate), subscribing to what the agents
  send instead of to `/fusion/tracks` directly.
- **Its own view**: the device runs the same visibility model as the detector
  (FOV, range, ray-cast occlusion from the world file), so for every track it
  knows whether it could see the entity itself. The headline visual is the
  entity drawn at a position the device has no line of sight to.
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
  "fraction of time the device holds a track on an entity it cannot see".

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
- **Headline curves**: end-to-end latency (entity first visible to any agent
  -> track on the device) and position error, each as a function of link
  quality (bandwidth, loss rate, latency), with the Phase 1 perfect link as the
  reference point.

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
  model on a simulated RGB camera). It has to publish the same
  `/agent_<n>/detections` contract: one `DetectionArray` per frame, even when
  empty, stamped with the capture time, world positions with full covariances.
- Several entities, clutter and false detections, where the swappable
  associator (nearest neighbour vs Hungarian today) starts to matter.
