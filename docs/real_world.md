# From simulation to the real world

This is a simulation, played like a game in Phase 3, but it is built as a
proof of concept for real hardware. This page is the honest account: what
would carry over unchanged, what is simplified, and which settings stand for
which real-world conditions.

## What carries over

| here | on real hardware | status |
|---|---|---|
| PX4 v1.17 SITL | the same PX4 firmware on each agent's flight controller (e.g. a Pixhawk) | unchanged; SITL-only parameters are listed in `firmware/params/sitl_only.params` |
| ROS 2 nodes (`agent_offboard`, detectors, `track_fusion`, `overwatch`) | the same nodes on each agent's companion computer (e.g. a Jetson) and on the ground device | unchanged code; the uXRCE-DDS link to PX4 becomes serial or Ethernet |
| `coop_msgs` | the messages that cross the radio link | sized for it: 68 bytes for a frame with one detection, 64 bytes per track |
| the detector interface | a camera and a detector (e.g. YOLO) projecting each box into the world with the agent's pose | the synthetic detector publishes exactly this; a real one replaces it without changes downstream |
| fusion, overlay projection, planner | the same code on the device | unchanged |
| the evaluation | field trials scored against surveyed or RTK ground truth | same metrics |

## What is simplified, and how it is (or will be) modelled

| gap | real-world issue | here |
|---|---|---|
| detection | real detectors miss people, see people who are not there, and struggle with light, distance and partial occlusion; one camera measures distance poorly | range-dependent noise (worse along the line of sight) and 10 % missed frames; no false detections yet; next step is YOLO on Gazebo's rendered images, then real footage |
| agent navigation | GPS is off by 1-3 m (a few cm with RTK); every agent's error lands in its detections | the agents' real estimators run (ESKF on flow, or PX4's EKF2 on GPS) and their reported uncertainty is part of each detection; Phase 1 found navigation, not detection, dominates the fused error; registration (estimating each agent's bias) is Phase 2 milestone 3 |
| **device pose** | a phone or tablet knows its heading to a few degrees; 2 deg at 20 m moves the overlay 70 cm | the device's pose is exact today; a pose-error setting is the next realism step, and **this is the largest real-world risk to the see-through effect** |
| the link | limited bandwidth, delay, loss behind buildings and at range, outages | perfect today; Phase 4 is the link model, and its curves are the headline result |
| clocks | every agent and the device need one time base | every message carries its capture time; real hardware would use GPS time or PTP |
| physics | wind, gusts, battery (20-30 min), speed and tilt limits | PX4 flies real multirotor dynamics in Gazebo; wind is available in Gazebo and not yet used |
| people | real people stop, turn, crowd and hide | scripted loops at constant speed |
| rules | altitude limits (about 120 m), line of sight, flying over people | out of scope; worth knowing before any field test |

## Realism settings

The "difficulty" of a session is its realism. Each of these is a parameter in
the scenario or node configuration:

- detector noise (`detector.noise.*`), missed-frame rate (`p_miss`);
- the agents' camera: field of view, pitch, range (`detector.camera.*`); the
  vision radius is a camera spec, not a free number: a detector needs a
  person roughly 20-30 px tall, so resolution and field of view set the range;
- the agents' navigation source (`pose_source`: ESKF, PX4, or perfect);
- the planner's radius and separation (`planner.*`);
- to come: false detections, device pose error, wind, and the link model.

## Rules that keep the game layer honest

1. Ground truth drives only the simulator (where things are, what each camera
   could see) and the scoring. Nothing the device shows or decides uses it.
2. The minimap and the planner use only the map, the device's own position,
   the agents' own reports and the tracks the device holds.
3. Every session, played or scripted, is recorded and scored with the same
   metrics.
