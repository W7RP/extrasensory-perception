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
| detection | real detectors miss people, see people who are not there, and struggle with light, distance and partial occlusion; one camera measures distance poorly | low agents (Phases 1-3): range-dependent noise (worse along the line of sight) and 10 % missed frames. The high unit (Phase 4): detection probability from pixels on target (the Johnson-criteria TTPF, N50 = 6 px for an automated detector on visible imagery), geolocation by ground intersection with a pixel and pointing error. No false detections yet; next step is YOLO on Gazebo's rendered images (the zoom camera already renders), then real footage |
| sensors | a payload's resolution, field of view and pointing accuracy decide what it can see from where | Phase 4's presets are typical of commercial payloads, not any one product: a 4K 60 deg wide camera, a 1080p zoom at 6 deg (the long end of a 30x zoom), a 640x512 thermal camera; pointing 0.05-0.1 deg (a stabilised gimbal with a good INS). Atmospheric haze, motion blur and rolling shutter are not modelled, and all three get worse with range |
| canopy | trees hide people from visible cameras; thermal sees through sparse canopy some of the time | canopies are solid to visible cameras; the thermal preset passes each canopy with probability 0.3 |
| agent navigation | GPS is off by 1-3 m (a few cm with RTK); every agent's error lands in its detections | the agents' real estimators run (ESKF on flow, or PX4's EKF2 on GPS) and their reported uncertainty is part of each detection; SITL's GPS has no absolute error, so `nav_bias` adds a constant, seeded one (1.5 m horizontal, 2.5 m vertical for plain GPS). From a high unit a vertical error also moves the ground intersection, by dz / tan(depression). Phase 1 found navigation, not detection, dominates the fused error; registration (estimating each agent's bias) is Phase 2 milestone 3 |
| **device pose** | a phone or tablet knows its heading to a few degrees; 2 deg at 20 m moves the overlay 70 cm | the device's pose is exact today; a pose-error setting is the next realism step, and **this is the largest real-world risk to the see-through effect** |
| the link | limited bandwidth, delay, loss behind buildings and at range, outages | perfect today; Phase 5 is the link model, and its curves are the headline result |
| clocks | every agent and the device need one time base | every message carries its capture time; real hardware would use GPS time or PTP |
| physics | wind, gusts, battery (20-30 min), speed and tilt limits | PX4 flies real multirotor dynamics in Gazebo; wind is available in Gazebo and not yet used |
| people | real people stop, turn, crowd and hide | scripted loops at constant speed |
| rules | altitude limits (about 120 m in most places), line of sight, flying over people | out of scope, but it bounds Phase 4's sweep: 150 and 250 m are there to show where the cameras run out, not as a flight plan |
| endurance | one high multirotor flies 20-40 min; covering a session means swapping or tethering | not modelled; a tethered drone (power over the cable, hours aloft at 50-100 m) is the usual answer for a fixed overwatch |

## Realism settings

The "difficulty" of a session is its realism. Each of these is a parameter in
the scenario or node configuration:

- detector noise (`detector.noise.*`), missed-frame rate (`p_miss`);
- the agents' camera: field of view, pitch, range (`detector.camera.*`); the
  vision radius is a camera spec, not a free number: a detector needs a
  person roughly 20-30 px tall, so resolution and field of view set the range;
- the agents' navigation source (`pose_source`: ESKF, PX4, or perfect) and
  its absolute error (`nav_bias`: GPS or RTK grade);
- the high unit's sensors (`detector.cameras`: presets from imaging.cpp), its
  altitude and camera pitch;
- the planner's radius and separation (`planner.*`);
- to come: false detections, device pose error, wind, haze, and the link model.

## Rules that keep the game layer honest

1. Ground truth drives only the simulator (where things are, what each camera
   could see) and the scoring. Nothing the device shows or decides uses it.
2. The minimap and the planner use only the map, the device's own position,
   the agents' own reports and the tracks the device holds.
3. Every session, played or scripted, is recorded and scored with the same
   metrics.
