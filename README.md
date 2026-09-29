# coop-perception-sim

A ground device that can see through walls, because agents in the air see for
it. Several airborne agents watch a scene, detect entities of interest on
board, and send compact results to a device on the ground as fast as they
can. The device fuses them and highlights, on its own camera view, an entity
that is hidden from where it stands: a box, an outline, or a skeleton drawn
over the wall where the entity really is, like an "ESP" view in a game but
built from real sensor data.

The project is both halves of that: the see-through overlay itself, and a
measurement of how fast and how accurately it follows the real entity as the
link between the agents and the device degrades.

Everything runs on the same stack a real vehicle would: **PX4** SITL for
flight control, **Gazebo Harmonic** for physics and sensors, and **ROS 2**
nodes in C++20 for navigation, detection and fusion. There is no hardware.

![Playing the device: a person behind a building, highlighted from what the agents see](docs/media/phase3_game.png)

*Phase 3, playing the device. You walk the map with WASD; the grey building
in front of you hides a person, drawn where they really are (cyan: an agent
sees them right now, 0.2 s ago) with a fainter grey prediction of another
behind it. The minimap shows the map, you and your view (white and green),
the two agents placed around you by the planner and their views (blue), the
tracks, and the fog of war: ground only an agent sees is blue, ground nobody
sees is dark.*

Can it work for real? It is a simulation, but built as a proof of concept on
the same software you would fly: [what carries over, and what does
not](docs/real_world.md).

## Phase 1: two agents, one fused track

Two agents fly scripted routes on either side of a building while the entity
walks behind it. At any moment nobody, one, or both of them can see it. Each
agent localises itself with its own error-state Kalman filter and detects the
entity with a synthetic detector, and a tracker fuses both agents' detections
into one track. The same tracker fed by one agent at a time is the baseline.
The link is perfect in this phase.

![Phase 1: the scene from above, track error over time, and who could see the entity when](docs/media/phase1_fusion.png)

*Phase 1, one scored run. Left: the building (grey), the entity's path behind
it, the two agents' routes on either side, and the fused track, coloured by
how many agents were seeing the entity (green both, amber one, grey nobody:
the track is coasting on prediction). Right: horizontal error of the fused
track and of each agent alone, when each tracker had a track at all, and the
true visibility timeline.*

Over three scored runs (a fourth, with the GUI, is in the same range; a
fifth failed, see "Things that broke"):

| | **fused** | agent 1 alone | agent 2 alone |
|---|---|---|---|
| track continuity (share of the time someone could see the entity) | **0.985-0.991** | 0.86-0.88 | 0.69-0.72 |
| position error while visible (horizontal RMS) | **0.39-0.55 m** | 0.53-0.64 m | 0.45-0.70 m |
| ID switches per run | **0-1** | 2 | 1 |
| time to first track | 0.23-0.40 s | 0.30-0.42 s | 0.24-2.10 s |

The biggest gain from fusion is coverage. Either agent alone loses the entity
for an eighth to a third of the time some agent could see it; together they
almost never do, and the fused track coasts through the gaps where nobody sees
it. Accuracy improves too, but less than it could, for a reason worth its own
bullet below. On the real-time side, the fusion node's ingest callback
averaged 3-5 us (worst case 222 us, in the GUI run, against a 500 us budget), with zero heap
allocations in its callbacks.

The full write-up, with metric definitions and every finding, is
[docs/phase1_two_agent_fusion.md](docs/phase1_two_agent_fusion.md).

## Phase 2 (in progress): the device sees through the wall

A ground device now walks west of the building, where the building hides the
entity, and round its south end, where it can see it directly. It fuses what
the agents send and draws every track on its own camera image, where the
entity really is: a box and a person outline, cyan when the agents see it
right now, green when the device could see it anyway, grey and dashed when
it is only a prediction.

![The device's camera: seen through the wall, predicted, and in its own view](docs/media/phase2_see_through.png)

*The device's camera in one scored run. Left: the entity is behind the
building, two agents see it, and it is drawn where it is (the dashed magenta
box is the true position, drawn for checking only). Middle: nobody has seen
it for 3.4 s, so it is a grey prediction with its uncertainty on the ground.
Right: the device has walked round the corner, and the overlay sits on the
real person.*

Over three scored runs, whenever the entity was hidden from the device but
visible to an agent, it was highlighted in 99 % of the camera frames, 5-10 px
(median) from where it really was, from data 0.06-0.13 s old. That is the
baseline Phase 4's degraded link will be measured against. The skeleton
overlay, registration of the agents' navigation bias, and a comparison of
where to fuse are the next milestones
([write-up](docs/phase2_device_overlay.md)).

## Phase 3: the playable scene

You are the device now. `scripts/demo_phase3.sh --play` opens a game window
on a map generated from a seed (buildings, walls, shipping containers, crates
and trees, in colour) where several people walk their own loops. You walk it
with WASD and the mouse. Two GPS agents fly above the rooftops, re-tasked
every second by a planner that places them around you where, between them,
they see the most ground you cannot. Whatever they see is drawn on your view
through the walls, and the minimap shows who can see what.

![A generated map: map 7](docs/media/phase3_map.png)

Over three scored sessions with four people on the map: whenever an agent
could see someone you could not, they were highlighted on your view 93-96 %
of the time, a median 7-9 px from where they really were, from data about a
tenth of a second old. The device's fused picture of all four scored a MOTA
(multi-object tracking accuracy) of 0.96-0.97, against 0.40-0.68 from either
agent alone. The honest limits: two agents cover only 35-43 % of the ground
hidden from you at any moment, and people who turn a corner while nobody
watches often come back under a new track id.

The same session runs scripted and headless as a scored benchmark, and a
played session is scored the same way when you press Esc. Nothing you see in
the game uses ground truth: the overlay, the minimap and the planner work
from the map, the device's own position, the agents' own reports and the
tracks ([write-up](docs/phase3_playable_scene.md)).

## Why it's built this way

- **The detector interface is the real one.** The synthetic detector computes
  what a camera could see from ground truth (field of view, range, a ray cast
  against the building, noise, missed frames), but it publishes exactly what
  a camera detector would: world-frame positions with covariances, one message
  per frame, empty or not. A YOLO-on-camera detector can replace it without
  touching anything downstream.
- **Navigation error is part of perception.** Each detection is placed in the
  world with the agent's *own* position and attitude estimate, not the true
  one, and the estimator's uncertainty is part of its covariance. That turned
  out to matter more than detector noise (below).
- **The world file is the occlusion model.** The detector reads the building
  straight out of the Gazebo world with libsdformat, so what blocks the view
  in the physics is exactly what blocks it in the detector.
- **Small messages, from day one.** A frame with one detection is 68 bytes on
  the wire, a track 64. Phase 4 pushes these through a bandwidth-limited link,
  so the format was designed for that before there was a link.
- **Real-time discipline, measured.** The fusion node has a dedicated ingest
  thread, fixed-capacity storage, a non-blocking hand-off, and live counters
  for callback time and heap allocations. Its callbacks averaged 3-5 us,
  and the counter read zero allocations in every run.
- **Scored, not eyeballed.** Every run ends with an evaluation against
  Gazebo's ground truth, with thresholds set in advance, comparing the fused
  track against each agent on its own.

## Reused from quad-autonomy-sim

This project builds on my earlier
[quad-autonomy-sim](https://github.com/W7RP/quad-autonomy-sim) and copies two
pieces of it into this repo. **agent_offboard** is its offboard flight node,
which takes off, flies a waypoint route with velocity setpoints and lands.
**agent_estimation** is its hand-rolled C++20 ESKF (IMU, optical flow,
rangefinder, magnetometer), along with the real-time instruments built for
it. Both were designed and validated there; the changes for running several
agents are listed in [docs/estimation.md](docs/estimation.md). The toolchain
(PX4 build, Gazebo, ROS 2, the XRCE agent) is the one that project installed,
and its two PX4 patches and parameter files are reused as-is.

## Things that broke

- **Both agents ran away as soon as they took off.** On the ground everything
  looked fine. In the air, both agents' velocity estimates fell behind,
  they accelerated east past 20 m/s, and PX4 blind-landed them. One agent
  alone flew perfectly. The cause was in PX4's simulated optical-flow sensor:
  its feature tracker keeps the previous image and tracked corners in
  file-level `static` variables, shared by every flow sensor in the Gazebo
  process. With two vehicles, each one tracked features across the *other*
  vehicle's images. A small patch gives each sensor its own tracker state
  ([firmware/README.md](firmware/README.md)).
- **Navigation, not detection, dominates the fused error.** One agent's ESKF
  drifted 0.6 m early in the flight while reporting 0.16 m of uncertainty, so
  all its detections landed 0.6 m east of the entity with too-small
  covariances. Fusion then puts the track between the two agents' biases.
  With perfect navigation the same scenario gives a fused error of 0.22 m,
  half of what it is with the agents' own estimates.
  Estimating each agent's bias from jointly seen entities is on the roadmap.
  The worst case turned out to be a timing bug, not an estimator
  limitation (next bullet).
- **The agents' IMU data "had gaps" that weren't there.** In 1 of 6 Phase 1
  runs, one agent's estimate jumped by a metre while it still claimed 0.24 m
  of uncertainty, and fusion, trusting it, did worse than the other agent
  alone. In Phase 2, with a third camera rendering, the gaps showed up in
  every run, and in one the overlay stood a metre beside the real person. Recording the raw IMU
  stream showed that no sample was ever missing: uXRCE-DDS was re-mapping
  PX4's clock, which in simulation is simulation time, onto the wall clock,
  and correcting the mapping in steps whenever the simulation ran slower
  than real time. One parameter (`UXRCE_DDS_SYNCT 0`) keeps PX4's stamps on
  simulation time. The gaps went to zero, and all four scored runs since
  (three Phase 2, one Phase 1) have passed.
- **One outlier could steal a track.** A unit test found that a single
  4-sigma detection starts a tentative track whose wide covariance makes the
  next real detections look closer to it than to the established track.
  Association now ranks by likelihood instead of distance, and established
  tracks pick first.
- **Tracks died while their predictions were still good.** Deleting coasting
  tracks at 4 m of uncertainty killed them about 4 s into an occlusion, when
  the prediction was still within 0.6 m. That cost an ID switch each time the
  entity reappeared. The limit is now 8 m, and the 10 s timeout decides.
- **The simulation ran at a fifth of real time.** Two 50 Hz flow cameras
  rendering shadows headless was too much. Turning shadows off brought it
  back to about 0.9x.
- **Parameter files silently did nothing.** A node name without its
  namespace (`detector:`) doesn't match `/agent_1/detector`, and ROS 2
  ignores the mismatch without an error. The first detectors started with no
  world file, and the ESKFs listened to the wrong PX4.

## How it fits together

```mermaid
flowchart LR
  subgraph SIM["Simulation"]
    GZ["Gazebo Harmonic<br/>building + entity"]
    PX["PX4 SITL x2<br/>/px4_1, /px4_2"]
  end
  subgraph AG["Per agent (/agent_n)"]
    ESKF["ESKF<br/>navigation"]
    DET["synthetic detector"]
    OFF["offboard<br/>scripted route"]
  end
  FUS["track_fusion<br/>fused + per-agent baselines"]
  DEV["device + see-through overlay<br/>(Phase 2)"]
  GZ <--> PX
  PX -- "IMU, flow, range, mag" --> ESKF
  ESKF -- "pose + covariance" --> DET
  GZ -. "ground truth<br/>(what the camera could see)" .-> DET
  OFF -- "setpoints" --> PX
  DET -- "DetectionArray<br/>(world frame)" --> FUS
  FUS -- "TrackArray" --> DEV
```

| package | what |
|---|---|
| `coop_msgs` | compact detection and track messages |
| `agent_offboard` | offboard flight node (from quad-autonomy-sim) |
| `agent_estimation` | the ESKF (from quad-autonomy-sim) and the shared real-time instruments |
| `synthetic_detector` | sensor model, detector node, Gazebo ground-truth bridge |
| `track_fusion` | Kalman filter, swappable associators, tracker, fusion node |
| `device_view` | the device's see-through overlay, its controller, and the game window |
| `overwatch` | the planner that places the agents around the device |

Nodes, topics, frames and timing are in
[docs/architecture.md](docs/architecture.md).

```
firmware/     PX4 parameters and patches (incl. the multi-vehicle optical-flow fix)
sim/          worlds (hand-made and generated), models, scenarios, map generator, RViz views
ros2_ws/src/  the seven packages above
scripts/      simulator launcher, demo, evaluation, environment checks
docs/         architecture, per-phase write-ups, roadmap, environment
```

## Roadmap

| phase | what it adds | status |
|---|---|---|
| 1. Two-agent fusion | two agents, one hidden entity, perfect link, one fused track, scored against ground truth | done, tag `phase1-two-agent-fusion` |
| 2. The device | a ground node with its own pose, camera and occlusion-aware view, fusing what the agents send and drawing the see-through overlay (box, outline, skeleton) on its camera image | in progress: device, fusion and box + outline overlay done; skeleton, registration and fusion placement next |
| 3. The playable scene | generated colourful maps, several people, you drive the device in a game window, agents placed around you by a planner | done |
| 4. The link | bandwidth caps with priority queues, latency, jitter, range- and occlusion-dependent loss, outages; latency and accuracy against link quality are the headline result, with the overlay degrading from skeleton to box as the link gets worse | planned |
| 5. More agents, harder maps | 3-6 agents, planning with lookahead, larger and denser maps, people who react | planned |

The plan for each is in [docs/roadmap.md](docs/roadmap.md).

## Running it

Built on WSL2 with Ubuntu 22.04, ROS 2 Humble, PX4 v1.17.0 and Gazebo
Harmonic 8.15, reusing quad-autonomy-sim's toolchain. Setup, manual runs and
the output files are in [docs/running.md](docs/running.md). Exact versions
are in [docs/environment.md](docs/environment.md).

**Phase 1.** `./scripts/demo_phase1.sh` runs the whole thing headless and
scores it (`--gui` adds the Gazebo GUI and RViz). It clears any stale
simulator first, and exits 0 only if both agents completed their routes and
the fused track met its thresholds. It takes about three and a half minutes.

**Phase 2.** `./scripts/demo_phase2.sh` flies the same mission with the
ground device and scores its see-through overlay (`--gui` shows the device's
view live in RViz). The log directory gets `overlay.mp4`, the device's
annotated camera view for the whole flight, and a still every 2 s.

**Phase 3.** `./scripts/demo_phase3.sh --play` to play (WASD to walk, Q/E
or the right mouse button to turn, Shift to run, Esc to end);
`./scripts/demo_phase3.sh` for the scored, headless session. `--seed N` picks
another map, generated on first use.

**Phases 4-5.** Not built yet.

## License

MIT, see [LICENSE](LICENSE). The patches in `firmware/px4_patches` modify PX4
and PX4-OpticalFlow and stay under their BSD 3-Clause licenses.
