# Architecture

How the pieces fit, what talks to what, and the conventions every node
follows. Phase-specific design and results are in the phase write-ups.

## Nodes and topics (Phase 1)

```mermaid
flowchart LR
  subgraph SIM["Simulation (scripts/sim.sh)"]
    GZ["Gazebo Harmonic<br/>world coop_field"]
    P1["PX4 SITL instance 1"]
    P2["PX4 SITL instance 2"]
    XA["MicroXRCEAgent<br/>UDP 8888"]
    GT["ground_truth_bridge"]
    CB["parameter_bridge<br/>/clock"]
    EM["entity_mover.py"]
  end
  subgraph A1["/agent_1"]
    E1["eskf"]
    D1["detector"]
    O1["offboard"]
  end
  subgraph A2["/agent_2"]
    E2["eskf"]
    D2["detector"]
    O2["offboard"]
  end
  subgraph FUS["Fusion"]
    F["/fusion/track_fusion<br/>(agents 1+2)"]
    B1["/baseline_agent_1/track_fusion"]
    B2["/baseline_agent_2/track_fusion"]
  end
  GZ <--> P1 & P2
  P1 & P2 <--> XA
  XA -- "/px4_1/fmu/out/*" --> E1 & O1
  XA -- "/px4_2/fmu/out/*" --> E2 & O2
  O1 -- "/px4_1/fmu/in/*" --> XA
  O2 -- "/px4_2/fmu/in/*" --> XA
  GZ -. "pose/info" .-> GT
  GZ -. "clock" .-> CB
  EM -. "set_pose" .-> GZ
  GT -- "/sim/ground_truth" --> D1 & D2
  E1 -- "eskf/odometry" --> D1
  E2 -- "eskf/odometry" --> D2
  D1 -- "/agent_1/detections" --> F & B1
  D2 -- "/agent_2/detections" --> F & B2
  F --> OUT["/fusion/tracks, /fusion/markers"]
```

Dotted lines are simulation-only plumbing: nothing on the perception side
depends on them except through `/sim/ground_truth`, which only the synthetic
detector (to decide what its camera can see) and the evaluation read.

| topic | type | from | to | notes |
|---|---|---|---|---|
| `/clock` | rosgraph_msgs/Clock | Gazebo via ros_gz_bridge | every node | all nodes run with `use_sim_time` |
| `/px4_<n>/fmu/{in,out}/*` | px4_msgs | PX4 instance n via XRCE agent | offboard, eskf | PX4 namespaces instance n itself |
| `/sim/ground_truth` | tf2_msgs/TFMessage | ground_truth_bridge | detectors, evaluation | world poses of `entity`, `agent_<n>`, stamped with sim time; also on `/tf` for RViz |
| `/sim/scene_markers` | visualization_msgs/MarkerArray | ground_truth_bridge | RViz | occluders read from the world file, latched |
| `/agent_<n>/eskf/odometry` | nav_msgs/Odometry | eskf | detector | ENU/FLU, relative to the agent's start point |
| `/agent_<n>/detections` | coop_msgs/DetectionArray | detector | fusion | **the interface a camera detector must keep** |
| `/agent_<n>/visibility_truth` | coop_msgs/VisibilityTruth | detector | evaluation | ground truth, never on a link |
| `/fusion/tracks` | coop_msgs/TrackArray | fusion | (Phase 2: the device) | all tracks predicted to publish time |
| `/fusion/markers` | visualization_msgs/MarkerArray | fusion | RViz | coloured by agents contributing, 2-sigma ellipse |
| `/baseline_agent_<n>/tracks` | coop_msgs/TrackArray | baseline fusion | evaluation | the same tracker fed by agent n only |
| `/diagnostics` | diagnostic_msgs/DiagnosticArray | eskf, fusion | evaluation | real-time counters, named by node |

## Frames

- **world**: Gazebo's world frame, ENU (x east, y north, z up), metres. Every
  detection and track is in it. There is no per-message `frame_id`: the
  message set is world-frame by definition, which saves a string per message.
- **PX4 local NED**, per agent: only at the PX4 boundary (offboard setpoints,
  EKF2's local position). Each instance's origin is its own start point.
- **ESKF output**: ENU/FLU relative to where the ESKF initialised, which is the
  agent's start point. The detector adds that surveyed start position
  (`origin_world_enu`, from the scenario) to place the agent in the world.
- **Body**: FLU, as Gazebo models and REP-103 odometry use. The camera is the
  body frame pitched down by its mount angle.

## Time

Everything runs on simulation time from `/clock`. Stamps mean "when the thing
happened": a detection carries the time of the ground-truth sample it was made
from, and a track array carries the time its tracks were predicted to.
PX4's own timestamps are a different time base (PX4 time plus the uXRCE-DDS
timesync offset, which steps; see quad-autonomy-sim's Phase 2 write-up), so no
node here uses them for anything but ordering inside the ESKF. Messages sent
*to* PX4 are stamped 0 ("stamp on arrival"), the fix quad-autonomy-sim found
for offboard setpoints looking stale.

## Real-time discipline

The fusion node and the ESKF share one design and one set of instruments
(`agent_estimation`'s `TimingStats`, `FixedRing`, `alloc_probe`):

- a dedicated ingest/sensor thread whose callbacks do bounded work on
  fixed-capacity storage and never allocate;
- a separate output thread for everything that allocates (messages, markers,
  logging, parameter services);
- a hand-off from the first to the second with `try_lock`, so the hot path
  never blocks;
- live counters on `/diagnostics`: callback time histogram (mean, p99, max),
  overruns against a budget, and heap allocations counted by a replaced
  `operator new`, both inside our callbacks and on the whole thread.

## Messages (`coop_msgs`)

Small on purpose, because Phase 3 pushes them through a bandwidth-limited
link model. Measured CDR sizes:

| message | size |
|---|---|
| `DetectionArray` | 24 bytes + 44 per detection (a frame with one detection is 68 bytes) |
| `TrackArray` | 20 bytes + 64 per track |

A `Detection` is class, confidence, world position and the 6 unique entries of
its position covariance. `DetectionArray` adds the capture time, the source
agent and a per-agent frame counter (so a receiver can count frames the link
lost). `Track` carries id, class, status, a bit mask of the agents that
contributed recently, last update time, position, velocity and position
covariance. `VisibilityTruth` is evaluation-only.

## Packages

| package | language | what |
|---|---|---|
| `coop_msgs` | msg | the message set above |
| `agent_offboard` | C++20 | offboard flight node (copied from quad-autonomy-sim) |
| `agent_estimation` | C++20 | the ESKF (copied from quad-autonomy-sim) and the shared real-time instruments |
| `synthetic_detector` | C++20 | sensor model (FOV, range, ray-cast occlusion, noise, dropout), detector node, ground-truth bridge |
| `track_fusion` | C++20 | Kalman filter, associators, tracker, reorder buffer, fusion node |

Each C++ package keeps its logic in a ROS-free library with unit tests, and
the node is a thin I/O layer around it.
