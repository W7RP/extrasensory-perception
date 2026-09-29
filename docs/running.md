# Running it

Setup, the scored demo, and how to run each piece by hand. Tested on WSL2 with
Ubuntu 22.04; exact versions are in [environment.md](environment.md).

## Setup

This project reuses the toolchain of
[quad-autonomy-sim](https://github.com/W7RP/quad-autonomy-sim) rather than
installing its own: ROS 2 Humble, PX4 v1.17.0 SITL at `~/PX4-Autopilot`
(with that repo's two PX4 patches), Gazebo Harmonic 8.15, the Micro XRCE-DDS
agent in `~/.local`, and the ros_gz bridge. On a fresh machine, run
quad-autonomy-sim's `scripts/setup/0{1,2,3,4}_*.sh` first; they need sudo for
the apt steps.

Then, in this repo (no sudo anywhere):

```bash
./scripts/setup/apply_px4_patches.sh   # PX4 patches + the PX4-OpticalFlow fix (idempotent)
./scripts/check_env.sh                 # verifies every version; installs nothing
./scripts/setup/build_workspace.sh     # fetches px4_msgs if needed, colcon build
source scripts/env.sh                  # in every new shell
```

`apply_px4_patches.sh` matters if you run more than one agent: without the
third patch, every simulated optical-flow sensor in a Gazebo server shares one
feature tracker, and the agents' flow is garbage as soon as they fly
([firmware/README.md](../firmware/README.md)).

Unit tests:

```bash
cd ros2_ws && colcon test && colcon test-result --verbose
```

## The scored demo

```bash
./scripts/demo_phase1.sh             # headless (the default)
./scripts/demo_phase1.sh --gui       # with the Gazebo GUI and RViz
POSE_SOURCE=ground_truth ./scripts/demo_phase1.sh   # detectors get perfect navigation
```

It owns the whole run. It first stops any stale Gazebo, PX4, XRCE agent or
project node (from an earlier run, or from another project using the same
simulator), then starts the simulator, waits for each stage by polling for real
messages, flies both agents, records a rosbag, scores the flight and tears
everything down, including on Ctrl-C. It exits 0 only if both agents completed
their routes and the fused track met the thresholds in
`scripts/eval_phase1.py`. A run takes about three and a half minutes.

Output goes to `logs/phase1_<timestamp>/`:

| file | what |
|---|---|
| `results.txt` | the results table, as printed |
| `metrics.json` | every metric, per tracker, plus the final `/diagnostics` of every node |
| `phase1.png` | top-down view, error over time, track existence, visibility timeline |
| `coop_bag/` | the rosbag the evaluation read (`python3 scripts/eval_phase1.py <logdir>` re-scores it) |
| `params/` | the exact parameter file each node ran with |
| `*.log` | one log per process (PX4 instances, Gazebo, every node) |

## The Phase 2 demo: the device's see-through view

```bash
./scripts/demo_phase2.sh             # headless, scored
./scripts/demo_phase2.sh --gui       # Gazebo, and RViz with the device's camera view live
RECORD_IMU=1 ./scripts/demo_phase2.sh   # also record the agents' raw PX4 IMU streams
```

Same lifecycle as Phase 1, plus the ground device: it walks west of the
building and round its south end while the agents fly, fuses their
detections, and draws every track on its own camera image. Output, in
`logs/phase2_<timestamp>/`:

| file | what |
|---|---|
| `overlay.mp4` | the device's annotated camera view, the whole flight |
| `frames/` | a PNG still every 2 s of simulation time |
| `phase2_frames.png` | three typical stills: seen through the wall, predicted, and the device's own view |
| `phase2.png` | per-frame timeline: hidden or visible to the device, agents seeing, overlay drawn, pixel error |
| `results.txt`, `metrics.json` | overlay and track metrics against ground truth |

In the image, cyan means "behind the wall, and the agents see it right now",
green "the device can see it itself", grey dashed "predicted, nobody sees it";
the dashed magenta box is the true position (simulation only).

## By hand

Three terminals, each with `source scripts/env.sh`:

```bash
./scripts/sim.sh          # 1: world + both PX4 agents + bridges (--gui for the Gazebo GUI)
./scripts/stack.sh --rviz # 2: ESKFs, detectors, fused + baseline trackers, RViz
./scripts/fly.sh          # 3: the entity starts walking, both agents fly their routes
```

For Phase 2, use `./scripts/stack.sh --device --rviz` in terminal 2: RViz
then shows the device's see-through view next to the map (or run
`ros2 run rqt_image_view rqt_image_view /device/overlay/image`).

`sim.sh` refuses to start if a simulator is already running (a stale one
silently shares topics with the new one). In RViz, the building is grey, the
TF frames are the true positions of the entity and the agents, and the fused
track is a sphere with its 2-sigma ellipse: green while both agents see the
entity, amber for one, grey while it coasts on prediction alone. The
single-agent baselines are there too, off by default.

Useful while it runs:

```bash
ros2 topic echo /fusion/tracks
ros2 topic echo /agent_1/detections
ros2 topic echo /diagnostics          # timing and allocation counters, per node
```

## Designing a scenario

Everything that depends on where things are comes from one file,
`sim/scenarios/two_agent_wall.yaml`: spawn poses, routes, the entity's path,
detector and fusion settings. Before spending simulator time on a new route,
preview its visibility timeline:

```bash
python3 scripts/lib/scenario.py preview sim/scenarios/two_agent_wall.yaml
```

It flies the routes at constant speed with the same visibility rules as the
detector and prints how often nobody, one or both agents see the entity. Pass
another scenario with `--scenario` to `demo_phase1.sh`, `sim.sh` and
`stack.sh`, or as the first argument of `fly.sh`.
