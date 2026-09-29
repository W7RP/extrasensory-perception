# firmware/

PX4 configuration for the simulated agents. The PX4 source is not vendored:
it is quad-autonomy-sim's checkout at `~/PX4-Autopilot` (PX4 v1.17.0).
`scripts/setup/apply_px4_patches.sh` applies the patches below, idempotently,
and `scripts/check_env.sh` verifies them.

## Parameters

| file | what |
|---|---|
| `params/common.params` | offboard-loss behaviour, auto-disarm, DDS domain, IMU integration rate |
| `params/sitl_only.params` | SITL relaxations: no ground station, PX4's WMM magnetometer simulator, pinned magnetometer calibration, the simulated world's declination |

Both are copied unchanged from quad-autonomy-sim, where each value was found
the hard way; each line carries its reason. `scripts/sim.sh` applies them to
every agent at every boot.

The airframe is PX4's stock `4021_gz_x500_flow`: downward optical flow and
rangefinder, no GPS. EKF2 flies on flow, and so does each agent's own ESKF.

## Patches (`px4_patches/`)

| patch | applies to | why |
|---|---|---|
| `0001-uxrce-dds-export-estimation-topics.patch` | PX4 | exports the ESKF's inputs (flow, rangefinder, magnetometer) and the SITL ground-truth topics over uXRCE-DDS (quad-autonomy-sim) |
| `0002-gz-bridge-use-wmm-mag-sim.patch` | PX4 | uses PX4's magnetometer simulator instead of Gazebo Harmonic's, which is wrong under tilt (quad-autonomy-sim) |
| `0003-px4-opticalflow-per-instance-state.patch` | PX4-OpticalFlow, PX4's simulated flow sensor library, fetched into `build/px4_sitl_default/OpticalFlow` | **new in this project**: makes the flow sensor work with more than one vehicle per Gazebo server |

### 0003: two agents, one feature tracker

PX4's simulated flow sensor is a Gazebo plugin that runs OpenCV KLT tracking
on a downward camera. Its tracker, `klt_feature_tracker/trackFeatures.cpp`,
kept the previous image and the tracked corners in file-level `static`
variables, and `OpticalFlow::limitRate` kept its last publish time in a
function-local `static`. Every flow sensor in the Gazebo server process shares
those. With one vehicle that is invisible. With two, each sensor tracked
features from the other vehicle's last image, and each sensor's rate limiter
was reset by the other's publications.

In the first Phase 1 flights, both agents were fine on the ground (both
cameras see nearly the same texture at rest), then diverged as soon as they
flew: EKF2's velocity estimate fell behind, both agents accelerated east to
over 20 m/s, PX4 declared the setpoints invalid, blind-landed, and flipped.
The ESKF, which gates flow on its innovation, rejected most flow samples from
takeoff on (690 rejected, 409 fused; flying alone: 1845 fused, 0 rejected).
The same agent flying alone in the same world was fine, which pointed at
shared state.

The patch moves the tracker's memory into a `KltState` owned by each
`OpticalFlowOpenCV` instance (the old single-stream function is kept as a
wrapper), and makes the rate limiter's timestamp a member. It changes a class
layout, so both `libOpticalFlow.so` and PX4's `libOpticalFlowSystem.so`
Gazebo plugin are rebuilt; PX4 itself is not. With one vehicle the behaviour is
identical, checked by re-running quad-autonomy-sim's Phase 2 demo afterwards.
It is a candidate for an upstream fix.

The patches modify PX4 and PX4-OpticalFlow, so they are covered by those
projects' BSD 3-Clause licenses rather than this repository's MIT license.
