# Phase 1: two agents, one fused track (design record)

**Status: done, tag `phase1-two-agent-fusion`.** Two simulated agents fly
scripted routes on either side of a building while an entity of interest
walks behind it. Each agent localises itself with its own ESKF and detects
the entity with a synthetic detector (field of view, range, ray-cast
occlusion, noise, dropout). A C++20 tracker fuses both agents' detections into
one track, and the same tracker fed by one agent at a time is the baseline.
Everything is scored against Gazebo's ground truth. The link is perfect;
degrading it is Phase 3.

## Results

Three scored runs (A-C), headless, detectors on ESKF navigation, plus one
diagnostic run (D) with perfect navigation. The entity was visible to nobody /
one / both agents 43-44 / 33-34 / 23-25 % of the time in every run. Each cell
is A / B / C.

| metric (horizontal) | **fused** | agent 1 alone | agent 2 alone |
|---|---|---|---|
| position RMSE while some agent sees it | **0.55 / 0.48 / 0.39 m** | 0.64 / 0.53 / 0.60 m | 0.70 / 0.64 / 0.45 m |
| position RMSE while nobody does (coasting) | 0.64 / 0.94 / 0.97 m | 0.71 / 1.12 / 1.09 m | 0.93 / 1.05 / 1.19 m |
| position RMSE, all samples | **0.58 / 0.64 / 0.59 m** | 0.66 / 0.72 / 0.73 m | 0.75 / 0.76 / 0.70 m |
| track continuity (of visible time) | **0.991 / 0.991 / 0.985** | 0.879 / 0.863 / 0.868 | 0.692 / 0.690 / 0.719 |
| ID switches | **0 / 0 / 1** | 2 / 2 / 2 | 1 / 1 / 1 |
| time to first track | 0.40 / 0.32 / 0.23 s | 0.30 / 0.32 / 0.42 s | 2.10 / 1.82 / 0.24 s |
| NEES (ideal 3) | 4.2 / 3.1 / 3.4 | 1.5 / 1.5 / 2.5 | 6.9 / 6.5 / 3.6 |
| false confirmed tracks | 0 / 0 / 0 | 0 / 0 / 0 | 0 / 0 / 0 |

| run D, perfect navigation | **fused** | agent 1 alone | agent 2 alone |
|---|---|---|---|
| position RMSE while some agent sees it | **0.22 m** | 0.47 m | 0.30 m |
| continuity / ID switches | 0.991 / 0 | 0.870 / 2 | 0.712 / 1 |
| NEES | 1.8 | 1.9 | 1.7 |

The agents' own navigation (ESKF vs ground truth, horizontal RMSE over the
flight): agent 1 0.23 / 0.25 / 0.23 m, agent 2 0.54 / 0.41 / 0.22 m. Their
detections: 0.50-0.77 m RMS error, NEES 1.8-2.1 for agent 1 and 3.0-5.8 for
agent 2.

Real time, measured live over each run (~2,040 ingest callbacks for the fused
node, ~1,020 per baseline):

| | callback mean | p99 | max | budget | overruns | heap allocations in our callbacks |
|---|---|---|---|---|---|---|
| fusion node, ingest (A / B / C) | 4.7 / 5.1 / 3.0 us | 20 us | 70 / 42 / 109 us | 500 us | 0 | **0** |
| ESKF, IMU (both agents, A-C) | 9.3-9.9 us | 30-35 us | 100-215 us | 1000 us | 0 | **0** |

No batch was ever late, the reorder buffer never held more than three, and
the output thread's lock was busy for at most 3 hand-offs per run.

How to read it:
- **Fusion buys coverage above all.** Continuity goes from 0.69-0.88 for
  either agent alone to 0.99, and the fused track survived every occlusion gap
  but one, where the single-agent trackers lost it one to two times per run.
  In two of the runs agent 2 first saw the entity about 1.5 s after agent 1;
  its own tracker had to wait for that, the fused track did not.
- **Accuracy improves, but less than it could.** The fused track beat the
  better single agent in every run, by 0.05-0.09 m while visible. With perfect
  navigation (run D) the gain is 0.08 m on the better agent and 0.25 m on the
  worse, and the fused error is half what it is with ESKF navigation. The difference is navigation bias (see
  "Findings"): each agent's ESKF error is common to all of that agent's
  detections, and the tracker cannot average it away.
- **Consistency follows the same story.** With perfect navigation every
  tracker's NEES is 1.7-1.9 (slightly conservative covariances). With ESKF
  navigation, agent 2's detections are overconfident whenever its ESKF drifts
  more than it reports, and the fused NEES rises to 3-4.
- **Coasting error** is dominated by the entity turning round while nobody
  watches: the constant-velocity prediction carries on the wrong way at up to
  0.6 m/s. The evaluation's 2 m matching radius caps it, so a coasting track
  that drifts further counts as lost, not as a large error.
- **Real time holds with a wide margin**: the slowest callback in runs A-C
  was 109 us against a 500 us budget (222 us in the GUI run below).

Two more runs on the final code, one with the GUI, are listed under
"Final verification" at the end.

## Scenario

`sim/scenarios/two_agent_wall.yaml`, world `sim/worlds/coop_field.sdf`.

- **Building**: 2 m thick, 20 m long (north-south), 6 m tall, centred on the
  origin. The only occluder.
- **Entity**: a 1.75 m person-sized shape walking back and forth along the
  building's east face (x = 4, y from -7 to 7) at 0.6 m/s. Detectors aim at its
  centre, 0.9 m up.
- **Agent 1**: starts at (-8, -16), camera facing north, flies east-west legs
  along y = -16 at 4 m and 1.5 m/s. It sees round the building's south end
  once it is east of about x = -1.
- **Agent 2**: the mirror image on the north side (y = 16, facing south), on a
  slower beat (1.0 m/s) and different leg ends, so the two drift in and out of
  phase.

The routes were designed with `scripts/lib/scenario.py preview`, which flies
them kinematically with the detector's own visibility rules. It predicted the
entity visible to nobody / one / both agents 38 / 37 / 26 % of the time; the
flights gave 44 / 33 / 23 % (PX4 accelerates, overshoots and takes off later
than a constant-speed model). The mission is about 80 s from the first
sighting to landing, with three stretches of several seconds where nobody sees
the entity, one of them at the end.

## Detector

`synthetic_detector`: a stand-in for camera + detector that publishes exactly
what a real one would. The model (`sensor_model.hpp`, unit-tested):

1. **Visibility, from ground truth.** The entity's centre must be inside the
   camera's field of view (90 x 60 deg, pitched 15 deg down, facing the agent's
   heading), within 0.5-25 m, and the straight line from the camera to it must
   cross no occluder. Occluders are every box collision of every static model
   in the world file, read with libsdformat, so the detector's geometry is
   Gazebo's geometry.
2. **Dropout.** A visible entity is still missed 10 % of the time per frame.
3. **Measurement.** The true camera-frame vector to the entity, plus Gaussian
   noise that grows with range and is worse along the line of sight (range:
   0.15 m + 2 cm/m; across: 5 cm + 5 mrad), is rotated into the world with the
   agent's **estimated** attitude and added to its **estimated** position (its
   ESKF). The agent's own navigation error ends up in the detection, as it
   would for a real camera.
4. **Covariance.** The noise covariance rotated the same way, plus the ESKF's
   position covariance, plus its attitude variance through the lever arm
   (`[r]x Sigma_att [r]x^T`: half a degree of yaw is 17 cm at 20 m).

At 10 Hz of simulation time it publishes one `DetectionArray` per frame, empty
or not, stamped with the ground-truth sample it was made from. **That is the
contract a camera detector has to keep to replace it**: one message per
processed frame on `/agent_<n>/detections`, stamped with the capture time,
world positions, full covariances. Separately, for evaluation only, it
publishes the ground-truth visibility flags (`VisibilityTruth`).

## Fusion

`track_fusion`, ROS-free core in `kalman.cpp`, `associator.cpp`, `tracker.cpp`,
`reorder_buffer.hpp`, all unit-tested.

**Filter.** A constant-velocity Kalman filter per track: state position and
velocity in the world frame, white-acceleration process noise
(q = 0.5 m^2/s^3, enough to follow a walker turning round), position
measurements with the detection's own covariance, Joseph-form update.

**Per batch** (one agent's frame, in time order): predict every track to the
batch time; gate each detection against each track on the squared
Mahalanobis distance (d^2 <= 16, about 99.9 % for 3 dof); associate; update;
start a tentative track from every detection left over; run the lifecycle.

**Association** is an interface with two implementations, chosen by the
`associator` parameter: greedy nearest neighbour (the default) and optimal
Hungarian assignment. The unit tests include a case where they disagree.
Pairs are ranked by negative log likelihood (d^2 + ln|S|), not by distance
alone, and in two passes: established tracks pick first, tentative ones share
what is left. Both came from a failing unit test (see "Findings").

**Lifecycle.** Tentative until 3 detections (then confirmed), dropped if unseen
for 1 s. A confirmed track coasts after 0.5 s without a detection: it is
predicted forward and its covariance grows, which is what the device should
see. It is deleted after 10 s without a detection or once its horizontal
1-sigma passes 8 m.

**Ordering.** Agents publish independently, so the node holds batches in a
small reorder buffer and releases each once every agent heard from has
published something at least as new (or it is 0.3 s old). The tracker refuses
anything older than what it has processed and counts it. On the perfect link
nothing was ever late; Phase 3 will change that.

**Output.** At 10 Hz, every track predicted to the current time, with a mask
of the agents that contributed in the last 0.5 s, and RViz markers coloured by
that count (green 2, amber 1, grey 0) with a 2-sigma ellipse.

## Real-time design

The fusion node follows the ESKF's design and reuses its instruments:

| requirement | how |
|---|---|
| bounded-time callbacks | fixed-capacity storage everywhere (32 tracks, 16 detections per batch, 32 buffered batches), so the full-table case is the worst case; per-callback timing histogram and overrun count against a 500 us budget |
| no heap allocation in the hot path | fixed-size Eigen and `std::array` only; a replaced `operator new` counts every allocation inside the ingest callbacks, live; a unit test runs 500 batches through the tracker, with both associators, inside the counter and asserts zero |
| explicit threads and callback groups | an ingest thread (one MutuallyExclusive group: the subscriptions, the reorder buffer, the tracker) and an output thread (publishing, markers, diagnostics, parameters) |
| never block the hot path | the ingest thread hands the track table over with `try_lock`; a busy lock skips one copy (counted) |

What "0 allocations" does not cover: rclcpp's own take path on the same thread
allocates the incoming message, about 7 times per message
(`ingest_thread_allocations`). The ESKF avoids most of that with pre-allocated
message pools, but rclcpp only offers those for fixed-size messages, and a
fixed 16-slot detection array would put every empty slot on the wire, which
Phase 3's bandwidth budget rules out. WSL2 is not a real-time OS either: the
p99 and max are what this machine delivered, not guarantees.

## Evaluation

`scripts/eval_phase1.py` reads the run's rosbag and scores three trackers,
identical except for their inputs: fused (both agents), agent 1 alone, agent 2
alone. All three are scored against the same ground truth and the same
visibility timeline (the detectors' own ground-truth flags).

- **Matched track**: at each published sample, the nearest confirmed or
  coasting track within 2 m (horizontally) of the entity.
- **Position RMSE**: horizontal error of the matched track, over every sample
  from 1 s before the first sighting to the end; also split into "some agent
  sees it" and "nobody does" (coasting on prediction).
- **Continuity**: of the time the entity is visible to at least one agent, the
  fraction during which a track is matched to it. The same denominator for all
  three trackers: it measures what the consumer of the picture gets.
- **ID switches**: changes of the matched track's id.
- **Time to first track**: from the entity first becoming visible to any agent
  to the first matched confirmed track.
- **NEES**: mean normalised position error of the matched track (3 = the
  covariance is honest; above 3, overconfident).

### Acceptance (fused tracker, `--check`)

| metric | threshold | reasoning |
|---|---|---|
| continuity | >= 0.90 | each re-acquisition costs ~0.2-0.3 s (3 detections at 10 Hz, 10 % dropout) and there are about four; 0.90 is a failure line with margin |
| RMSE while visible | <= 0.75 m | single detections are ~0.6 m RMS at these ranges (noise plus navigation error); a tracker doing worse than that is broken |
| ID switches | <= 2 | the scenario has three stretches where nobody sees the entity |
| time to first track | <= 1.0 s | 3 detections at 10 Hz is 0.2-0.3 s |
| hot-path allocations, overruns | 0 | the real-time claims |
| continuity vs best single agent | >= | fusion must not lose coverage |

These were set after the shakedown runs that found the bugs below, and before
the scored runs. One tuning change was made on the way, from a shakedown run:
the deletion limit went from 4 m to 8 m of horizontal sigma (see "Findings").

## Findings along the way

| finding | how it showed up | fix |
|---|---|---|
| **PX4's simulated optical flow is single-vehicle only.** Its KLT tracker kept the previous image and corners in file-level `static` variables, and its rate limiter its last publish time in a function-local `static`, shared by every flow sensor in the Gazebo process. | Both agents fine on the ground, then EKF2's velocity fell behind, both accelerated east past 20 m/s and PX4 blind-landed them. The ESKF rejected 690 of 1,100 flow samples. One agent alone in the same world: 0 rejected, 0.3 m error. | Patch 0003: per-instance tracker state. Library and Gazebo plugin rebuilt, PX4 not. quad-autonomy-sim's Phase 2 re-run after: ESKF 0.20 m (0.22 m before) |
| **Shadows cost most of the real-time factor.** | 0.2-0.6x real time with two flow cameras; PX4 instance 1 at 100 % CPU in its work-queue manager | Shadows off in the world: 0.85-0.95x |
| **Parameter files silently did nothing.** A key `detector:` only matches a node with no namespace. | Detectors crashed without a world file; ESKFs listened to `/fmu/...` instead of `/px4_n/fmu/...` and never aligned | Fully qualified keys (`/agent_1/detector`), and `/**/eskf` for the package defaults |
| **An empty YAML list has no type.** | The offboard node died on `route_nea: []` in its default config | Removed from the defaults |
| **One outlier could steal a track** (unit test). | A 4-sigma detection births a tentative track whose wide covariance makes the next true detections look closer to it; the established track starves, an ID switch | Rank by d^2 + ln|S| instead of d^2, and let established tracks associate first. Regression test `OutlierDoesNotStealTheTrack` |
| **Tracks were deleted while their predictions were still good.** | With deletion at 4 m of horizontal sigma, the fused track died ~4 s into an occlusion with 0.6 m of error, and came back under a new id: 2 ID switches per run | Limit raised to 8 m, so the 10 s timeout decides: 0-1 switches |
| **Navigation bias dominates the fused error.** | Agent 2's detections sat on average 0.6 m east of the entity, the same as its ESKF error at the time, while the ESKF reported 0.16 m of sigma. Fusing two differently biased agents puts the track between them | Not fixed: registration is Phase 2 work. Run D quantifies it |
| **No message pools for bounded sequences.** | `MessagePoolMemoryStrategy` rejects `DetectionArray` at compile time: it only takes fixed-size messages | Accepted: ~7 allocations per message in rclcpp's take path, counted separately. A fixed 16-slot array would cost bandwidth on every frame |
| **The TimingStats p99 could exceed the max.** | "p99 10 us, max 5 us": p99 is the upper edge of a 5 us histogram bin | Capped at the true maximum, with a unit test |
| **`pkill -f` matched its own shell.** | An unanchored pattern killed the command that ran it, the same pitfall quad-autonomy-sim documented | Stale-process patterns are exact names or anchored install paths; long node names are matched by path because Linux truncates process names to 15 characters |

## Limitations

- **Synthetic detector.** Visibility is a single ray to the entity's centre: a
  partly occluded entity is either fully seen or not at all, and there are no
  false detections. Occluders must be boxes.
- **One entity, known class.** Association is exercised by unit tests with
  several entities, but the scenario has one.
- **Constant-velocity model.** The entity turns round instantly at the ends of
  its path; while nobody watches, the prediction carries on the wrong way.
  That is where the coasting error comes from.
- **Navigation bias is not estimated.** Each detection carries its agent's
  navigation error, and the ESKF's reported covariance is optimistic early in
  flight, so fused positions sit between the agents' biases. Registration
  (estimating each agent's bias from jointly seen entities) is future work.
- **Perfect link.** Detections reach the fusion node within milliseconds and
  in order; the reorder buffer and late-batch counters exist for Phase 3.
- **Simulation only.** No hardware, WSL2 timing, lockstep-free Gazebo at
  0.85-0.95x real time.

## Reproduce

```bash
./scripts/demo_phase1.sh                              # headless, scored
./scripts/demo_phase1.sh --gui                        # Gazebo + RViz
POSE_SOURCE=ground_truth ./scripts/demo_phase1.sh     # perfect navigation (diagnostic)
```

Details in [running.md](running.md).

## Final verification

Two more runs on the code as tagged (after a clean rebuild, 0 warnings, 52
unit tests passing), and the by-hand path (`sim.sh` + `stack.sh` + `fly.sh`:
both agents landed, one track for the whole flight, nothing left running
after Ctrl-C).

| run | result | fused RMSE visible | continuity | ID switches | fused NEES | agent 2 navigation RMSE |
|---|---|---|---|---|---|---|
| E, `--gui` (Gazebo GUI + RViz) | **pass** | 0.31 m (agents alone 0.57 / 0.43) | 0.980 (0.815 / 0.717) | 1 | 1.6 | 0.15 m |
| F, headless | **fail** (RMSE) | 0.84 m (agents alone 0.55 / 1.21) | 0.984 (0.877 / 0.660) | 0 | 14.6 | 0.90 m |

**Run F is a known, intermittent failure**, 1 in 6 runs with ESKF navigation.
Agent 2's ESKF logged 80 IMU gaps, 26 suspect timestamps and 24 measurements
off its timeline in that run, against 1-44 gaps and 5-14 suspect stamps in
every other run. The burst came during a turnaround at the west end of its
route. The ESKF bridges each missing IMU interval by holding the last sample,
which under hard deceleration jumped its position about 1 m in 4 s, while its
reported sigma stayed at 0.24 m. Every later detection from agent 2 landed
1.2 m off with a covariance four times too small (NEES 16), and fusion,
trusting it, did worse than agent 1 alone. The demo exited non-zero, as it
should. The samples are lost or misstamped somewhere between PX4's uXRCE-DDS
client, the shared agent and ROS 2 (quad-autonomy-sim saw the same kind of
timesync steps with one vehicle, far less often). Candidate fixes, none made
here: find where the samples go (one XRCE agent per vehicle; IMU at a lower
rate), make the ESKF inflate its covariance over IMU gaps instead of only
counting them, and, on the fusion side, gate per-agent consistency, e.g.
down-weighting an agent whose detections disagree with the others (the
registration work in the roadmap).

**Update, found in Phase 2: the cause, and the fix.** The IMU samples were
never lost. uXRCE-DDS timestamp synchronisation was re-mapping PX4's clock
(simulation time) to the agent's wall clock, and correcting the mapping in
steps whenever the simulation ran below real time; the ESKF saw those steps as
gaps and clock jumps. With the device camera's extra load in Phase 2 it
happened in every run, which made it measurable (recording the raw IMU stream:
no sample missing, 169-185 apparent gaps). `UXRCE_DDS_SYNCT 0` keeps PX4's
stamps on simulation time. The Phase 1 demo re-run with it passed with 0 IMU
gaps on both agents: fused RMSE while visible 0.39 m (agents alone 0.56 /
0.44 m), continuity 0.991, 0 ID switches, navigation 0.24 / 0.31 m. Details in
[phase2_device_overlay.md](phase2_device_overlay.md), "Findings".
