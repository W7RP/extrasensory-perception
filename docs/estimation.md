# Agent navigation: the ESKF

Each agent localises itself with the error-state Kalman filter from
quad-autonomy-sim, copied into `ros2_ws/src/agent_estimation`. It is the
author's own code, designed, tuned and validated there (its
`docs/phase2_state_estimation.md`); this page covers only how it is used here
and what changed.

## What it is

A C++20 ESKF that fuses the IMU, a downward optical-flow sensor, a downward
rangefinder and magnetometer heading, on PX4's `x500_flow` airframe (no GPS).
It runs in shadow mode: PX4's EKF2 flies the agent, the ESKF only estimates.
In quad-autonomy-sim it tracked a held-out 12 m square to 0.34 m horizontal RMS
(EKF2: 0.48 m) with a 10-14 us IMU callback and zero heap allocations in its
callbacks.

## How it is used here

One ESKF per agent, as `/agent_<n>/eskf`, reading `/px4_<n>/fmu/out/*`. Its
estimate is what the agent's detector uses to place detections in the world
(`pose_source: eskf`, the default): the detection is the camera-frame
measurement rotated by the ESKF's attitude and added to the ESKF's position,
plus the agent's surveyed start point. So navigation error lands in every
detection the way it would on a real vehicle, and the ESKF's reported position
and attitude variances are part of each detection's covariance.

`POSE_SOURCE=ground_truth` swaps in perfect navigation, to separate how much of
the fused error is navigation and how much is the detector.

## What changed in the copy

- Namespaced: the node runs per agent, its diagnostics status carries the
  node's full name (`/agent_1/eskf: filter`), and its parameter file matches
  any namespace (`/**/eskf`).
- Simulation time: the estimate is stamped on `/clock`, like every other node.
- The real-time instruments (`TimingStats`, `FixedRing`, `alloc_probe`) are
  exported for `track_fusion`, which uses the same design. `alloc_probe` became
  a static library so executables can link it; `TimingStats` now caps its p99
  at the true maximum (the 5 us histogram bin could report a p99 above it).
- The offline replay tool was left out.

The filter itself, its tuning (`config/eskf.yaml`) and its unit tests are
unchanged.

## Known limits that matter here

- Flat ground only: the rangefinder model assumes it, so agents must never fly
  over the building (the routes keep them 6 m clear of it).
- Its position covariance is optimistic early in a flight: in the scored runs,
  one agent's ESKF was 0.6 m off while reporting a 0.16 m sigma. That shows up
  directly as a detection bias (Phase 1 write-up, "Findings").
