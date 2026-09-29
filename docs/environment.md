# Environment (verified 2026-09-28)

The machine everything here was built and scored on. `scripts/check_env.sh`
checks the same list and changes nothing.

| item | version |
|---|---|
| Host | Windows 11 + WSL2, kernel 6.18.33.2-microsoft-standard-WSL2; 32 threads, 15 GB RAM for WSL |
| GPU | NVIDIA GeForce RTX 4070 SUPER (driver 610.74), used through WSLg's D3D12 Mesa driver (Mesa 23.2.1) |
| OS | Ubuntu 22.04.5 LTS |
| ROS 2 | Humble (apt) |
| PX4 | v1.17.0 at `~/PX4-Autopilot`, SITL build `px4_sitl_default`, with `firmware/px4_patches/0001` and `0002` |
| PX4-OpticalFlow | 8758bd5 (fetched by PX4's build), klt_feature_tracker a2733b5, with `firmware/px4_patches/0003` |
| px4_msgs | `release/1.17` @ 86d8239 (in `ros2_ws/src/external`, git-ignored) |
| Gazebo | Harmonic, gz-sim 8.15.0; libsdformat14 14.9.0; gz-transport 13.6.0 (+ Python bindings) |
| ROS 2 <-> Gazebo | `ros-humble-ros-gzharmonic` 0.244.12 (Gazebo's apt repository) |
| Micro XRCE-DDS Agent | v2.4.3, built into `~/.local` |
| Compiler / build | g++ 11.4.0, CMake 3.22.1, colcon; C++20 |
| Eigen | 3.4.0 |
| Python | 3.10.12; NumPy 2.2.6 and matplotlib 3.10.9 in `~/.local` (see note 3) |
| Game window | SDL2 2.0.20 (`libsdl2-dev`), OpenCV 4.5.4 (`libopencv-dev`), both already installed |

Everything above was already installed by quad-autonomy-sim's setup scripts;
this project installed nothing. The one change to the shared toolchain is
patch 0003, applied to PX4's fetched copy of PX4-OpticalFlow and rebuilt
(library + Gazebo plugin, no PX4 rebuild). It does not change single-vehicle
behaviour: quad-autonomy-sim's Phase 2 demo was re-run before and after it
(results in the Phase 1 write-up).

## Checks

| check | result |
|---|---|
| `scripts/check_env.sh` | all checks pass |
| Clean build (`rm -rf build install log; colcon build`) | 6 packages, 0 compiler warnings (`-Wall -Wextra -Wpedantic -Wshadow -Wconversion`) |
| Unit tests | 52 gtest cases in 5 test executables, 0 failures |
| Phase 1 demo | see [phase1_two_agent_fusion.md](phase1_two_agent_fusion.md), "Results" |

## Notes

1. **Two PX4 instances in one world.** Agent n is `px4 -i n` started with
   `PX4_GZ_STANDALONE=1`: PX4 spawns its own `x500_flow_n` into the running
   world and namespaces its uXRCE-DDS topics `/px4_n`. Both share one XRCE
   agent on UDP 8888 (`UXRCE_DDS_KEY` differs per instance). Each instance
   keeps its parameters and logs in `~/.local/state/coop-perception-sim/sitl/agent_n`.
2. **Real-time factor.** With the two agents' flow cameras rendering headless
   through WSLg, Gazebo ran at 0.2-0.6x real time while the world had shadows
   on, and PX4 instance 1 sat at 100 % CPU in its work-queue manager trying to
   keep up. With shadows off (this repo's world) it runs at about 0.85-0.95x,
   and the PX4 instances at ~13 % CPU each.
3. **NumPy 2 vs apt Python packages** (from quad-autonomy-sim): PX4's
   `ubuntu.sh` put NumPy 2.2.6 in `~/.local`, which breaks Ubuntu's
   matplotlib, OpenCV (`cv2`) and SciPy. A user matplotlib 3.10.9 fixes
   plotting; this repo avoids the other two in Python (drawing and video are
   in C++, track matching is plain NumPy). The evaluation prints a harmless
   "Unable to import Axes3D" warning because both matplotlib versions are
   installed.
