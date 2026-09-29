# sim/

Gazebo Harmonic world, models and scenarios.

| path | what |
|---|---|
| `worlds/coop_field.sdf` | Phase 1 world: PX4's default physics, magnetic field and geodetic origin, the textured `flow_ground` (optical flow needs texture), one 2 x 20 x 6 m building centred on the origin, and the entity parked behind it. Shadows are off (see below). |
| `models/entity_person` | the entity of interest: a 1.75 m person-sized shape (cylinder body, sphere head), static and moved kinematically |
| `models/device` | the ground device (Phase 2): a 640 x 480, 15 Hz, 70 deg camera at 1.6 m on a post, static and moved kinematically |
| `worlds/map_<seed>.sdf`, `scenarios/map_<seed>.yaml` | generated maps (Phase 3) by `tools/generate_map.py`: coloured buildings, walls, containers, crates and trees, several entities on walking loops, the device's camera at 960 x 720, 30 Hz |
| `models/grass_ground` | 200 m grass ground (generated texture, tileable every 25 m) for the generated maps |
| `models/flow_ground` | 200 m textured ground, visual only (from quad-autonomy-sim) |
| `scenarios/two_agent_wall.yaml` | where everything is: agent spawn poses and routes, the entity's path, detector and fusion settings |
| `rviz/phase1.rviz` | RViz view: building, true positions (TF), fused and baseline tracks |
| `rviz/phase2.rviz` | the same map with the device's tracks, plus the device's see-through camera view |

**The world file is the occlusion model.** The synthetic detector reads every
box, cylinder and sphere collision of every static model from `coop_field.sdf` (libsdformat, with
poses resolved) and ray-casts against those boxes. Moving or resizing the
building here changes occlusion everywhere; nothing is duplicated.

**The entity and the device move kinematically.** `scripts/scene_mover.py`
walks both back and forth along their paths in the scenario (the device
turned to face its look-at point) with Gazebo's `set_pose` at 20 Hz of
simulation time (an in-process gz-transport request, under 1 ms). Their
positions are functions of simulation time only, and everything else reads
them back from Gazebo. The device's camera, being a sensor on a static model,
follows the model when it is moved (checked: 15 Hz while moving).

**Agents** are PX4's stock `x500_flow`, spawned by PX4 itself (instance n
spawns `x500_flow_n`); nothing here overrides their models.

**Why shadows are off.** Two agents means two 50 Hz flow cameras rendering in
the server. With the world's directional light casting shadows, each render
was expensive enough to hold the simulation at 0.2-0.6x real time. Without
shadows it runs at 0.85-0.95x. The flow cameras only need the ground texture.

**Scenario design.** `python3 scripts/lib/scenario.py preview <scenario>`
flies the routes kinematically with the detector's visibility rules and prints
the timeline. The Phase 1 routes were chosen with it: agent 1 south of the
building facing north, agent 2 north facing south, at 4 m (below the 6 m roof),
on beats of different length so the entity is seen by nobody, one or both
agents at different times (preview: 38 / 37 / 26 %; flown: 44 / 33 / 23 %).
