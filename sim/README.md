# sim/

Gazebo Harmonic world, models and scenarios.

| path | what |
|---|---|
| `worlds/coop_field.sdf` | Phase 1 world: PX4's default physics, magnetic field and geodetic origin, the textured `flow_ground` (optical flow needs texture), one 2 x 20 x 6 m building centred on the origin, and the entity parked behind it. Shadows are off (see below). |
| `models/entity_person` | the entity of interest: a 1.75 m person-sized shape (cylinder body, sphere head), static and moved kinematically |
| `models/flow_ground` | 200 m textured ground, visual only (from quad-autonomy-sim) |
| `scenarios/two_agent_wall.yaml` | where everything is: agent spawn poses and routes, the entity's path, detector and fusion settings |
| `rviz/phase1.rviz` | RViz view: building, true positions (TF), fused and baseline tracks |

**The world file is the occlusion model.** The synthetic detector reads every
box collision of every static model from `coop_field.sdf` (libsdformat, with
poses resolved) and ray-casts against those boxes. Moving or resizing the
building here changes occlusion everywhere; nothing is duplicated.

**The entity moves kinematically.** `scripts/entity_mover.py` walks it back
and forth along the scenario's path with Gazebo's `set_pose` at 20 Hz of
simulation time (an in-process gz-transport request, under 1 ms). Its
position is a function of simulation time only, and everything else reads it
back from Gazebo.

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
