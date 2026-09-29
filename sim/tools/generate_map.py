#!/usr/bin/env python3
"""Generate a playable map: a coloured Gazebo world and the scenario that goes with it.

  generate_map.py --seed 7                       # sim/worlds/map_7.sdf + sim/scenarios/map_7.yaml
  generate_map.py --seed 7 --entities 6 --size 70

Deterministic: the same seed gives the same map, byte for byte.

What it places, inside a square of --size metres:
  buildings   boxes 4-12 m on a side, 3-8 m tall, brick / concrete / plaster colours
  walls       long, thin, 2.5-3.5 m: the classic "behind the wall"
  containers  6 x 2.4 x 2.6 m shipping containers, red / blue / green / rust
  crates      1-1.5 m wooden crates: too low to hide behind standing, but block the view of the ground
  trees       a trunk (cylinder) and a canopy (sphere): both are occluders; only the trunk blocks walking
It keeps a clear plaza in the middle (where the device starts) and one launch
pad per agent at the edge, and gives every entity a closed walking loop that
never passes through anything.

Everything that hides is a box, cylinder or sphere collision on a static
model: exactly what the detectors, the overlay and the walker read back from
the world file, so there is one geometry for physics, rendering and occlusion.

The scenario it writes is ready for scripts/demo_phase3.sh: GPS agents flying
above the rooftops, positioned around the device by the overwatch planner, the
device either driven (game mode) or on a scripted loop through the plaza
(scored mode).
"""
import argparse
import math
from pathlib import Path

import numpy as np
import yaml

REPO = Path(__file__).resolve().parents[2]

BUILDING_COLOURS = [(0.62, 0.27, 0.2), (0.55, 0.55, 0.55), (0.8, 0.74, 0.6), (0.45, 0.38, 0.33),
                    (0.7, 0.66, 0.62), (0.52, 0.3, 0.25)]
CONTAINER_COLOURS = [(0.7, 0.15, 0.12), (0.12, 0.3, 0.6), (0.15, 0.45, 0.25), (0.6, 0.35, 0.15),
                     (0.85, 0.6, 0.1)]
ENTITY_COLOURS = [(0.95, 0.4, 0.1), (0.9, 0.15, 0.5), (0.95, 0.85, 0.1), (0.3, 0.8, 0.95),
                  (0.6, 0.3, 0.9), (0.95, 0.95, 0.95), (0.2, 0.9, 0.3), (0.9, 0.5, 0.6)]


# ------------------------------------------------------------------ layout

class Layout:
    """Footprints on a grid, for spacing checks and walkable-space queries."""

    def __init__(self, size, cell=0.5):
        self.size = size
        self.cell = cell
        self.n = int(size / cell)
        self.blocked = np.zeros((self.n, self.n), dtype=bool)  # [ix, iy]
        self.circles = []  # (x, y, r) footprints for spacing

    def idx(self, x, y):
        return int((x + self.size / 2) / self.cell), int((y + self.size / 2) / self.cell)

    def free_circle(self, x, y, r):
        h = self.size / 2
        if abs(x) + r > h - 1 or abs(y) + r > h - 1:
            return False
        return all(math.hypot(x - cx, y - cy) > r + cr for cx, cy, cr in self.circles)

    def add_box(self, x, y, lx, ly, yaw, margin=0.6):
        c, s = math.cos(yaw), math.sin(yaw)
        hx, hy = lx / 2 + margin, ly / 2 + margin
        for ix in range(self.n):
            px = -self.size / 2 + (ix + 0.5) * self.cell
            for iy in range(self.n):
                py = -self.size / 2 + (iy + 0.5) * self.cell
                dx, dy = px - x, py - y
                u, v = c * dx + s * dy, -s * dx + c * dy
                if abs(u) <= hx and abs(v) <= hy:
                    self.blocked[ix, iy] = True
        self.circles.append((x, y, math.hypot(lx, ly) / 2))

    def add_disc(self, x, y, r, margin=0.6, block=True):
        if block:
            rr = r + margin
            for ix in range(self.n):
                px = -self.size / 2 + (ix + 0.5) * self.cell
                for iy in range(self.n):
                    py = -self.size / 2 + (iy + 0.5) * self.cell
                    if math.hypot(px - x, py - y) <= rr:
                        self.blocked[ix, iy] = True
        self.circles.append((x, y, r))

    def walkable(self, x, y):
        ix, iy = self.idx(x, y)
        return 0 <= ix < self.n and 0 <= iy < self.n and not self.blocked[ix, iy]

    def segment_walkable(self, a, b):
        steps = max(2, int(math.dist(a, b) / (self.cell / 2)))
        return all(self.walkable(a[0] + (b[0] - a[0]) * k / steps, a[1] + (b[1] - a[1]) * k / steps)
                   for k in range(steps + 1))


def place(rng, layout, n, radius_fn, keep_out, tries=400):
    """Up to n positions whose footprint circles do not overlap anything so far."""
    out = []
    for _ in range(n):
        for _ in range(tries):
            h = layout.size / 2
            x, y = rng.uniform(-h, h), rng.uniform(-h, h)
            r = radius_fn()
            if any(math.hypot(x - kx, y - ky) < kr + r for kx, ky, kr in keep_out):
                continue
            if layout.free_circle(x, y, r + 1.2):
                out.append((x, y, r))
                break
    return out


def loop_route(rng, layout, keep_out, n_points=5, tries=3000, center=None, radius=(6, 14)):
    """A closed walking loop of n_points waypoints through walkable space."""
    h = layout.size / 2 - 3
    for _ in range(tries):
        cx, cy = center if center is not None else (rng.uniform(-h, h), rng.uniform(-h, h))
        rad = rng.uniform(*radius)
        pts = []
        for k in range(n_points):
            a = 2 * math.pi * k / n_points + rng.uniform(-0.4, 0.4)
            r = rad * rng.uniform(0.6, 1.2)
            pts.append((round(cx + r * math.cos(a), 2), round(cy + r * math.sin(a), 2)))
        if any(math.hypot(p[0] - kx, p[1] - ky) < kr for p in pts for kx, ky, kr in keep_out):
            continue
        closed = pts + [pts[0]]
        if all(layout.segment_walkable(a, b) for a, b in zip(closed, closed[1:])):
            return pts
    return None


# --------------------------------------------------------------------- SDF

def material(rgb, roughness=0.9):
    r, g, b = rgb
    return (f"<material><diffuse>{r:.3f} {g:.3f} {b:.3f} 1</diffuse>"
            f"<ambient>{r * 0.8:.3f} {g * 0.8:.3f} {b * 0.8:.3f} 1</ambient>"
            f"<specular>0.05 0.05 0.05 1</specular></material>")


def box_model(name, x, y, lx, ly, lz, yaw, rgb):
    geom = f"<geometry><box><size>{lx:.2f} {ly:.2f} {lz:.2f}</size></box></geometry>"
    return (f'    <model name="{name}"><static>true</static>'
            f"<pose>{x:.2f} {y:.2f} {lz / 2:.2f} 0 0 {yaw:.3f}</pose><link name=\"link\">"
            f'<collision name="collision">{geom}</collision>'
            f'<visual name="visual">{geom}{material(rgb)}</visual></link></model>\n')


def tree_model(name, x, y, trunk_r, trunk_h, canopy_r, green):
    trunk = f"<geometry><cylinder><radius>{trunk_r:.2f}</radius><length>{trunk_h:.2f}</length></cylinder></geometry>"
    canopy = f"<geometry><sphere><radius>{canopy_r:.2f}</radius></sphere></geometry>"
    cz = trunk_h + canopy_r * 0.7
    return (f'    <model name="{name}"><static>true</static><pose>{x:.2f} {y:.2f} 0 0 0 0</pose>'
            f'<link name="link">'
            f'<collision name="trunk"><pose>0 0 {trunk_h / 2:.2f} 0 0 0</pose>{trunk}</collision>'
            f'<visual name="trunk"><pose>0 0 {trunk_h / 2:.2f} 0 0 0</pose>{trunk}{material((0.36, 0.24, 0.14))}</visual>'
            f'<collision name="canopy"><pose>0 0 {cz:.2f} 0 0 0</pose>{canopy}</collision>'
            f'<visual name="canopy"><pose>0 0 {cz:.2f} 0 0 0</pose>{canopy}{material(green)}</visual>'
            f"</link></model>\n")


def entity_model(name, x, y, rgb):
    body = "<geometry><cylinder><radius>0.22</radius><length>1.44</length></cylinder></geometry>"
    head = "<geometry><sphere><radius>0.15</radius></sphere></geometry>"
    return (f'    <model name="{name}"><static>true</static><pose>{x:.2f} {y:.2f} 0 0 0 0</pose>'
            f'<link name="link"><visual name="body"><pose>0 0 0.72 0 0 0</pose>{body}{material(rgb, 0.6)}</visual>'
            f'<visual name="head"><pose>0 0 1.6 0 0 0</pose>{head}{material(rgb, 0.6)}</visual>'
            f"</link></model>\n")


def device_model(x, y, width, height, rate, hfov):
    return f'''    <model name="device"><static>true</static><pose>{x:.2f} {y:.2f} 0 0 0 0</pose>
      <link name="link">
        <visual name="post"><pose>0 0 0.78 0 0 0</pose><geometry><cylinder><radius>0.02</radius><length>1.56</length></cylinder></geometry>{material((0.15, 0.15, 0.15))}</visual>
        <visual name="body"><pose>-0.03 0 1.6 0 0 0</pose><geometry><box><size>0.06 0.18 0.12</size></box></geometry>{material((0.1, 0.3, 0.8))}</visual>
        <sensor name="camera" type="camera"><pose>0 0 1.6 0 0 0</pose><always_on>1</always_on>
          <update_rate>{rate}</update_rate><visualize>false</visualize><topic>device/camera/image</topic>
          <camera><horizontal_fov>{hfov:.4f}</horizontal_fov>
            <image><width>{width}</width><height>{height}</height><format>R8G8B8</format></image>
            <clip><near>0.1</near><far>200</far></clip></camera></sensor>
      </link></model>
'''


def world_sdf(name, models, shadows):
    return f'''<?xml version="1.0" encoding="UTF-8"?>
<!-- Generated by sim/tools/generate_map.py; do not edit (regenerate instead).
     Physics, magnetic field and geodetic origin as PX4's default world. -->
<sdf version="1.9">
  <world name="{name}">
    <physics type="ode">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>250</real_time_update_rate>
    </physics>
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type="adiabatic"/>
    <scene>
      <grid>false</grid>
      <ambient>0.55 0.55 0.58 1</ambient>
      <background>0.62 0.76 0.92 1</background>
      <shadows>{"true" if shadows else "false"}</shadows>
    </scene>
    <model name="ground_plane"><static>true</static><link name="link">
      <collision name="collision"><geometry><plane><normal>0 0 1</normal><size>1 1</size></plane></geometry></collision>
    </link></model>
    <include><uri>model://grass_ground</uri></include>
    <light name="sun" type="directional">
      <pose>0 0 500 0 0 0</pose>
      <cast_shadows>{"true" if shadows else "false"}</cast_shadows>
      <intensity>1.1</intensity>
      <direction>0.35 0.45 -0.82</direction>
      <diffuse>1.0 0.97 0.9 1</diffuse>
      <specular>0.25 0.25 0.25 1</specular>
    </light>
{models}    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>47.397971057728974</latitude_deg>
      <longitude_deg>8.546163739800146</longitude_deg>
      <elevation>0</elevation>
    </spherical_coordinates>
  </world>
</sdf>
'''


# ---------------------------------------------------------------- generate

def generate(a):
    rng = np.random.default_rng(a.seed)
    size = a.size
    h = size / 2
    layout = Layout(size)
    plaza = (0.0, 0.0, 7.0)                       # the device's start: keep it open
    n_agents = a.agents
    pads = [(round((h - 4) * math.cos(2 * math.pi * k / n_agents + math.pi / 4), 1),
             round((h - 4) * math.sin(2 * math.pi * k / n_agents + math.pi / 4), 1), 3.0)
            for k in range(n_agents)]
    keep_out = [plaza] + pads
    models = []
    tallest = 0.0

    for i, (x, y, r) in enumerate(place(rng, layout, a.buildings, lambda: rng.uniform(3.0, 6.5), keep_out)):
        lx, ly = rng.uniform(4, 12), rng.uniform(4, 10)
        s = 2 * r / math.hypot(lx, ly)
        lx, ly = lx * s, ly * s
        lz = rng.uniform(3, 8)
        yaw = rng.choice([0.0, math.pi / 2]) + rng.uniform(-0.25, 0.25)
        tallest = max(tallest, lz)
        layout.add_box(x, y, lx, ly, yaw)
        models.append(box_model(f"building_{i}", x, y, lx, ly, lz, yaw,
                                BUILDING_COLOURS[int(rng.integers(len(BUILDING_COLOURS)))]))
    for i, (x, y, r) in enumerate(place(rng, layout, a.walls, lambda: rng.uniform(3.0, 7.5), keep_out)):
        length, lz, yaw = 2 * r, rng.uniform(2.5, 3.5), rng.uniform(0, math.pi)
        tallest = max(tallest, lz)
        layout.add_box(x, y, length, 0.4, yaw)
        models.append(box_model(f"wall_{i}", x, y, length, 0.4, lz, yaw, (0.66, 0.64, 0.6)))
    for i, (x, y, r) in enumerate(place(rng, layout, a.containers, lambda: 3.3, keep_out)):
        yaw = rng.uniform(0, math.pi)
        layout.add_box(x, y, 6.0, 2.4, yaw)
        models.append(box_model(f"container_{i}", x, y, 6.0, 2.4, 2.6, yaw,
                                CONTAINER_COLOURS[int(rng.integers(len(CONTAINER_COLOURS)))]))
    for i, (x, y, r) in enumerate(place(rng, layout, a.crates, lambda: 0.9, keep_out)):
        s, yaw = rng.uniform(1.0, 1.5), rng.uniform(0, math.pi)
        layout.add_box(x, y, s, s, yaw)
        models.append(box_model(f"crate_{i}", x, y, s, s, s, yaw, (0.55, 0.4, 0.22)))
    for i, (x, y, r) in enumerate(place(rng, layout, a.trees, lambda: rng.uniform(1.3, 2.4), keep_out)):
        trunk_r, trunk_h = rng.uniform(0.18, 0.32), rng.uniform(2.0, 3.2)
        green = (rng.uniform(0.12, 0.25), rng.uniform(0.4, 0.6), rng.uniform(0.12, 0.22))
        layout.add_disc(x, y, trunk_r)                 # only the trunk blocks walking
        layout.circles[-1] = (x, y, r)                 # but space trees by their canopy
        tallest = max(tallest, trunk_h + 1.7 * r)
        models.append(tree_model(f"tree_{i}", x, y, trunk_r, trunk_h, r, green))

    # Entities: closed walking loops through walkable space, not through the plaza.
    entities = []
    for i in range(a.entities):
        route = loop_route(rng, layout, [plaza])
        if route is None:
            continue
        name = f"entity_{i + 1}"
        entities.append({"name": name, "model": name, "ref_height_m": 0.9, "mode": "loop",
                         "speed_mps": round(float(rng.uniform(0.6, 1.1)), 2), "path": [list(p) for p in route]})
        models.append(entity_model(name, route[0][0], route[0][1], ENTITY_COLOURS[i % len(ENTITY_COLOURS)]))

    # The device: starts in the plaza; its scripted (scored) loop goes round
    # the middle of the map, where it passes most of what there is to see.
    dev_route = loop_route(rng, layout, [], n_points=7, center=(0.0, 0.0), radius=(10, 16)) or \
        [[4, 0], [0, 4], [-4, 0], [0, -4]]
    models.append(device_model(0.0, 0.0, a.cam_width, a.cam_height, a.cam_rate, math.radians(a.cam_hfov)))

    world_name = f"map_{a.seed}"
    world_path = REPO / "sim" / "worlds" / f"{world_name}.sdf"
    world_path.write_text(world_sdf(world_name, "".join(models), a.shadows))

    altitude = math.ceil(tallest) + 4.0
    scenario = {
        "world": world_name,
        "world_file": f"sim/worlds/{world_name}.sdf",
        "generated": {"seed": a.seed, "size_m": size, "tallest_m": round(tallest, 2),
                      "by": "sim/tools/generate_map.py"},
        "bounds_xy": [-h, h, -h, h],
        "entities": entities,
        "agents": [{"id": k + 1, "spawn": [pads[k][0], pads[k][1], 0.0, 0.0],
                    "altitude_m": altitude + 2.0 * k, "speed_mps": 3.0} for k in range(n_agents)],
        "device": {
            "model": "device", "control": "scripted", "speed_mps": 1.0,
            "path": [list(p) for p in dev_route], "mode": "loop", "look_at": None,
            "camera": {"mount_xyz": [0.0, 0.0, 1.6], "image_topic": "/device/camera/image",
                       "info_topic": "/device/camera/camera_info", "hfov_deg": a.cam_hfov},
            "entity_size_m": [0.5, 0.5, 1.75],
        },
        "px4": {"model": "x500"},
        "detector": {
            "rate_hz": 10.0, "pose_source": "px4",
            "camera": {"hfov_deg": 90.0, "vfov_deg": 70.0, "pitch_down_deg": 40.0,
                       "min_range_m": 1.0, "max_range_m": a.vision_radius},
            "noise": {"range_sigma_base_m": 0.15, "range_sigma_per_m": 0.02,
                      "cross_sigma_base_m": 0.05, "cross_sigma_per_m": 0.005, "p_miss": 0.1},
        },
        "fusion": {"associator": "hungarian", "process_noise": 0.5, "gate_d2": 16.0,
                   "confirm_hits": 3, "coast_after_s": 0.5, "delete_after_s": 10.0,
                   "max_position_sigma_m": 8.0, "publish_rate_hz": 10.0},
        "planner": {"radius_m": a.radius, "rate_hz": 1.0, "interest_radius_m": a.radius + 12.0,
                    "min_separation_m": 8.0, "candidates": 24, "cell_m": 1.5, "hysteresis": 0.15},
    }
    scen_path = REPO / "sim" / "scenarios" / f"{world_name}.yaml"
    with open(scen_path, "w") as f:
        f.write(f"# Generated by sim/tools/generate_map.py --seed {a.seed}; regenerate rather than edit.\n")
        yaml.safe_dump(scenario, f, sort_keys=False, default_flow_style=None)
    print(f"{world_path.relative_to(REPO)}: {len(models)} models ({len(entities)} entities), "
          f"tallest {tallest:.1f} m, agents at {altitude:.0f}+ m")
    print(scen_path.relative_to(REPO))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--size", type=float, default=60.0, help="side of the square map [m]")
    ap.add_argument("--buildings", type=int, default=7)
    ap.add_argument("--walls", type=int, default=5)
    ap.add_argument("--containers", type=int, default=5)
    ap.add_argument("--crates", type=int, default=10)
    ap.add_argument("--trees", type=int, default=14)
    ap.add_argument("--entities", type=int, default=4)
    ap.add_argument("--agents", type=int, default=2)
    ap.add_argument("--radius", type=float, default=14.0, help="agents' ring around the device [m]")
    ap.add_argument("--vision-radius", type=float, default=35.0, help="agents' camera range [m]")
    ap.add_argument("--cam-width", type=int, default=960)
    ap.add_argument("--cam-height", type=int, default=720)
    ap.add_argument("--cam-rate", type=int, default=30)
    ap.add_argument("--cam-hfov", type=float, default=75.0)
    ap.add_argument("--shadows", action="store_true", help="directional shadows (costs frame rate)")
    generate(ap.parse_args())


if __name__ == "__main__":
    main()
