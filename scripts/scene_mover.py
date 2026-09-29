#!/usr/bin/env python3
"""Move the scenario's kinematic models in simulation time: the entity and,
if the scenario has one, the ground device.

Scenario orchestration, not perception: it plays the part of the world.
  entities  each walks its `path` at its `speed_mps`, facing its direction of
            travel: back and forth (`mode: pingpong`, the default) or round a
            closed loop (`mode: loop`);
  device  walks back and forth along `device.path` at `device.speed_mps`,
          its camera always facing `device.look_at`.
Both start when this script starts. Positions are pure functions of
simulation time since the start, so a slow simulation just moves them slower
in wall time, never differently.

It moves the (static, kinematic) Gazebo models with in-process gz-transport
set_pose requests at 20 Hz of simulation time: under 1 ms per call, measured,
where the `gz service` CLI takes ~1 s to start (quad-autonomy-sim, Phase 4).
Nothing downstream sees this script's numbers: everything reads the poses
back from Gazebo (ground_truth_bridge).
"""
import argparse
import math
import sys
from pathlib import Path

import rclpy
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.pose_pb2 import Pose
from gz.transport13 import Node as GzNode
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter

sys.path.insert(0, str(Path(__file__).resolve().parent / "lib"))
import scenario as scn  # noqa: E402


def walk(points, speed, t, mode="pingpong"):
    """(x, y, heading) after t seconds on the path: back and forth, or round a loop."""
    if mode == "loop":
        return loop(points, speed, t)
    return pingpong(points, speed, t)


def loop(points, speed, t):
    closed = list(points) + [points[0]]
    segs = list(zip(closed, closed[1:]))
    lens = [math.dist(p, q) for p, q in segs]
    s = (t * speed) % sum(lens)
    for (p, q), L in zip(segs, lens):
        if s <= L and L > 0:
            f = s / L
            return p[0] + f * (q[0] - p[0]), p[1] + f * (q[1] - p[1]), math.atan2(q[1] - p[1], q[0] - p[0])
        s -= L
    return points[0][0], points[0][1], 0.0


def pingpong(points, speed, t):
    """(x, y, heading) after t seconds walking the polyline back and forth."""
    segs = list(zip(points, points[1:]))
    lens = [math.dist(p, q) for p, q in segs]
    total = sum(lens)
    s = (t * speed) % (2 * total)
    backwards = s > total
    if backwards:
        s = 2 * total - s
    for (p, q), L in zip(segs, lens):
        if s <= L:
            f = s / L
            x, y = p[0] + f * (q[0] - p[0]), p[1] + f * (q[1] - p[1])
            heading = math.atan2(q[1] - p[1], q[0] - p[0]) + (math.pi if backwards else 0.0)
            return x, y, heading
        s -= L
    return points[-1][0], points[-1][1], 0.0


class SceneMover(Node):
    def __init__(self, sc, rate_hz):
        super().__init__("scene_mover", parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.world = sc["world"]
        # (model, path, speed, look_at or None, mode)
        self.movers = [(e["model"], [tuple(p) for p in e["path"]], float(e["speed_mps"]), None,
                        e.get("mode", "pingpong")) for e in sc["entities"]]
        # The device is moved here only on a scripted route; when it is driven
        # (`device.control: teleop`), device_controller moves it.
        if "device" in sc and sc["device"].get("control", "scripted") == "scripted":
            d = sc["device"]
            look_at = tuple(d["look_at"]) if d.get("look_at") else None  # None: face the way it walks
            self.movers.append((d["model"], [tuple(p) for p in d["path"]], float(d["speed_mps"]),
                                look_at, d.get("mode", "pingpong")))
        self.gz = GzNode()
        self.t0 = None
        self.failures = 0
        self.create_timer(1.0 / rate_hz, self.tick)

    def tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if now == 0.0:
            return  # no /clock yet
        if self.t0 is None:
            self.t0 = now
            for model, path, speed, _, _ in self.movers:
                self.get_logger().info(f"moving {model} along {path} at {speed} m/s")
        for model, path, speed, look_at, mode in self.movers:
            x, y, heading = walk(path, speed, now - self.t0, mode)
            if look_at is not None:
                heading = math.atan2(look_at[1] - y, look_at[0] - x)
            req = Pose()
            req.name = model
            req.position.x, req.position.y, req.position.z = x, y, 0.0
            req.orientation.z, req.orientation.w = math.sin(heading / 2), math.cos(heading / 2)
            ok, rep = self.gz.request(f"/world/{self.world}/set_pose", req, Pose, Boolean, 1000)
            if not (ok and rep.data):
                self.failures += 1
                if self.failures % 20 == 1:
                    self.get_logger().warn(f"set_pose {model} failed ({self.failures} so far)")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scenario", default=str(scn.REPO_ROOT / "sim/scenarios/two_agent_wall.yaml"))
    ap.add_argument("--rate-hz", type=float, default=20.0)
    a = ap.parse_args()
    rclpy.init()
    node = SceneMover(scn.load(a.scenario), a.rate_hz)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass  # SIGINT/SIGTERM from the demo: a normal stop
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
