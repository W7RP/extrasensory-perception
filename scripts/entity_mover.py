#!/usr/bin/env python3
"""Walk the entity of interest along the scenario's path, in simulation time.

Scenario orchestration, not perception: it plays the part of the world. The
entity walks back and forth (ping-pong) along `entity.path` at
`entity.speed_mps`, facing its direction of travel, starting when this script
starts. Position is a pure function of simulation time since the start, so a
slow simulation just walks it slower in wall time, never differently.

It moves the (static, kinematic) Gazebo model with an in-process gz-transport
set_pose request at 20 Hz of simulation time: under 1 ms per call, measured,
where the `gz service` CLI takes ~1 s to start (quad-autonomy-sim, Phase 4).
Detectors never see this script's numbers: they read the pose back from
Gazebo like everything else (ground_truth_bridge).
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


class EntityMover(Node):
    def __init__(self, sc, rate_hz):
        super().__init__("entity_mover", parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.world = sc["world"]
        self.model = sc["entity"]["model"]
        self.path = [tuple(p) for p in sc["entity"]["path"]]
        self.speed = float(sc["entity"]["speed_mps"])
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
            self.get_logger().info(f"walking {self.model} along {self.path} at {self.speed} m/s")
        x, y, heading = pingpong(self.path, self.speed, now - self.t0)
        req = Pose()
        req.name = self.model
        req.position.x, req.position.y, req.position.z = x, y, 0.0
        req.orientation.z, req.orientation.w = math.sin(heading / 2), math.cos(heading / 2)
        ok, rep = self.gz.request(f"/world/{self.world}/set_pose", req, Pose, Boolean, 1000)
        if not (ok and rep.data):
            self.failures += 1
            if self.failures % 20 == 1:
                self.get_logger().warn(f"set_pose failed ({self.failures} so far)")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scenario", default=str(scn.REPO_ROOT / "sim/scenarios/two_agent_wall.yaml"))
    ap.add_argument("--rate-hz", type=float, default=20.0)
    a = ap.parse_args()
    rclpy.init()
    node = EntityMover(scn.load(a.scenario), a.rate_hz)
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
