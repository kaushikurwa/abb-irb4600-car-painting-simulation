#!/usr/bin/env python3
"""Reachability check: can the arm put the spray gun on every stripe waypoint?
Does NOT move the robot. Asks MoveIt's /compute_ik service (collision checking ON)."""
import json
import math
import os

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState

TOOLS_DIR = os.path.dirname(os.path.abspath(__file__))
STRIPES_JSON = os.path.join(TOOLS_DIR, "roof_stripes.json")
YAWS_DEG = [0, 90, -90, 180]     # ways to spin the gun about its own spray axis
RAIL_LIMIT = 3.2


def tool_down_quaternion(yaw_deg):
    """Orientation (x, y, z, w) that points the gun's +Z (spray direction) straight down."""
    half = math.radians(yaw_deg) / 2.0
    return (math.cos(half), math.sin(half), 0.0, 0.0)


class ReachCheck(Node):
    def __init__(self):
        super().__init__("reach_check")
        self.client = self.create_client(GetPositionIK, "/compute_ik")
        if not self.client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError("/compute_ik not available - is the MoveIt demo running?")

    def ik(self, point, yaw_deg, rail_x):
        req = GetPositionIK.Request()
        r = req.ik_request
        r.group_name = "arm"
        r.ik_link_name = "paint_tcp"
        r.avoid_collisions = True
        r.timeout = Duration(sec=0, nanosec=200_000_000)
        r.robot_state.joint_state = JointState(name=["rail_joint"], position=[rail_x])
        ps = PoseStamped()
        ps.header.frame_id = "world"
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = point
        qx, qy, qz, qw = tool_down_quaternion(yaw_deg)
        ps.pose.orientation.x, ps.pose.orientation.y = qx, qy
        ps.pose.orientation.z, ps.pose.orientation.w = qz, qw
        r.pose_stamped = ps
        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result().error_code.val


def main():
    with open(STRIPES_JSON) as f:
        stripes = json.load(f)
    rclpy.init()
    node = ReachCheck()
    total_ok = total_pts = 0
    print("%-8s %-7s %-9s %s" % ("stripe", "rail_x", "best yaw", "reached"))
    for s in stripes:
        rail_x = max(-RAIL_LIMIT, min(RAIL_LIMIT, s["rail_x"]))
        best = (-1, None, [])
        for yaw in YAWS_DEG:
            first, last = s["points"][0], s["points"][-1]
            test_pts = s["points"] + [[first[0], first[1], first[2] + 0.15],
                                      [last[0], last[1], last[2] + 0.15]]
            codes = [node.ik(p, yaw, rail_x) for p in test_pts]
            n_ok = sum(1 for c in codes if c == 1)
            if n_ok > best[0]:
                best = (n_ok, yaw, codes)
            if n_ok == len(codes):
                break
        n_ok, yaw, codes = best
        n = len(codes)
        total_ok += n_ok
        total_pts += n
        fails = sorted(set(c for c in codes if c != 1))
        note = ("error codes %s" % fails) if fails else ""
        print("%-8s %-7.2f %-9s %d/%d %s" % (s["name"], rail_x, yaw, n_ok, n, note))
    print("TOTAL reachable: %d/%d" % (total_ok, total_pts))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
