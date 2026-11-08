#!/usr/bin/env python3
"""Paint roof stripes: rail -> hover -> descend -> paint (Cartesian) -> lift. Draws paint trail on the car.
Usage: paint_roof.py [stripe_name | all]      (default: roof_00)
Assumes turntable_joint = 0 and car_y_joint = 0 (car frame == world frame)."""
import json
import os
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point, Pose, PoseStamped
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
from moveit_msgs.srv import GetCartesianPath, GetPositionIK
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

TOOLS_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_JSON = os.path.join(TOOLS_DIR, "roof_stripes.json")
ARM_JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
RAIL_LIMIT = 3.2        # m
HOVER = 0.15            # m above the stripe for approach and lift
SPRAY_DIST = 0.28       # m, nozzle to surface (trail is drawn on the surface)
JOINT_SPEED = 0.15      # rad/s, slowest joint while painting
MIN_FRACTION = 0.99     # required fraction of each Cartesian path that must be feasible
MAX_JUMP = 0.6          # rad, largest allowed joint change between neighbouring path points


def make_pose(p):
    """Gun pointing straight down (tool +Z = spray direction = world -Z)."""
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = float(p[0]), float(p[1]), float(p[2])
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = 1.0, 0.0, 0.0, 0.0
    return pose


def retime(traj, speed):
    """Give every trajectory point a time so that no joint moves faster than `speed` rad/s."""
    t, prev = 0.0, None
    for pt in traj.joint_trajectory.points:
        if prev is not None:
            d = max(abs(a - b) for a, b in zip(pt.positions, prev))
            t += max(d / speed, 0.05)
        pt.velocities, pt.accelerations, pt.effort = [], [], []
        pt.time_from_start = Duration(sec=int(t), nanosec=int((t - int(t)) * 1e9))
        prev = list(pt.positions)


class Painter(Node):
    def __init__(self):
        super().__init__("painter")
        self.ik_srv = self.create_client(GetPositionIK, "/compute_ik")
        self.cart_srv = self.create_client(GetCartesianPath, "/compute_cartesian_path")
        self.move_ac = ActionClient(self, MoveGroup, "/move_action")
        self.exec_ac = ActionClient(self, ExecuteTrajectory, "/execute_trajectory")
        for c in (self.ik_srv, self.cart_srv):
            if not c.wait_for_service(timeout_sec=10.0):
                raise RuntimeError("MoveIt service missing - is the demo running?")
        for a in (self.move_ac, self.exec_ac):
            if not a.wait_for_server(timeout_sec=10.0):
                raise RuntimeError("MoveIt action server missing - is the demo running?")
        self.joints = {}
        self.create_subscription(JointState, "/joint_states", self.on_js, 10)
        self.tf_buf = Buffer()
        self.tf_listener = TransformListener(self.tf_buf, self)
        self.trail_pub = self.create_publisher(MarkerArray, "/paint_trail", 10)
        self.trails = {}
        self.painting = None
        self.create_timer(0.05, self.tick)

    # ---------- helpers ----------
    def on_js(self, msg):
        for n, p in zip(msg.name, msg.position):
            self.joints[n] = p

    def current_state(self):
        js = JointState()
        js.name = list(self.joints.keys())
        js.position = [self.joints[n] for n in js.name]
        return js

    def wait(self, sec):
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def tick(self):
        if self.painting:
            try:
                t = self.tf_buf.lookup_transform("car", "paint_tcp", Time())
                p = t.transform.translation
                ox, oy, oz = getattr(self, "trail_offset", (0.0, 0.0, -SPRAY_DIST))
                self.trails.setdefault(self.painting, []).append((p.x + ox, p.y + oy, p.z + oz))
            except Exception:
                pass
        arr = MarkerArray()
        for i, (name, pts) in enumerate(self.trails.items()):
            if len(pts) < 2:
                continue
            mk = Marker()
            mk.header.frame_id = "car"
            mk.ns = "paint_trail"
            mk.id = i + (100 if name.startswith("far_") else 200 if name.startswith("side_") else 0)
            mk.type = Marker.LINE_STRIP
            mk.action = Marker.ADD
            mk.scale.x = 0.20
            mk.color.r, mk.color.g, mk.color.b, mk.color.a = ((1.0, 0.45, 0.1, 0.75) if name.startswith("far_") else (0.2, 0.8, 0.3, 0.75) if name.startswith("side_") else (0.1, 0.6, 1.0, 0.75))
            mk.pose.orientation.w = 1.0
            mk.points = [Point(x=a, y=b, z=c) for a, b, c in pts]
            arr.markers.append(mk)
        if arr.markers:
            self.trail_pub.publish(arr)

    def run_action(self, client, goal):
        fut = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut)

        handle = fut.result()
        if handle is None or not handle.accepted:
            return None

        rfut = handle.get_result_async()
        rclpy.spin_until_future_complete(self, rfut)

        result = rfut.result()
        if result is None:
            return None

        return result.result

    # ---------- motions ----------

    def move_joints(self, group, names, values, label):
        goal = MoveGroup.Goal()
        r = goal.request
        r.group_name = group
        r.num_planning_attempts = 10
        r.allowed_planning_time = 5.0
        r.max_velocity_scaling_factor = 0.3
        r.max_acceleration_scaling_factor = 0.3

        c = Constraints()
        for n, v in zip(names, values):
            c.joint_constraints.append(JointConstraint(
                joint_name=n,
                position=float(v),
                tolerance_above=0.001,
                tolerance_below=0.001,
                weight=1.0
            ))

        r.goal_constraints.append(c)
        goal.planning_options.plan_only = False

        res = self.run_action(self.move_ac, goal)

        if res is None:
            print("  [FAILED] %s: MoveIt action returned no result" % label)
            self.wait(0.5)
            return False

        code = res.error_code.val
        print("  MoveIt error code: %d" % code)

        ok = code == 1
        print("  [%s] %s" % ("ok" if ok else "FAILED", label))

        self.wait(0.5)
        return ok

    def ik(self, pt):
        req = GetPositionIK.Request()
        r = req.ik_request
        r.group_name = "arm"
        r.ik_link_name = "paint_tcp"
        r.avoid_collisions = True
        r.timeout = Duration(sec=0, nanosec=300_000_000)
        r.robot_state.joint_state = self.current_state()
        ps = PoseStamped()
        ps.header.frame_id = "world"
        ps.pose = make_pose(pt)
        r.pose_stamped = ps
        fut = self.ik_srv.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        res = fut.result()
        if res.error_code.val != 1:
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[n] for n in ARM_JOINTS]

    def go_cartesian(self, pts, label, paint=None):
        req = GetCartesianPath.Request()
        req.header.frame_id = "world"
        req.start_state.joint_state = self.current_state()
        req.group_name = "arm"
        req.link_name = "paint_tcp"
        req.waypoints = [make_pose(p) for p in pts]
        req.max_step = 0.02
        req.jump_threshold = 0.0
        req.avoid_collisions = True

        fut = self.cart_srv.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        res = fut.result()

        traj = res.solution
        n = len(traj.joint_trajectory.points)

        if res.fraction < MIN_FRACTION or n < 2:
            print("  [FAILED] %s: %.1f%% feasible" %
                  (label, 100 * res.fraction))
            print("  MoveIt error code:", res.error_code.val)
            print("  Trajectory points:", n)
            print("  Checking individual path prefixes...")

            for i in range(1, len(pts) + 1):
                test = GetCartesianPath.Request()
                test.header.frame_id = "world"
                test.start_state.joint_state = self.current_state()
                test.group_name = "arm"
                test.link_name = "paint_tcp"
                test.waypoints = [make_pose(p) for p in pts[:i]]
                test.max_step = 0.02
                test.jump_threshold = 0.0
                test.avoid_collisions = True

                fut = self.cart_srv.call_async(test)
                rclpy.spin_until_future_complete(self, fut)
                check = fut.result()

                print("    Through waypoint %d: %.1f%%" %
                      (i, 100 * check.fraction))

            return False

        pos = [list(p.positions) for p in traj.joint_trajectory.points]
        jump = max(
            max(abs(a - b) for a, b in zip(pos[i], pos[i + 1]))
            for i in range(n - 1)
        )

        if jump > MAX_JUMP:
            print("  [FAILED] %s: joint jump %.2f rad" % (label, jump))
            return False

        retime(traj, JOINT_SPEED)

        goal = ExecuteTrajectory.Goal()
        goal.trajectory = traj

        if paint:
            self.painting = paint

        out = self.run_action(self.exec_ac, goal)
        self.painting = None
        self.wait(0.5)

        ok = out is not None and out.error_code.val == 1

        print("  [%s] %s (%d points, %.1f s)" %
              ("ok" if ok else "FAILED", label, n,
               traj.joint_trajectory.points[-1].time_from_start.sec))

        return ok

    def paint_stripe(self, s):
        pts = s["points"]
        rail_x = max(-RAIL_LIMIT, min(RAIL_LIMIT, s["rail_x"]))
        print("stripe %s  (rail_x=%.2f, %d points)" % (s["name"], rail_x, len(pts)))
        if not self.move_joints("rail", ["rail_joint"], [rail_x], "slide rail"):
            return False
        hover = [pts[0][0], pts[0][1], pts[0][2] + HOVER]
        q = self.ik(hover)
        if q is None:
            print("  [FAILED] no IK solution for the hover point")
            return False
        if not self.move_joints("arm", ARM_JOINTS, q, "approach (hover point)"):
            return False
        if not self.go_cartesian([pts[0]], "descend"):
            return False
        if not self.go_cartesian(pts[1:], "paint %s" % s["name"], paint=s["name"]):
            return False
        last = pts[-1]
        for up in (HOVER, 0.08, 0.04):
            if self.go_cartesian([[last[0], last[1], last[2] + up]], "lift %.2f m" % up):
                return True
        print("  [warn] could not lift; continuing - the next move is planned with collision checking")
        return True


def main():
    which = sys.argv[1] if len(sys.argv) > 1 else "roof_00"
    json_file = sys.argv[2] if len(sys.argv) > 2 else DEFAULT_JSON
    json_file = os.path.expanduser(json_file)

    if not os.path.isfile(json_file):
        print("JSON file not found:", json_file)
        return

    with open(json_file) as f:
        stripes = json.load(f)

    if which != "all":
        stripes = [s for s in stripes if s["name"] == which]

    if not stripes:
        print("no stripe called", which)
        return

    rclpy.init()
    node = Painter()
    node.wait(1.5)

    for s in stripes:
        ok = node.paint_stripe(s)
        print("=> %s %s" % (s["name"], "DONE" if ok else "STOPPED"))
        if not ok:
            break

    print("Trail stays on /paint_trail. Ctrl+C to quit.")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
