#!/usr/bin/env python3
"""Paint the NEAR SIDE PANEL (doors) with the gun pointing sideways.
Sequence: park robot -> slide car away from the robot -> horizontal stripes -> park -> slide car back."""
import json
import os
import sys

import rclpy
from geometry_msgs.msg import Pose

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import paint_roof as pr

TOOLS_DIR = os.path.dirname(os.path.abspath(__file__))
SIDE_JSON = os.path.join(TOOLS_DIR, "side_stripes.json")
CAR_Y = 1.0            # m, how far the car is slid away from the robot
PARK_RAIL_X = 3.2      # m, far end of the rail


def side_pose(p):
    """Gun pointing along world +y (towards the car side): a -90 degree rotation about x."""
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = float(p[0]), float(p[1]), float(p[2])
    pose.orientation.x, pose.orientation.y = -0.70710678, 0.0
    pose.orientation.z, pose.orientation.w = 0.0, 0.70710678
    return pose


pr.make_pose = side_pose      # every IK and Cartesian call in the painter now uses the sideways gun


def to_world(stripe, car_y):
    """Car frame -> world frame (turntable stays at 0, only the y-slide moves the car)."""
    pts = [[x, y + car_y, z] for x, y, z in stripe["points"]]
    return {"name": stripe["name"], "rail_x": sum(p[0] for p in pts) / len(pts), "points": pts}


def back_off(p, dist):
    """A point 'dist' metres away from the car side, towards the robot."""
    return [p[0], p[1] - dist, p[2]]


def park(node):
    ok = node.move_joints("arm", pr.ARM_JOINTS, [0.0] * 6, "tuck arm")
    return ok and node.move_joints("rail", ["rail_joint"], [PARK_RAIL_X], "park rail at the end")


def paint_side_stripe(node, s):
    pts = s["points"]
    rail_x = max(-pr.RAIL_LIMIT, min(pr.RAIL_LIMIT, s["rail_x"]))
    print("stripe %s  (rail_x=%.2f, %d points)" % (s["name"], rail_x, len(pts)))
    if not node.move_joints("rail", ["rail_joint"], [rail_x], "slide rail"):
        return False
    q = node.ik(back_off(pts[0], pr.HOVER))
    if q is None:
        print("  [FAILED] no IK solution for the hover point")
        return False
    if not node.move_joints("arm", pr.ARM_JOINTS, q, "approach (hover point)"):
        return False
    if not node.go_cartesian([pts[0]], "move in to the door"):
        return False
    if not node.go_cartesian(pts[1:], "paint %s" % s["name"], paint=s["name"]):
        return False
    for dist in (pr.HOVER, 0.08, 0.04):
        if node.go_cartesian([back_off(pts[-1], dist)], "back off %.2f m" % dist):
            return True
    print("  [warn] could not back off; continuing - the next move is planned with collision checking")
    return True


def main():
    with open(SIDE_JSON) as f:
        stripes = json.load(f)
    rclpy.init()
    node = pr.Painter()
    node.trail_offset = (0.0, pr.SPRAY_DIST, 0.0)   # trail is drawn 0.28 m in FRONT of the nozzle (car +y)
    node.wait(1.5)
    print("1) park the robot, then slide the car away from it")
    if not park(node):
        print("=> STOPPED while parking")
        return
    if not node.move_joints("positioner", ["car_y_joint", "turntable_joint"], [CAR_Y, 0.0],
                            "slide car away (y = %.1f m)" % CAR_Y):
        print("=> STOPPED while sliding the car")
        return
    node.wait(1.0)
    car_y = node.joints["car_y_joint"]
    print("   car_y = %.3f m" % car_y)
    print("2) paint the side panel")
    for s in stripes:
        ok = paint_side_stripe(node, to_world(s, car_y))
        print("=> %s %s" % (s["name"], "DONE" if ok else "STOPPED"))
        if not ok:
            break
    print("3) park the robot and slide the car back")
    if park(node):
        node.move_joints("positioner", ["car_y_joint", "turntable_joint"], [0.0, 0.0], "slide car back")
    print("Trail stays on /paint_trail.  Ctrl+C to quit.")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
