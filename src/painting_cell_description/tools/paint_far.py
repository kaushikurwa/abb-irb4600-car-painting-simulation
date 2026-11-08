#!/usr/bin/env python3
"""Paint the FAR half of the roof: park robot -> turn car ~180 deg -> paint -> park -> turn car back.
Stripes are stored in the CAR frame; they are converted to the WORLD frame using the real turntable angle."""
import json
import math
import os
import sys

import rclpy

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import paint_roof as pr

TOOLS_DIR = os.path.dirname(os.path.abspath(__file__))
FAR_JSON = os.path.join(TOOLS_DIR, "roof_far_stripes.json")
PARK_RAIL_X = 3.2      # m, far end of the rail
TURN = 3.14            # rad, just under the 3.1416 joint limit


def to_world(stripe, theta, car_y):
    """Rotate the car-frame points by the turntable angle, then shift by the car's y offset."""
    c, s = math.cos(theta), math.sin(theta)
    pts = [[c * x - s * y, s * x + c * y + car_y, z] for x, y, z in stripe["points"]]
    return {"name": stripe["name"], "rail_x": pts[0][0], "points": pts}


def park(node):
    ok = node.move_joints("arm", pr.ARM_JOINTS, [0.0] * 6, "tuck arm")
    return ok and node.move_joints("rail", ["rail_joint"], [PARK_RAIL_X], "park rail at the end")


def main():
    with open(FAR_JSON) as f:
        stripes = json.load(f)
    rclpy.init()
    node = pr.Painter()
    node.wait(1.5)
    print("1) park the robot, then turn the car")
    if not park(node):
        print("=> STOPPED while parking")
        return
    if not node.move_joints("positioner", ["car_y_joint", "turntable_joint"], [0.0, TURN], "turn car ~180 deg"):
        print("=> STOPPED while turning the car")
        return
    node.wait(1.0)
    theta = node.joints["turntable_joint"]
    car_y = node.joints["car_y_joint"]
    print("   turntable angle = %.3f rad, car_y = %.3f m" % (theta, car_y))
    print("2) paint the far half of the roof (now facing the robot)")
    for s in stripes:
        ok = node.paint_stripe(to_world(s, theta, car_y))
        print("=> %s %s" % (s["name"], "DONE" if ok else "STOPPED"))
        if not ok:
            break
    print("3) park the robot and turn the car back")
    if park(node):
        node.move_joints("positioner", ["car_y_joint", "turntable_joint"], [0.0, 0.0], "turn car back to 0")
    print("Trail stays on /paint_trail.  Ctrl+C to quit.")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
