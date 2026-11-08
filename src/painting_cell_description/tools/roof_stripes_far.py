#!/usr/bin/env python3
"""Build stripes for the FAR half of the roof (car frame y = 0 .. +0.65) and save them to JSON."""
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import trimesh
import rclpy
import roof_stripes as rs

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "roof_far_stripes.json")

rs.Y_NEAR, rs.Y_MID = 0.0, 0.65
mesh = trimesh.load(rs.CAR_STL, force="mesh")
stripes = rs.build_roof_stripes(mesh)
for i, s in enumerate(stripes):
    s["name"] = "far_%02d" % i
    s["surface"] = "roof_far"
print("built %d far-half stripes" % len(stripes))
for s in stripes:
    zs = [p[2] for p in s["points"]]
    print("  %s  points=%d  tcp z %.2f..%.2f" % (s["name"], len(s["points"]), min(zs), max(zs)))
with open(OUT, "w") as f:
    json.dump(stripes, f, indent=1)
print("saved", OUT)

rclpy.init()
node = rs.StripeMarkers(stripes)
print("publishing /paint_waypoints  (Ctrl+C to stop)")
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
