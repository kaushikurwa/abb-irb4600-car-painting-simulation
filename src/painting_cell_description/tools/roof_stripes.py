#!/usr/bin/env python3
"""Stage A: build roof stripes (in the car frame), save to JSON, show as RViz markers."""
import json
import os

import numpy as np
import trimesh
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

TOOLS_DIR = os.path.dirname(os.path.abspath(__file__))
PACKAGE_DIR = os.path.dirname(TOOLS_DIR)

CAR_STL = os.path.join(PACKAGE_DIR, "meshes", "car", "car_visual.stl")
OUT_JSON = os.path.join(TOOLS_DIR, "roof_stripes.json")

STANDOFF = 0.28       # nozzle to surface distance (m)
STRIPE_STEP = 0.25    # distance between neighbouring stripes (m)
POINT_STEP = 0.10     # distance between waypoints along a stripe (m)
ROOF_MIN_Z = 1.30     # surface higher than this counts as roof
MIN_NORMAL_Z = 0.93   # surface must be this flat (about 21 degrees or less) to count as roof
Y_NEAR, Y_MID = -0.65, 0.0   # near half of the roof (facing the robot when turntable = 0)


def surface_z(mesh, xs, ys):
    """Shoot a ray straight down at each (x, y). Returns (height, flatness) of the highest hit.
    flatness = |normal_z|: 1.0 is perfectly flat, smaller means more sloped (glass, hood edge)."""
    origins = np.column_stack([xs, ys, np.full(len(xs), 3.0)])
    dirs = np.tile([0.0, 0.0, -1.0], (len(xs), 1))
    locs, ray_idx, tri_idx = mesh.ray.intersects_location(origins, dirs, multiple_hits=True)
    z = np.full(len(xs), -np.inf)
    nz = np.zeros(len(xs))
    for loc, r, t in zip(locs, ray_idx, tri_idx):
        if loc[2] > z[r]:
            z[r] = loc[2]
            nz[r] = abs(mesh.face_normals[t][2])
    z[np.isinf(z)] = np.nan
    return z, nz


def build_roof_stripes(mesh):
    stripes = []
    for k, x in enumerate(np.arange(-1.2, 0.61, STRIPE_STEP)):
        ys = np.arange(Y_NEAR, Y_MID + 1e-9, POINT_STEP)
        z, nz = surface_z(mesh, np.full_like(ys, x), ys)
        ok = (z >= ROOF_MIN_Z) & (nz >= MIN_NORMAL_Z)
        print("  x=%5.2f: %d of %d points are flat roof" % (x, ok.sum(), len(ok)))
        if ok.sum() < 3:
            print("stripe at x=%.2f skipped (not enough roof)" % x)
            continue
        ys, z = ys[ok], z[ok]
        pts = np.column_stack([np.full_like(ys, x), ys, z + STANDOFF])
        if k % 2:                      # zig-zag: every second stripe goes the other way
            pts = pts[::-1]
        stripes.append({
            "name": "roof_%02d" % k,
            "surface": "roof",
            "rail_x": round(float(x), 3),
            "tool_dir": [0.0, 0.0, -1.0],   # gun points straight down
            "points": pts.round(4).tolist(),
        })
    return stripes


class StripeMarkers(Node):
    def __init__(self, stripes):
        super().__init__("stripe_markers")
        self.stripes = stripes
        self.pub = self.create_publisher(MarkerArray, "/paint_waypoints", 10)
        self.create_timer(1.0, self.tick)

    def tick(self):
        arr = MarkerArray()
        for i, s in enumerate(self.stripes):
            mk = Marker()
            mk.header.frame_id = "car"      # glued to the car, so they turn with the turntable
            mk.ns = "stripes"
            mk.id = i
            mk.type = Marker.LINE_STRIP
            mk.action = Marker.ADD
            mk.scale.x = 0.012
            mk.color.r, mk.color.g, mk.color.b, mk.color.a = 1.0, 0.5, 0.0, 1.0
            mk.pose.orientation.w = 1.0
            mk.points = [Point(x=p[0], y=p[1], z=p[2]) for p in s["points"]]
            arr.markers.append(mk)
        self.pub.publish(arr)


def main():
    mesh = trimesh.load(CAR_STL, force="mesh")
    stripes = build_roof_stripes(mesh)
    print("built %d stripes" % len(stripes))
    for s in stripes:
        zs = [p[2] for p in s["points"]]
        print("  %s  rail_x=%5.2f  points=%d  tcp z %.2f..%.2f" %
              (s["name"], s["rail_x"], len(s["points"]), min(zs), max(zs)))
    with open(OUT_JSON, "w") as f:
        json.dump(stripes, f, indent=1)
    print("saved", OUT_JSON)

    rclpy.init()
    node = StripeMarkers(stripes)
    print("publishing /paint_waypoints  (Ctrl+C to stop)")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
