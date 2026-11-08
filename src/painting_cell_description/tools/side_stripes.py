#!/usr/bin/env python3
"""Build horizontal stripes for the near side panel (doors), in the CAR frame.
The gun points along +y (towards the car side) and sits STANDOFF metres in front of the surface."""
import json
import os
import sys

import numpy as np
import trimesh
import rclpy

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import roof_stripes as rs

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "side_stripes.json")
X_MIN, X_MAX = -1.0, 0.5                 # door region along the car (m)
Z_LEVELS = np.linspace(0.55, 0.95, 4)    # stripe heights (m)
POINT_STEP = 0.10                        # waypoint spacing along a stripe (m)
MIN_NORMAL_Y = 0.85                      # surface must face sideways to count as door


def side_hits(mesh, xs, zs):
    """Shoot a ray along +y at each (x, z). Return y and sideways-ness of the FIRST surface hit."""
    origins = np.column_stack([xs, np.full(len(xs), -3.0), zs])
    dirs = np.tile([0.0, 1.0, 0.0], (len(xs), 1))
    locs, ray_idx, tri_idx = mesh.ray.intersects_location(origins, dirs, multiple_hits=True)
    y = np.full(len(xs), np.inf)
    ny = np.zeros(len(xs))
    for loc, r, t in zip(locs, ray_idx, tri_idx):
        if loc[1] < y[r]:
            y[r] = loc[1]
            ny[r] = abs(mesh.face_normals[t][1])
    y[np.isinf(y)] = np.nan
    return y, ny


mesh = trimesh.load(rs.CAR_STL, force="mesh")
xs = np.arange(X_MIN, X_MAX + 1e-9, POINT_STEP)
stripes = []
for k, z in enumerate(Z_LEVELS):
    y, ny = side_hits(mesh, xs, np.full_like(xs, z))
    ok = (ny >= MIN_NORMAL_Y) & (y < -0.7) & (y > -1.2)
    print("  z=%.2f: %d of %d points are flat side panel" % (z, ok.sum(), len(ok)))
    if ok.sum() < 5:
        print("  stripe at z=%.2f skipped (not enough door)" % z)
        continue
    x_ok, y_ok = xs[ok], y[ok]
    pts = np.column_stack([x_ok, y_ok - rs.STANDOFF, np.full_like(x_ok, z)])
    if k % 2:
        pts = pts[::-1]
    stripes.append({
        "name": "side_%02d" % len(stripes),
        "surface": "side",
        "rail_x": round(float(x_ok.mean()), 3),
        "points": pts.round(4).tolist(),
    })

print("built %d side stripes" % len(stripes))
for s in stripes:
    ys = [p[1] for p in s["points"]]
    print("  %s  rail_x=%5.2f  points=%d  gun y %.2f..%.2f" %
          (s["name"], s["rail_x"], len(s["points"]), min(ys), max(ys)))
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
