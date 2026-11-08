import os
import numpy as np
import trimesh
import fast_simplification

D = os.path.expanduser("~/yuktii_ws/src/painting_cell_description/meshes/car")
src = trimesh.load(os.path.join(D, "car_visual.stl"), force='mesh')

def shrink(mesh, target_faces):
    pts = np.asarray(mesh.vertices, dtype=np.float32)
    f = np.asarray(mesh.faces, dtype=np.int32)
    for _ in range(10):
        if len(f) <= target_faces:
            break
        red = min(1 - target_faces / len(f), 0.9)
        p2, f2 = fast_simplification.simplify(pts, f, target_reduction=red, agg=10)
        if len(f2) >= 0.98 * len(f):
            print("  decimation stalled at", len(f2), "faces")
            break
        pts, f = p2, f2
    return trimesh.Trimesh(vertices=pts, faces=f)

def ground(mesh):
    b = mesh.bounds
    mesh.apply_translation([-(b[0][0] + b[1][0]) / 2, -(b[0][1] + b[1][1]) / 2, -b[0][2]])
    return mesh

vis = ground(shrink(src, 40000))
vis.export(os.path.join(D, "car_visual.stl"))
print("car_visual.stl faces:", len(vis.faces), "extents:", vis.extents)

col = shrink(src, 5000)
if len(col.faces) > 20000:
    print("  using convex hull for collision")
    col = src.convex_hull
col = ground(col)
col.export(os.path.join(D, "car_collision.stl"))
print("car_collision.stl faces:", len(col.faces), "extents:", col.extents)
