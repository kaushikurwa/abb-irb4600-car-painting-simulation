import os
import numpy as np
import trimesh
import fast_simplification

SRC = os.path.expanduser("~/Downloads/nissan-altima-2013-1.snapshot.1/Nissan Altima 2013.stl")
OUT = os.path.expanduser("~/yuktii_ws/src/painting_cell_description/meshes/car")
TARGET_LENGTH = 4.8  # metres

m = trimesh.load(SRC, force='mesh')

# length is along Y -> rotate 90 deg about Z so length runs along X
m.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2, [0, 0, 1]))

# uniform scale to 4.8 m long
m.apply_scale(TARGET_LENGTH / m.extents[0])

# centre in X/Y, wheels on z = 0
b = m.bounds
m.apply_translation([-(b[0][0] + b[1][0]) / 2, -(b[0][1] + b[1][1]) / 2, -b[0][2]])
print("final extents (L,W,H):", m.extents)

def decimate(mesh, reduction, name):
    pts, faces = fast_simplification.simplify(
        np.asarray(mesh.vertices, dtype=np.float32),
        np.asarray(mesh.faces, dtype=np.int32),
        target_reduction=reduction)
    out = trimesh.Trimesh(vertices=pts, faces=faces)
    out.export(os.path.join(OUT, name))
    print(name, "faces:", len(out.faces), "extents:", out.extents)

decimate(m, 0.98, "car_visual.stl")
decimate(m, 0.997, "car_collision.stl")
