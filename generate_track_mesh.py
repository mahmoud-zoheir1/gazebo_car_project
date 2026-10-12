
import csv
import math
from pathlib import Path

import numpy as np
import trimesh

BASE = Path.home() / "gazebo_car_project"
TRACKS = Path.home() / "Control_Project/track_environment/tracks"

ROAD_WIDTH = 3.0
ROAD_THICKNESS = 0.04
ROAD_Z = 0.03

# Load original centerline
with open(TRACKS / "centerline_0.csv", newline="") as f:
    points = np.array([
        [float(row["x"]), float(row["y"])]
        for row in csv.DictReader(f)
    ])

# Remove duplicated final point
if np.linalg.norm(points[0] - points[-1]) < 0.01:
    points = points[:-1]

n = len(points)

# Tangent and normal at each waypoint
prev_pts = np.roll(points, 1, axis=0)
next_pts = np.roll(points, -1, axis=0)

tangents = next_pts - prev_pts
lengths = np.linalg.norm(tangents, axis=1)

if np.any(lengths < 1e-8):
    raise ValueError("Duplicate or degenerate centerline points")

tangents = tangents / lengths[:, None]

normals = np.column_stack((-tangents[:, 1], tangents[:, 0]))

left = points + normals * (ROAD_WIDTH / 2)
right = points - normals * (ROAD_WIDTH / 2)

# Each point has 4 vertices:
# top-left, top-right, bottom-left, bottom-right
vertices = []

for i in range(n):
    vertices.extend([
        [left[i, 0], left[i, 1], ROAD_Z],
        [right[i, 0], right[i, 1], ROAD_Z],
        [left[i, 0], left[i, 1], ROAD_Z - ROAD_THICKNESS],
        [right[i, 0], right[i, 1], ROAD_Z - ROAD_THICKNESS],
    ])

faces = []

for i in range(n):
    j = (i + 1) % n

    a, b, c, d = 4*i, 4*i+1, 4*i+2, 4*i+3
    e, f, g, h = 4*j, 4*j+1, 4*j+2, 4*j+3

    # Top surface
    faces.extend([[a, b, f], [a, f, e]])

    # Bottom surface
    faces.extend([[c, g, h], [c, h, d]])

    # Left wall
    faces.extend([[a, e, g], [a, g, c]])

    # Right wall
    faces.extend([[b, d, h], [b, h, f]])

# Close end walls are unnecessary for a closed circuit
mesh = trimesh.Trimesh(
    vertices=np.array(vertices),
    faces=np.array(faces),
    process=False
)

mesh.export(BASE / "road_mesh.stl")

print("Road mesh generated successfully!")
print(f"Centerline points: {n}")
print(f"Vertices: {len(vertices)}")
print(f"Triangles: {len(faces)}")
print(f"Output: {BASE / 'road_mesh.stl'}")
