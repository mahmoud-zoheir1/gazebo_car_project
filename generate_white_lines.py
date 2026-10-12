
import csv
from pathlib import Path
import numpy as np
import trimesh

BASE = Path.home() / "gazebo_car_project"
TRACK = Path.home() / "Control_Project/track_environment/tracks/centerline_0.csv"

ROAD_WIDTH = 3.0
LINE_WIDTH = 0.08
LINE_Z = 0.045

with open(TRACK, newline="") as f:
    points = np.array([
        [float(row["x"]), float(row["y"])]
        for row in csv.DictReader(f)
    ])

if np.linalg.norm(points[0] - points[-1]) < 0.01:
    points = points[:-1]

n = len(points)

prev_pts = np.roll(points, 1, axis=0)
next_pts = np.roll(points, -1, axis=0)

tangents = next_pts - prev_pts
lengths = np.linalg.norm(tangents, axis=1)
if np.any(lengths < 1e-8):
    raise ValueError("Degenerate centerline points")

tangents /= lengths[:, None]
normals = np.column_stack((-tangents[:, 1], tangents[:, 0]))

def make_line(offset, filename):
    center = points + normals * offset
    inner = center - normals * (LINE_WIDTH / 2)
    outer = center + normals * (LINE_WIDTH / 2)

    vertices = []
    faces = []

    for i in range(n):
        vertices.extend([
            [inner[i, 0], inner[i, 1], LINE_Z],
            [outer[i, 0], outer[i, 1], LINE_Z],
        ])

    for i in range(n):
        j = (i + 1) % n
        a, b = 2*i, 2*i+1
        c, d = 2*j, 2*j+1

        faces.extend([[a, d, b], [a, c, d]])

    mesh = trimesh.Trimesh(
        vertices=np.array(vertices),
        faces=np.array(faces),
        process=False
    )

    mesh.export(BASE / filename)

make_line(ROAD_WIDTH / 2 - 0.12, "white_line_left.stl")
make_line(-ROAD_WIDTH / 2 + 0.12, "white_line_right.stl")

print("White boundary lines generated successfully")
