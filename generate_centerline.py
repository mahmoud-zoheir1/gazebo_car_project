
import csv
from pathlib import Path

import numpy as np
import trimesh

BASE = Path.home() / "gazebo_car_project"
TRACK = Path.home() / "Control_Project/track_environment/tracks/centerline_0.csv"

LINE_WIDTH = 0.055
LINE_Z = 0.06
DASH_LENGTH = 0.65
GAP_LENGTH = 0.65

with open(TRACK, newline="") as f:
    points = np.array([
        [float(r["x"]), float(r["y"])]
        for r in csv.DictReader(f)
    ], dtype=float)

if np.linalg.norm(points[0] - points[-1]) < 0.01:
    points = points[:-1]

# Close the racetrack
closed = np.vstack([points, points[0]])

segments = np.diff(closed, axis=0)
lengths = np.linalg.norm(segments, axis=1)

vertices = []
faces = []

period = DASH_LENGTH + GAP_LENGTH
distance = 0.0

for i, segment in enumerate(segments):
    length = lengths[i]

    if length < 1e-8:
        continue

    direction = segment / length
    normal = np.array([-direction[1], direction[0]])

    local = 0.0

    while local < length - 1e-9:
        phase = (distance + local) % period

        if phase < DASH_LENGTH:
            step = min(length - local, DASH_LENGTH - phase)
            draw = True
        else:
            step = min(length - local, period - phase)
            draw = False

        if step < 1e-8:
            break

        if draw:
            p0 = points[i] + direction * local
            p1 = points[i] + direction * (local + step)

            half = normal * LINE_WIDTH / 2

            a = p0 - half
            b = p0 + half
            c = p1 - half
            d = p1 + half

            k = len(vertices)

            vertices.extend([
                [*a, LINE_Z],
                [*b, LINE_Z],
                [*c, LINE_Z],
                [*d, LINE_Z],
            ])

            # Upward-facing triangles
            faces.extend([
                [k, k + 2, k + 1],
                [k + 1, k + 2, k + 3],
            ])

        local += step

    distance += length

mesh = trimesh.Trimesh(
    vertices=np.asarray(vertices),
    faces=np.asarray(faces),
    process=False
)

output = BASE / "centerline_mesh.stl"
mesh.export(output)

print(f"Centerline generated: {output}")
print(f"Triangles: {len(faces)}")
