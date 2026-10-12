
import csv
import math
from pathlib import Path
from xml.sax.saxutils import escape

TRACK_DIR = Path.home() / "Control_Project/track_environment/tracks"
OUTPUT = Path.home() / "gazebo_car_project/generated_track.sdf"

# Read original centerline
with open(TRACK_DIR / "centerline_0.csv", newline="") as f:
    reader = csv.DictReader(f)
    points = [(float(r["x"]), float(r["y"])) for r in reader]

# Remove duplicated closing point if present
if len(points) > 2 and math.dist(points[0], points[-1]) < 0.01:
    points.pop()

# Read original boundary markers
markers = []
with open(TRACK_DIR / "random_track0.csv", newline="") as f:
    for row in csv.reader(f):
        if len(row) >= 3 and row[0] in ("blue", "yellow"):
            markers.append((row[0], float(row[1]), float(row[2])))

if len(points) < 3:
    raise ValueError("Not enough centerline points")

# Approximate road width for the first visual prototype.
# We'll verify this against the boundary markers later.
ROAD_WIDTH = 5.0
ROAD_THICKNESS = 0.04
STEP = 4  # Reduce number of segments for performance

sampled = points[::STEP]
if math.dist(sampled[-1], points[-1]) > 0.01:
    sampled.append(points[-1])

parts = ['''<?xml version="1.0"?>
<sdf version="1.9">
<model name="race_track">
  <static>true</static>
''']

# Build flat asphalt segments along the centerline
for i, (x1, y1) in enumerate(sampled):
    x2, y2 = sampled[(i + 1) % len(sampled)]
    length = math.hypot(x2 - x1, y2 - y1)
    if length < 0.001:
        continue

    mx, my = (x1 + x2) / 2, (y1 + y2) / 2
    yaw = math.atan2(y2 - y1, x2 - x1)

    parts.append(f'''
  <link name="road_{i}">
    <pose>{mx} {my} 0.005 0 0 {yaw}</pose>
    <collision name="collision">
      <geometry>
        <box><size>{length + 0.15} {ROAD_WIDTH} {ROAD_THICKNESS}</size></box>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <box><size>{length + 0.15} {ROAD_WIDTH} {ROAD_THICKNESS}</size></box>
      </geometry>
      <material>
        <ambient>0.12 0.12 0.12 1</ambient>
        <diffuse>0.16 0.16 0.16 1</diffuse>
      </material>
    </visual>
  </link>
''')

# Add blue/yellow boundary markers
for i, (color, x, y) in enumerate(markers):
    rgb = "0.05 0.25 1 1" if color == "blue" else "1 0.85 0.05 1"

    parts.append(f'''
  <link name="marker_{i}">
    <pose>{x} {y} 0.15 0 0 0</pose>
    <visual name="visual">
      <geometry>
        <cylinder>
          <radius>0.12</radius>
          <length>0.30</length>
        </cylinder>
      </geometry>
      <material>
        <ambient>{rgb}</ambient>
        <diffuse>{rgb}</diffuse>
      </material>
    </visual>
  </link>
''')

parts.append("</model>\n</sdf>\n")
OUTPUT.write_text("".join(parts))

print(f"Generated: {OUTPUT}")
print(f"Centerline points: {len(points)}")
print(f"Road segments: {len(sampled)}")
print(f"Boundary markers: {len(markers)}")
