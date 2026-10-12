
import csv
from pathlib import Path
from xml.sax.saxutils import escape

BASE = Path.home() / "gazebo_car_project"

CSV_FILE = (
    Path.home()
    / "Control_Project/track_environment/tracks/random_track0.csv"
)

OUTPUT = BASE / "track_cones.sdf"

parts = [
    '<?xml version="1.0"?>',
    '<sdf version="1.9">',
    '  <model name="track_boundary_cones">',
    '    <static>true</static>',
]

count = 0

with open(CSV_FILE, newline="") as f:
    reader = csv.reader(f)

    for row in reader:
        if len(row) < 3:
            continue

        color_name = row[0].strip().lower()

        if color_name not in ("blue", "yellow"):
            continue

        x = float(row[1])
        y = float(row[2])

        if color_name == "blue":
            color = "0.05 0.20 0.95 1"
        else:
            color = "1 0.85 0.05 1"

        parts.append(f"""
    <link name="cone_{count}">
      <pose>{x} {y} 0.15 0 0 0</pose>

      <visual name="body">
        <geometry>
          <cone>
            <radius>0.11</radius>
            <length>0.30</length>
          </cone>
        </geometry>

        <material>
          <ambient>{color}</ambient>
          <diffuse>{color}</diffuse>
        </material>
      </visual>
    </link>
""")

        count += 1

parts.extend([
    '  </model>',
    '</sdf>',
])

OUTPUT.write_text("\n".join(parts))

print(f"Generated {count} cones")
print(f"Saved to: {OUTPUT}")
