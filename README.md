# Gazebo 3D Vehicle Simulation

A Gazebo-based 3D vehicle simulation extension of the [ROS 2 Bicycle Gym Control Project](https://github.com/mahmoud-zoheir1/Control_Project).

## Overview

This repository contains an SDF vehicle model, a custom racetrack environment, mesh-generation utilities, and two ROS 2 Python nodes that connect the existing control-project topics to Gazebo vehicle commands and publish the reference path.

**Environment used during development:** Ubuntu 26.04, ROS 2 Lyrical, Gazebo, Python, SDF.

## Features

- Custom 3D vehicle and racetrack assets (`.sdf`, `.stl`).
- Track-generation scripts for the road, cones, centerline, and white boundary lines.
- A ROS 2 vehicle adapter that maps normalized throttle and steering commands to wheel-speed and steering-joint commands.
- A 0.5-second command watchdog that sends zero wheel-speed and steering commands when either input is stale.
- A ROS 2 reference-path publisher that reads the original Bicycle Gym centerline CSV and publishes a `nav_msgs/Path` at 1 Hz.

## Architecture

```text
Control_Project ROS 2 nodes
  /throttle (std_msgs/Float32) ----\
                                   > gazebo_vehicle_adapter.py
  /steer    (std_msgs/Float32) ----/      |
  /state    (nav_msgs/Odometry) ---------+  (speed read, not used for feedback yet)
                                         |
                          /car/rear_left_speed  (std_msgs/Float64)
                          /car/rear_right_speed (std_msgs/Float64)
                          /car/steer_left       (std_msgs/Float64)
                          /car/steer_right      (std_msgs/Float64)
                                         |
                          Gazebo vehicle joints (via configured bridge)

Control_Project/track_environment/tracks/centerline_0.csv
               |
      gazebo_path_publisher.py
               |
      /path (nav_msgs/Path)
```

The adapter currently performs **open-loop speed-command mapping**, not a closed-loop wheel-speed controller. The `/state` speed is subscribed to but is not used in its `update()` calculation. Gazebo joint-command transport requires the corresponding bridge and model plugin configuration.

## ROS 2 Interfaces

| Node | Subscribes | Publishes |
| --- | --- | --- |
| `gazebo_vehicle_adapter` | `/throttle` (`Float32`), `/steer` (`Float32`), `/state` (`Odometry`) | `/car/rear_left_speed`, `/car/rear_right_speed`, `/car/steer_left`, `/car/steer_right` (all `Float64`) |
| `gazebo_path_publisher` | — | `/path` (`nav_msgs/Path`) |

### Vehicle adapter parameters (currently hardcoded)

| Setting | Value |
| --- | --- |
| Wheel radius | 0.25 m |
| Maximum requested speed | 2.0 m/s |
| Steering limit | ±25° |
| Watchdog timeout | 0.5 s |
| Command update period | 0.05 s (20 Hz) |

For fresh commands, requested rear-wheel angular speed is `max(0, min(2.0, throttle * 2.0)) / 0.25` rad/s. Negative throttle currently requests zero speed rather than reverse or active braking. Both front steering command topics receive the same clamped steering angle.

## Repository Structure

```text
gazebo_car_project/
├── car_world.sdf
├── my_car.sdf
├── centerline.sdf
├── road_mesh_model.sdf
├── track_cones.sdf
├── white_lines.sdf
├── generated_track.sdf
├── *.stl
├── generate_track.py
├── generate_track_mesh.py
├── generate_centerline.py
├── generate_cones.py
├── generate_white_lines.py
├── gazebo_vehicle_adapter.py
└── gazebo_path_publisher.py
```

## Prerequisites

- ROS 2 Lyrical with `rclpy`, `std_msgs`, `nav_msgs`, and `geometry_msgs` available.
- Gazebo and the ROS–Gazebo bridge configured for the vehicle joint-command topics.
- The [Control_Project](https://github.com/mahmoud-zoheir1/Control_Project) checked out at `~/Control_Project`, because the path publisher currently reads `~/Control_Project/track_environment/tracks/centerline_0.csv`.

## Running the ROS 2 Nodes

The following commands start the two Python nodes; they **do not** start Gazebo, the vehicle model, the bridge, or the Bicycle Gym controllers. Those must be started separately using the verified setup for your installation.

```bash
source /opt/ros/lyrical/setup.bash
source ~/Control_Project/install/setup.bash
cd ~/gazebo_car_project
python3 gazebo_vehicle_adapter.py
```

In another terminal:

```bash
source /opt/ros/lyrical/setup.bash
cd ~/gazebo_car_project
python3 gazebo_path_publisher.py
```

> Note: Gazebo launch and bridge commands will be added after verifying the actual working commands and the `car_world.sdf` configuration.

## Screenshots

Screenshots of the Gazebo world, vehicle, and racetrack can be added under `images/` and linked here once captured.

## Related Project

- [Bicycle Gym — Autonomous Vehicle Control Project](https://github.com/mahmoud-zoheir1/Control_Project)

## Future Work

- Verify and document reproducible Gazebo launch and ROS–Gazebo bridge commands.
- Test the complete controller-to-Gazebo workflow and evaluate tracking behavior.
- Improve the vehicle dynamics and add feedback-based speed control where appropriate.
