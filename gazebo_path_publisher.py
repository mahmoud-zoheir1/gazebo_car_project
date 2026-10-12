import csv
import math
from pathlib import Path

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path as RosPath
from geometry_msgs.msg import PoseStamped


class GazeboPathPublisher(Node):

    def __init__(self):
        super().__init__('gazebo_path_publisher')

        self.publisher = self.create_publisher(
            RosPath, '/path', 10
        )

        csv_file = (
            Path.home()
            / 'Control_Project'
            / 'track_environment'
            / 'tracks'
            / 'centerline_0.csv'
        )

        with open(csv_file, newline='') as f:
            rows = list(csv.DictReader(f))

        self.points = [
            (float(row['x']), float(row['y']))
            for row in rows
        ]

        self.timer = self.create_timer(1.0, self.publish_path)

        self.get_logger().info(
            f'Loaded {len(self.points)} track waypoints'
        )

    def publish_path(self):
        msg = RosPath()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        n = len(self.points)

        for i, (x, y) in enumerate(self.points):
            next_x, next_y = self.points[(i + 1) % n]

            yaw = math.atan2(
                next_y - y,
                next_x - x
            )

            pose = PoseStamped()
            pose.header = msg.header

            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0

            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)

            msg.poses.append(pose)

        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = GazeboPathPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
