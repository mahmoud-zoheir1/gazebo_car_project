import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float64
from nav_msgs.msg import Odometry


class GazeboVehicleAdapter(Node):

    def __init__(self):
        super().__init__('gazebo_vehicle_adapter')

        self.wheel_radius = 0.25
        self.max_speed = 2.0
        self.max_steer = math.radians(25)

        self.throttle = 0.0
        self.steer = 0.0
        self.speed = 0.0

        self.last_throttle_time = None
        self.last_steer_time = None
        self.timeout = 0.5

        self.create_subscription(
            Float32, '/throttle', self.throttle_callback, 10
        )

        self.create_subscription(
            Float32, '/steer', self.steer_callback, 10
        )

        self.create_subscription(
            Odometry, '/state', self.state_callback, 10
        )

        self.left_speed_pub = self.create_publisher(
            Float64, '/car/rear_left_speed', 10
        )

        self.right_speed_pub = self.create_publisher(
            Float64, '/car/rear_right_speed', 10
        )

        self.left_steer_pub = self.create_publisher(
            Float64, '/car/steer_left', 10
        )

        self.right_steer_pub = self.create_publisher(
            Float64, '/car/steer_right', 10
        )

        self.create_timer(0.05, self.update)

        self.get_logger().info('Gazebo Vehicle Adapter Started')

    def throttle_callback(self, msg):
        self.throttle = max(-1.0, min(1.0, msg.data))
        self.last_throttle_time = self.get_clock().now()

    def steer_callback(self, msg):
        self.steer = max(
            -self.max_steer,
            min(self.max_steer, msg.data)
        )
        self.last_steer_time = self.get_clock().now()

    def state_callback(self, msg):
        self.speed = msg.twist.twist.linear.x

    def update(self):
        now = self.get_clock().now()

        commands_fresh = (
            self.last_throttle_time is not None
            and self.last_steer_time is not None
            and (now - self.last_throttle_time).nanoseconds < self.timeout * 1e9
            and (now - self.last_steer_time).nanoseconds < self.timeout * 1e9
        )

        if commands_fresh:
            # Convert normalized throttle into a speed request.
            # Simple proportional mapping for first integration test.
            target_speed = max(
                0.0,
                min(self.max_speed, self.throttle * self.max_speed)
            )
            steering = self.steer
        else:
            target_speed = 0.0
            steering = 0.0

        wheel_angular_speed = target_speed / self.wheel_radius

        speed_msg = Float64()
        speed_msg.data = wheel_angular_speed

        steer_msg = Float64()
        steer_msg.data = steering

        self.left_speed_pub.publish(speed_msg)
        self.right_speed_pub.publish(speed_msg)

        self.left_steer_pub.publish(steer_msg)
        self.right_steer_pub.publish(steer_msg)


def main():
    rclpy.init()
    node = GazeboVehicleAdapter()

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
