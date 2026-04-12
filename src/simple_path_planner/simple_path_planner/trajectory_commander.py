import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

LINEAR_SPEED = 0.2   # m/s
SIDE = 1.0           # m
TURN_SPEED = 0.5     # rad/s (in-place turns)
CIRCLE_RADIUS = 0.5  # m


class TrajectoryCommander(Node):
    def __init__(self):
        super().__init__('trajectory_commander')
        self.declare_parameter('path', 'square')
        path = self.get_parameter('path').get_parameter_value().string_value

        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.sequence = self._build_sequence(path)
        self.step = 0
        self.step_start = self.get_clock().now()
        self.create_timer(0.05, self._tick)
        self.get_logger().info(f'trajectory_commander started, path={path}')

    def _build_sequence(self, path):
        straight_t = SIDE / LINEAR_SPEED                  # 5.0 s
        turn_t = (math.pi / 2) / TURN_SPEED              # ~3.14 s

        if path == 'square':
            return [(LINEAR_SPEED, 0.0, straight_t),
                    (0.0, TURN_SPEED, turn_t)] * 4

        if path == 'nh_square':
            r = 0.3
            omega = LINEAR_SPEED / r
            arc_t = (math.pi / 2) / omega
            return [(LINEAR_SPEED, 0.0, straight_t),
                    (LINEAR_SPEED, omega, arc_t)] * 4

        if path == 'circle':
            omega = LINEAR_SPEED / CIRCLE_RADIUS
            circle_t = 2 * math.pi * CIRCLE_RADIUS / LINEAR_SPEED
            return [(LINEAR_SPEED, omega, circle_t)]

        if path == 'eight':
            omega = LINEAR_SPEED / CIRCLE_RADIUS
            circle_t = 2 * math.pi * CIRCLE_RADIUS / LINEAR_SPEED
            return [(LINEAR_SPEED, omega, circle_t),
                    (LINEAR_SPEED, -omega, circle_t)]

        self.get_logger().error(f'Unknown path: {path}')
        return []

    def _tick(self):
        if self.step >= len(self.sequence):
            self.pub.publish(Twist())  # stop
            return

        linear, angular, duration = self.sequence[self.step]
        elapsed = (self.get_clock().now() - self.step_start).nanoseconds / 1e9

        if elapsed >= duration:
            self.step += 1
            self.step_start = self.get_clock().now()
            return

        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(TrajectoryCommander())
    rclpy.shutdown()
