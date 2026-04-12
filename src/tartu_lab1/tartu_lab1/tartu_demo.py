import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import Twist
from turtlesim.action import RotateAbsolute


class TartuDemo(Node):
    def __init__(self):
        super().__init__('tartu_demo')
        self.sub = self.create_subscription(
            Twist, '/tartu/cmd_vel', self.cmd_callback, 10)
        self._action_server = ActionServer(
            self, RotateAbsolute, 'turtle1/rotate_absolute', self.execute_cb)
        self.get_logger().info('tartu_demo started')

    def cmd_callback(self, msg):
        self.get_logger().info(
            f'Received cmd_vel: linear={msg.linear.x:.2f} angular={msg.angular.z:.2f}')

    async def execute_cb(self, goal_handle):
        self.get_logger().info(f'Rotating to {goal_handle.request.theta:.2f} rad')
        feedback = RotateAbsolute.Feedback()
        feedback.remaining = goal_handle.request.theta
        goal_handle.publish_feedback(feedback)
        goal_handle.succeed()
        result = RotateAbsolute.Result()
        result.delta = goal_handle.request.theta
        return result


def main(args=None):
    rclpy.init(args=args)
    node = TartuDemo()
    rclpy.spin(node)
    rclpy.shutdown()
