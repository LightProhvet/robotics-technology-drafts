import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Twist
from action_msgs.msg import GoalStatusArray
from turtlesim.action import RotateAbsolute


class TeleopTurtle(Node):
    def __init__(self):
        super().__init__('teleop_turtle')
        self.pub = self.create_publisher(Twist, '/tartu/cmd_vel', 10)
        self.timer = self.create_timer(1.0, self.send_cmd)

        # status uses transient local durability
        status_qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            GoalStatusArray,
            'turtle1/rotate_absolute/_action/status',
            self.status_cb,
            status_qos)
        self.create_subscription(
            RotateAbsolute.Impl.FeedbackMessage,
            'turtle1/rotate_absolute/_action/feedback',
            self.feedback_cb,
            10)
        self.get_logger().info('teleop_turtle started')

    def send_cmd(self):
        msg = Twist()
        msg.linear.x = 1.0
        msg.angular.z = 0.5
        self.pub.publish(msg)

    def status_cb(self, msg):
        self.get_logger().info(f'Status: {msg.status_list}')

    def feedback_cb(self, msg):
        self.get_logger().info(f'Feedback remaining: {msg.feedback.remaining:.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = TeleopTurtle()
    rclpy.spin(node)
    rclpy.shutdown()
