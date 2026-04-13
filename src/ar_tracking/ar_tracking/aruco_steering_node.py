import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseArray


def yaw_from_quaternion(q):
    """Extract yaw (rotation around Z) from a quaternion.

    Derived from the rotation matrix Z-column:
      R[1,0] = 2(wx*wz + wx*wy) → numerator:   2*(w*z + x*y)
      R[0,0] = 1 - 2(y² + z²)  → denominator: 1 - 2*(y² + z²)
    atan2(R[1,0], R[0,0]) gives the angle of the projected X-axis,
    which is the yaw angle.
    """
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class ArucoSteering(Node):
    def __init__(self):
        super().__init__('aruco_steering')
        self.sub = self.create_subscription(PoseArray, '/aruco_poses', self.pose_cb, 10)
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.get_logger().info('aruco_steering started')

    def pose_cb(self, msg):
        if not msg.poses:
            return
        yaw = yaw_from_quaternion(msg.poses[0].orientation)
        twist = Twist()
        twist.angular.z = yaw
        self.pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ArucoSteering())
    rclpy.shutdown()
