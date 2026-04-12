# robotics-technology-drafts

## Lab1

### Goal
Create a ROS 2 package with two nodes:
- `/teleop_turtle` — publishes velocity commands on `/tartu/cmd_vel`
- `/tartu_demo` — subscribes to `/tartu/cmd_vel` and runs a `RotateAbsolute` action server, producing:
  - `turtle1/rotate_absolute/_action/feedback`
  - `turtle1/rotate_absolute/_action/status`

---

### Step 1 — Create the workspace structure

mkdir -p src

---

### Step 2 — Create the package
Generate `src/tartu_lab1/` with `package.xml`, `setup.py`, and a `tartu_lab1/` Python module directory.


```bash
cd src
source /opt/ros/humble/setup.bash
ros2 pkg create --build-type ament_python tartu_lab1 \
  --dependencies rclpy geometry_msgs turtlesim
cd ..
```

---

### Step 3 — Create the `teleop_turtle` node

```bash
cat > src/tartu_lab1/tartu_lab1/teleop_turtle.py << 'EOF'
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
EOF
```

---

### Step 4 — Create the `tartu_demo` node

```bash
cat > src/tartu_lab1/tartu_lab1/tartu_demo.py << 'EOF'
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
EOF
```

---

### Step 5 — Register the nodes in `setup.py`

In `src/tartu_lab1/setup.py`, update `entry_points`:

```python
entry_points={
    'console_scripts': [
        'teleop_turtle = tartu_lab1.teleop_turtle:main',
        'tartu_demo = tartu_lab1.tartu_demo:main',
    ],
},
```

---

### Step 6 — Build and source

For myself - i was using seteuptools version 82.0.0 before, but for humble a downgrade was required:
```bash
pip install setuptools==58.2.0
```

Then build and source as normal:
```bash
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

---

### Step 7 — Run

In separate terminals:

```bash
sourcehumble
sourcenow
# Terminal 1
ros2 run tartu_lab1 tartu_demo

# Terminal 2
ros2 run tartu_lab1 teleop_turtle
```

Verify topics and visualize the graph:

```bash
ros2 topic list --include-hidden-topics
rqt_graph
```
