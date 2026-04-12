# robotics-technology-drafts

## Lab2

In the classroom start with 
```bash
gh auth login
```
(git clone if needed) and end with deleteing workspace and this command: 
```bash
gh auth logout
```

### Goal
0) Test an existing package as a demo
1) Create a new ROS package and name it simple_path_planner.

2) Within the simple_path_planner create a ROS node - let us name it trajectory_commander - that publishes geometry_msgs/msg/Twist messages on a topic “cmd_vel”.

3) Within the trajectory_commander node implement functionality to make the robot drive along the following four different paths:
* simple square (side length of at least 1 m)
* non-holonomic square (side length of at least 1 m)
* circle
* shape of 8
4) The solution should be demonstrated in a scenario where the robot’s linear speed is 0.2 m/s.
5) Test and validate your solution using the Robotont’s Simple Simulator package.


Bonus: connect to the robot
---

### Step 0 — I already had this, but init some dependencies and ensure structure

sudo apt install python3-colcon-common-extensions
mkdir -p src

---

### Step 1 — Add submodules

```bash
cd src
git submodule add https://github.com/robotont/robotont_description
git submodule add https://github.com/robotont/robotont_simple_simulator
cd ..
```

---

### Step 2 — use rosdep to solve dependencies and build

```bash
sudo rosdep init
rosdep update
rosdep install --from-paths src -y --ignore-src
```
I also needed setup before building:
```bash
cd src/robotont_description/
git checkout humble-devel cd ../..
```
Then i changed the name of the launch file in the sample simulator:

display_robot_model - display_simulated_robot

(original line was
os.path.join(get_package_share_directory('robot
         -ont_description'), 'launch/display_robot_model.launch.py')
)

```bash
colcon build
```
---

### Step 3 — Test the robotont simulator

```bash
sourcenow
ros2 launch robotont_simple_simulator simple_driver.launch.py
```

---

### Step 4 — Create the package

```bash
cd src
ros2 pkg create --build-type ament_python simple_path_planner \
  --dependencies rclpy geometry_msgs
cd ..
```

---

### Step 5 — Create the `trajectory_commander` node

Each path is represented as a flat sequence of `(linear, angular, duration)` tuples. A 20 Hz timer steps through them one by one, publishing the corresponding `Twist` until the step's duration expires, then advancing to the next. When the sequence ends, a zero `Twist` stops the robot.

The path to run is selected via a ROS parameter at launch time. `_build_sequence` dispatches to a per-path method using `getattr(self, f'_path_{path}')`, so adding a new path only requires adding a new method — no branching needed in the dispatcher.

All paths use `LINEAR_SPEED = 0.2 m/s`:
- **square** — drive straight, turn in place 90°, repeat × 4
- **nh_square** — same, but corners are arc turns (simultaneous linear + angular) instead of point turns. TODO: Ask about non-holonomic shapes - IS it just an engineering thing or why it exists?
- **circle** — constant forward + angular velocity for one full loop
- **eight** — two circles in opposite directions

```bash
cat > src/simple_path_planner/simple_path_planner/trajectory_commander.py << 'EOF'
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
EOF
```

---

### Step 6 — Register the node in `setup.py`

In `src/simple_path_planner/setup.py`, update `entry_points`:

```python
entry_points={
    'console_scripts': [
        'trajectory_commander = simple_path_planner.trajectory_commander:main',
    ],
},
```

---

### Step 7 — Build

```bash
(sourcehumble)
colcon build --packages-select simple_path_planner
sourcenow
```

---

### Step 8 — Run

In Terminal 1, launch the simulator:

```bash
sourcenow
ros2 launch robotont_simple_simulator simple_driver.launch.py
```

In Terminal 2, run the trajectory (replace `square` with `nh_square`, `circle`, or `eight`):

```bash
sourcenow
ros2 run simple_path_planner trajectory_commander --ros-args -p path:=square
```


---

### BONUS
### STEP 1 — correct internet validation?
(source before use)
On the ROBOT!
```bash
ros2 run demo_nodes_cpp talker
```

In your computer:
```bash
ros2 run demo_nodes_cpp listener
```
---

### Step 2 — connect to robot from pc	
use the correct Ip address from checking wifi details on the robot or
$user@$machine_name

I assume the classroom computers have ssh keys set up, if not - ssh-keygen
```bash
ssh peko@robotont-3
tail ~/.bashrc
```

If problems - Explicitly set the middleware with::
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

