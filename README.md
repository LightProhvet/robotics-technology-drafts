# robotics-technology-drafts

## Lab4

In the classroom start with 
```bash
gh auth login
```
(git clone if needed) and end with deleteing workspace and this command: 
```bash
gh auth logout
```

### Goal
1) Setting up camera-based AR tag detection
2) Parameterize the four-wheeler created in Learning Nugget 1 as shown
3) Create a new ROS package rt_humanoid_description and in this package describe a humanoid robot.
 It must be possible to move the legs and arms.
 Create launch file for demo. 
---

### Step 1 — confirm you can use camera

I used humble so: export ROS_DISTRO=humble
```bash
sudo apt install ros-$ROS_DISTRO-usb-cam ros-$ROS_DISTRO-image-pipeline ros-$ROS_DISTRO-tf-transformations
ros2 run usb_cam usb_cam_node_exe
```
and in another terminal rviz2

I was missing .ros/camera_info/default_cam.yaml, but ignored it.
also the pydantic downgrading fix is pip install pydantic==1.10.9

I prefer adding elements by topic directly, but whatever
---

### Step 2 — Calibrate the camera
I needed to downgrade numpy:
```bash
pip install "numpy<2"
```
running ros topic list showed i should calibrate with, but since i don't have the equipemtn i have to change the image:= and camera:= anyway

also make sure the size 8x6 and square 0.025 are correct
```bash
ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.025 --ros-args -r image:=/image_raw -p camera:=/
```
press buttons for calibrate, save, and commit
Confirm that a new yaml-file has been created in the ~/.ros/camera_info folder
---

### Step 3 — 3rd party packages

(I needed to checkout into humble branch as well)
```bash
cp src
git submodule add https://github.com/JMU-ROBOTICS-VIVA/ros2_aruco
cd ..
pip install --upgrade transforms3d
pip install opencv-contrib-python==4.6.0.66
colcon build
sourcenow
```
My remapping was: 
```bash
ros2 run ros2_aruco aruco_node --ros-args -r /camera/image_raw:=/image_raw -r /camera/camera_info:=/camera_info 
```
Visualize (after sourcing ros and *now) with one of 
```bash
ros2 topic echo /aruco_markers
ros2 topic echo /aruco_poses
rviz2
```
configure RViz:
- Set **Fixed Frame** to `default_cam`
- Add **/aruco_poses** topic display with **PoseArray** type, change Shape to **Axes**
- Add **/image_raw** topic with Image or Camera display

For the next step generate the config file by
File -> Save Config
---

### Step 4 — create one-terminal-usage package

```bash
cd src
ros2 pkg create --build-type ament_python ar_tracking
mkdir -p ar_tracking/config ar_tracking/launch
cd ..
```
move the previously saved config under the name aruco.rviz into the config folder

Update `setup.py` to install the `launch` and `config` directories — add to `data_files`:
(NB: since i will need this in the future, i'm adding steer.launch already as well.)
```python
import os

data_files=[
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), ['launch/display.launch.py', 'launch/steer.launch.py']),
    (os.path.join('share', package_name, 'config'), ['config/aruco.rviz']),
],
```


Create the launch file:

```bash
cat > src/ar_tracking/launch/display.launch.py << 'EOF'
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
    ))

    ld.add_action(Node(
        package='ros2_aruco',
        executable='aruco_node',
        remappings=[
            ('/camera/image_raw', '/image_raw'),
            ('/camera/camera_info', '/camera_info'),
        ]
    ))

    ld.add_action(Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d' + os.path.join(get_package_share_directory('ar_tracking'), 'config', 'aruco.rviz')]
    ))

    return ld
EOF
```

Build and launch:

```bash
colcon build --packages-select ar_tracking
sourcenow
ros2 launch ar_tracking display.launch.py
```

---


### Step 5 — Learning milestone: AR tag steering node
https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles - see section "Quaternion to angles (in ZYX sequence) conversion"
I don't know around which axis the rotation is. I'm using Z in the example, but if it doesn't work i just need to swtich the formula between these:
Yaw (around Z) — equates to R[1,0] R[0,0]:
atan2( 2*(w*z + x*y),  1 - 2*(y² + z²) )
                 
Pitch (around Y) - equates to -R[2,0]:
asin(2*(w*y -x*z))

using asin is simpler, but i think it adds limits to turning - if something is weird use the more complicated version given with others.                                                                                
                                              
Roll (around X) - equates to R[2,1] R[2,2]:
atan2( 2*(w*x + y*z),  1 - 2*(x² + y²) )

```bash
cat > src/ar_tracking/ar_tracking/aruco_steering_node.py << 'EOF'
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
EOF
```

Register in `src/ar_tracking/setup.py`:

```python
entry_points={
    'console_scripts': [
        'aruco_steering_node = ar_tracking.aruco_steering_node:main',
    ],
},
```

---

### Step 7 — One-command launch with steer.launch.py

```bash
cat > src/ar_tracking/launch/steer.launch.py << 'EOF'
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
    ))

    ld.add_action(Node(
        package='ros2_aruco',
        executable='aruco_node',
        remappings=[
            ('/camera/image_raw', '/image_raw'),
            ('/camera/camera_info', '/camera_info'),
        ]
    ))

    ld.add_action(Node(
        package='ar_tracking',
        executable='aruco_steering_node',
    ))

    # Either the full launch or just simulation node
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('robotont_simple_simulator'),
                         'launch/simple_driver.launch.py')
        )
    ))
    """
    ld.add_action(Node(
        package='robotont_simple_simulator',
        executable='simple_driver_node',
    ))
    """

    ld.add_action(Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d' + os.path.join(get_package_share_directory('ar_tracking'), 'config', 'aruco.rviz')]
    ))

    return ld
EOF
```

Build and launch:

```bash
colcon build
sourcenow
ros2 launch ar_tracking steer.launch.py
```

Or run separately if needed:

```bash
# Terminal 1 — AR tracking pipeline
ros2 launch ar_tracking display.launch.py

# Terminal 2 — simulator
ros2 launch robotont_simple_simulator simple_driver.launch.py

# Terminal 3 — steering node
ros2 run ar_tracking aruco_steering_node
```

