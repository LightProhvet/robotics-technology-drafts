# robotics-technology-drafts

## Lab3

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
1) Build the robots as per tutorials
2) Parameterize the four-wheeler created in Learning Nugget 1 as shown
3) Create a new ROS package rt_humanoid_description and in this package describe a humanoid robot.
 It must be possible to move the legs and arms.
 Create launch file for demo. 
---

### Step 0 — download and analyse the boilerplate package

If you haven't already, create the src folder
```bash
cd src
git clone https://github.com/unitartu-edu/fourwheeler_description.git
sudo rm fourwheeler_description/.git/ -R
cd ..
```
I'm not sure what to bring out from the rviz config, launch or urdf. They are in their respective folders so finding them is easy.
The syntax is pretty straightforward as well. 

e.g:
rviz config: 2 panels, 3 displays in the manager, 2 views - 1 is a placeholder for saved views i think. and Window geometry describes macro - Displays: collapsed: false - ensure displays are expanded not collapsed. 

launch: adds 3 actions for nodes - 1) robot state 2) joint state 3) rviz

urdf: 1 robot object with 1 link with only visuals (geometry and material)

"Let us now build our colcon workspace and launch the one launch-file from the package." 
(I had problem some CMAKE problem colcon build failed so i basically had to build twice, but it's irrelevant)
```bash
sudo rosdep init
rosdep update
rosdep install --from-paths src -y --ignore-src
colcon build
sourcenow
ros2 launch fourwheeler_description displ	ay.launch.py
```
---

### Step 1 — URDF tutorial video

```bash
sudo apt-get install ros-humble-urdf-tutorial
sudo apt install ros-humble-joint-state-publisher-gui
```
And after defining your URDF i launched with: (NB either have your own launch file or give your own correct path)
```bash
ros2 launch urdf_tutorial display.launch.py model:=/home/lightprohvet/Schoolplace/Robotitehnoloogia/src/fourwheeler_description/urdf/fourwheeler.urdf
```

---

### Step 2 — XACRO tutorial video
ASK: why no mention of setting base link origin?
I changed the launch file to use the .xacro file. 
obviously After copy i made the changes, but the general idea is

```bash
cp fourwheeler.urdf fourwheeler.urdf.xacro
ros2 launch fourwheeler_description display.launch.py
```
I originally used this, like before
```bash
cp fourwheeler.urdf fourwheeler.urdf.xacro
ros2 launch urdf_tutorial display.launch.py model:=/home/lightprohvet/Schoolplace/Robotitehnoloogia/src/fourwheeler_description/urdf/fourwheeler.urdf.xacro
```
---

### Step 3 — create humanoid package
I mean i don't think there are any valid scenarios where i would just design a humanoid robot from scratch - unless i do it part by part, in which case - i'm still not doing a humanoid robot from scratch - the humanoid comes from assembling the existing parts. If i just need to have a humanoid in my simulation i'd use an existing one, e.g https://github.com/robot-descriptions/awesome-robot-descriptions/tree/main

So i will do the simplest one, as shown in one of the images: sphere head, sylinder body, 4 rectangular limbs.
The limbs need 2 links so i could rotate them in 2 dimensions as one joint can only have 1 parent. I demonstrate 2 ways to avoid collision, but neither of them work 100% in rviz. Collisions don't work in rviz at all, and limits are not strict enough, as i did not bother with the math. 

NB: the dependencies are not real dependencies. I just based them off the fourwheeler - ensures they exist during demo

```bash
cd src
ros2 pkg create --build-type ament_cmake rt_humanoid_description --dependencies joint_state_publisher_gui robot_state_publisher
mkdir -p rt_humanoid_description/urdf rt_humanoid_description/launch rt_humanoid_description/config
cd ..
```

Update `CMakeLists.txt` to install the directories — replace the `find_package(joint_state_publisher_gui REQUIRED)
find_package(robot_state_publisher REQUIRED)` block with:
```
install(
  DIRECTORY urdf launch config
  DESTINATION share/${PROJECT_NAME}
)
```


Create the URDF:

```bash
cat > src/rt_humanoid_description/urdf/humanoid.urdf.xacro << 'EOF'
<?xml version="1.0"?>
<robot name="humanoid" xmlns:xacro="http://www.ros.org/wiki/xacro">

    <xacro:property name="body_length" value="0.430"/>
    <xacro:property name="body_radius" value="0.1"/>
    <xacro:property name="head_radius" value="0.15"/>
    <xacro:property name="arm_length" value="0.25"/>
    <xacro:property name="arm_radius" value="0.05"/>
    <xacro:property name="leg_length" value="0.3"/>
    <xacro:property name="leg_radius" value="0.08"/>

    <link name="base_link">
        <visual>
            <geometry>
                <cylinder length="${body_length}" radius="${body_radius}"/>
            </geometry>
            <material name="red">
                <color rgba="0.8 0 0 1"/>
            </material>
        </visual>
        <collision>
            <geometry>
                <cylinder length="${body_length}" radius="${body_radius}"/>
            </geometry>
        </collision>
    </link>

    <link name="head">
        <visual>
            <geometry>
                <sphere radius="${head_radius}"/>
            </geometry>
            <material name="blue">
                <color rgba="0 0 0.8 1"/>
            </material>
        </visual>
    </link>
    <joint name="head_swivel" type="fixed">
        <parent link="base_link"/>
        <child link="head"/>
        <origin xyz="0 0 ${body_length/2 + head_radius}"/>
    </joint>

    <xacro:macro name="arm" params="arm_name reflect_y">
        <link name="${arm_name}_pivot"/>
        <joint name="${arm_name}_side_to_side" type="revolute">
            <parent link="base_link"/>
            <child link="${arm_name}_pivot"/>
            <origin xyz="0 ${reflect_y * (body_radius + arm_radius)} ${body_length/2 - arm_radius}"/>
            <axis xyz="1 0 0"/>
            <limit lower="0" upper="3.14" effort="10" velocity="1"/>
        </joint>
        <link name="${arm_name}">
            <visual>
                <origin xyz="0 0 -${arm_length/2}"/>
                <geometry>
                    <cylinder length="${arm_length}" radius="${arm_radius}"/>
                </geometry>
                <material name="blue">
                    <color rgba="0 0 0.8 1"/>
                </material>
            </visual>
        </link>
        <joint name="${arm_name}_front_to_back" type="revolute">
            <parent link="${arm_name}_pivot"/>
            <child link="${arm_name}"/>
            <origin xyz="0 0 0"/>
            <axis xyz="0 1 0"/>
            <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
        </joint>
    </xacro:macro>

    <xacro:macro name="leg" params="leg_name reflect_y">
        <link name="${leg_name}_pivot"/>
        <joint name="${leg_name}_side_to_side" type="continuous">
            <parent link="base_link"/>
            <child link="${leg_name}_pivot"/>
            <origin xyz="0 ${reflect_y * (body_radius/2 + leg_radius)} -${body_length/2}"/>
            <axis xyz="1 0 0"/>
        </joint>
        <link name="${leg_name}">
            <visual>
                <origin xyz="0 0 -${leg_length/2}"/>
                <geometry>
                    <cylinder length="${leg_length}" radius="${leg_radius}"/>
                </geometry>
                <material name="blue">
                    <color rgba="0 0 0.8 1"/>
                </material>
            </visual>
            <collision>
                <origin xyz="0 0 -${leg_length/2}"/>
                <geometry>
                    <cylinder length="${leg_length}" radius="${leg_radius}"/>
                </geometry>
            </collision>
        </link>
        <joint name="${leg_name}_front_to_back" type="continuous">
            <parent link="${leg_name}_pivot"/>
            <child link="${leg_name}"/>
            <origin xyz="0 0 0"/>
            <axis xyz="0 1 0"/>
        </joint>
    </xacro:macro>

    <xacro:arm arm_name="left_arm" reflect_y="1"/>
    <xacro:arm arm_name="right_arm" reflect_y="-1"/>
    <xacro:leg leg_name="left_leg" reflect_y="1"/>
    <xacro:leg leg_name="right_leg" reflect_y="-1"/>

</robot>
EOF
```

Copy the RViz config from fourwheeler_description (same 3-node setup):

```bash
cp src/fourwheeler_description/config/fw.rviz src/rt_humanoid_description/config/fw.rviz
```

Create the launch file:

```bash
cat > src/rt_humanoid_description/launch/display.launch.py << 'EOF'
from ament_index_python.packages import get_package_share_directory
import os
from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    pkg_dir = get_package_share_directory('rt_humanoid_description')
    robot_description = os.path.join(pkg_dir, 'urdf/humanoid.urdf.xacro')

    ld.add_action(DeclareLaunchArgument(name='model', default_value=str(robot_description),
                                        description='Absolute path to robot urdf/xacro file'))
    ld.add_action(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    ))
    ld.add_action(Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui'
    ))
    ld.add_action(Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d' + os.path.join(pkg_dir, 'config', 'fw.rviz')],
    ))
    return ld
EOF
```

Build and launch:

```bash
colcon build --packages-select rt_humanoid_description
sourcenow
ros2 launch rt_humanoid_description display.launch.py
```

