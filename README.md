# robotics-technology-drafts

## Lab5

In the classroom start with 
```bash
gh auth login
```
(git clone if needed) and end with deleting the workspace and running:
```bash
gh auth logout
```

### Goal

Learn how to use ROS MoveIt for motion planning with a simulated xArm7 robot manipulator. Control the robot via the MoveIt GUI and then programmatically via the MoveGroup C++ Interface, culminating in a Pick and Place task.

---

### Step 1 — Install MoveIt

```bash
sudo apt install ros-humble-moveit
```

---

### Step 2 — Clone and build xarm_ros2

Navigate to your colcon workspace source directory and clone the xArm ROS2 package:

```bash
cd src
git clone https://github.com/xArm-Developer/xarm_ros2.git --recursive -b humble
cd ..
rosdep install --from-paths src --ignore-src --rosdistro humble -y -r
colcon build
sourcenow
```

---

### Step 3 — Visualize xArm7 with gripper

```bash
ros2 launch xarm_description xarm7_rviz_display.launch.py add_gripper:=true
```

Verify the robot model appears in RViz. Shut it down when done.

---

### Step 4 — Launch MoveIt with fake hardware

```bash
ros2 launch xarm_moveit_config xarm7_moveit_fake.launch.py add_gripper:=true
```

In the RVIZ:
- Drag the interactive marker to a new goal pose
- Click **Plan** to compute a trajectory
- Click **Execute** to run it

---

### Step 5 — Hack `_robot_moveit_common2.launch.py`

Open:
```
src/xarm_ros2/xarm_moveit_config/launch/_robot_moveit_common2.launch.py
```

Find:
```python
move_group_node = Node(
    package='moveit_ros_move_group',
    executable='move_group',
    output='screen',
    parameters=[
        moveit_config_dict,
        {'use_sim_time': use_sim_time},
    ],
)
```

Change to:
```python
move_group_node = Node(
    package='moveit_ros_move_group',
    executable='move_group',
    output='screen',
    parameters=[
        moveit_config_dict,
        {'use_sim_time': use_sim_time,
         'publish_robot_description': True,
         'publish_robot_description_semantic': True
        },
    ],
)
```

Rebuild and re-source:

```bash
colcon build --packages-select xarm_moveit_config
source install/setup.bash
```

---

### Step 6 — Test using movegroup_interface_demo

```bash
cd ~/Schoolplace/Robotitehnoloogia/src
git clone https://github.com/ut-ims-robotics/movegroup_interface_demo.git
cd ..
colcon build --packages-select movegroup_interface_demo
sourcnow

ros2 launch xarm_moveit_config xarm7_moveit_fake.launch.py add_gripper:=true
```

In another sourced terminal:

```bash
ros2 run movegroup_interface_demo pose_goal --ros-args -p use_sim_time:=true
```

More examples:
https://github.com/ut-ims-robotics/pool-thesis-2023-moveit2-examples/tree/main/cpp_examples/src

---

### Step 7 — Pick and Place

Create package:

```bash
cd src
ros2 pkg create --build-type ament_cmake xarm_pick_and_place \
    --dependencies rclcpp moveit_ros_planning_interface
cd ..
```

Create the node file:

```bash
cat > src/xarm_pick_and_place/src/pick_and_place.cpp << 'EOF'
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared(
        "pick_and_place",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    // Separate thread for spinning so MoveIt callbacks are processed
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spin_thread([&executor]() { executor.spin(); });

    // Arm group
    moveit::planning_interface::MoveGroupInterface arm(node, "xarm7");
    arm.setMaxVelocityScalingFactor(0.3);
    arm.setMaxAccelerationScalingFactor(0.3);

    // Gripper group
    moveit::planning_interface::MoveGroupInterface gripper(node, "xarm_gripper");

    auto move_arm = [&](double x, double y, double z) {
        geometry_msgs::msg::Pose target;
        target.orientation.w = 1.0;
        target.position.x = x;
        target.position.y = y;
        target.position.z = z;
        arm.setPoseTarget(target);
        arm.move();
    };

    auto open_gripper = [&]() {
        gripper.setNamedTarget("open");
        gripper.move();
    };

    auto close_gripper = [&]() {
        gripper.setNamedTarget("close");
        gripper.move();
    };

    // --- Pick and Place sequence ---

    // 1. Open gripper (redundancy)
    RCLCPP_INFO(node->get_logger(), "Opening gripper");
    open_gripper();
    
    // 2. Move above location A
    RCLCPP_INFO(node->get_logger(), "Moving to location A");
    move_arm(0.3, 0.1, 0.3);

    // 3. Lower to grasp
    move_arm(0.3, 0.1, 0.15);

    // 4. Close gripper (pick)
    RCLCPP_INFO(node->get_logger(), "Closing gripper (pick)");
    close_gripper();

    // 5. Lift (same as point 1) 
    RCLCPP_INFO(node->get_logger(), "Moving to location B");
    move_arm(0.3, 0.1, 0.3);

    // 6. Move above location B
    move_arm(0.3, -0.2, 0.3);

    // 7. Lower to place
    move_arm(0.3, -0.2, 0.15);

    // 8. Open gripper (place)
    RCLCPP_INFO(node->get_logger(), "Opening gripper (place)");
    open_gripper();

    // 9. Retreat
    RCLCPP_INFO(node->get_logger(), "Retreating");
    move_arm(0.3, -0.2, 0.3);

    RCLCPP_INFO(node->get_logger(), "Pick and place complete!");

    executor.cancel();
    spin_thread.join();
    rclcpp::shutdown();
    return 0;
}
EOF
```

Edit `src/xarm_pick_and_place/CMakeLists.txt` to add the executable after the existing `find_package` lines:

```cmake
add_executable(pick_and_place src/pick_and_place.cpp)
ament_target_dependencies(pick_and_place rclcpp moveit_ros_planning_interface)
install(TARGETS pick_and_place DESTINATION lib/${PROJECT_NAME})
```

Build:

```bash
colcon build --packages-select xarm_pick_and_place
sourcenow
```

Run (with MoveIt already running in another terminal):

```bash
ros2 launch xarm_moveit_config xarm7_moveit_fake.launch.py add_gripper:=true
```

```bash
ros2 run xarm_pick_and_place pick_and_place --ros-args -p use_sim_time:=true
```



