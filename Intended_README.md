# Lab1 — Intended Approach

## Goal

Use only built-in ROS 2 and turtlesim packages via CLI to produce the required nodes and topics.

## Architecture

```
/teleop_turtle  →  /tartu/cmd_vel  →  /tartu_demo
                                           ↓
                                turtle1/rotate_absolute/_action/feedback
                                turtle1/rotate_absolute/_action/status
```

- `/teleop_turtle` — `turtle_teleop_key`, default node name
- `/tartu_demo` — `turtlesim_node`, renamed via `--remap __node:=tartu_demo`

---

## Step 1 — Launch tartu_demo (turtlesim, renamed)

```bash
source /opt/ros/humble/setup.bash
ros2 run turtlesim turtlesim_node --ros-args --remap __node:=tartu_demo
```

## Step 2 — Launch teleop_turtle

In a new terminal:

```bash
source /opt/ros/humble/setup.bash
ros2 run turtlesim turtle_teleop_key --ros-args --remap /turtle1/cmd_vel:=/tartu/cmd_vel
```

---

## Step 3 — Verify

```bash
ros2 node list
ros2 topic list --include-hidden-topics
ros2 action list
rqt_graph
```
