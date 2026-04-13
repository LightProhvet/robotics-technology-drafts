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
