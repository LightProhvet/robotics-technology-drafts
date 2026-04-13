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
