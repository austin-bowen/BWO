from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        Node(
            package='realsense_ros2_camera',
            node_executable='realsense_ros2_camera',
        ),
        Node(
            package='bwo',
            node_executable='object_detect',
        ),
        Node(
            package='bwo',
            node_executable='follow',
        ),
        Node(
            package='bwo',
            node_executable='neck',
        ),
        Node(
            package='bwo',
            node_executable='drive_motors',
        ),
    ])

