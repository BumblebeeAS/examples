"""BlueROV2 torpedo-task."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    bt = Node(
        package="bluerov_tasks",
        executable="bluerov_torpedo_mission_tree.py",
        name="bluerov_torpedo_mission_tree",
        output="screen",
    )

    return LaunchDescription([bt])
