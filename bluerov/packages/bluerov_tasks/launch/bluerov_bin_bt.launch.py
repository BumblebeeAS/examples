"""BlueROV2 bin-task."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")

    bt = Node(
        package="bluerov_tasks",
        executable="bluerov_bin_mission_tree.py",
        name="bluerov_bin_mission_tree",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        [DeclareLaunchArgument("use_sim_time", default_value="true"), bt]
    )
