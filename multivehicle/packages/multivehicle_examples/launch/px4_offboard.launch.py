import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")

    offboard_launch = os.path.join(
        get_package_share_directory("uav2_offboard"), "launch", "launch.py"
    )
    default_params = os.path.join(
        get_package_share_directory("multivehicle_examples"),
        "config",
        "uav2_offboard_x500.yaml",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=default_params),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(offboard_launch),
                launch_arguments={"params_file": params_file}.items(),
            ),
            Node(
                package="mission_planner_2",
                executable="uav2_offboard_demo_main.py",
                name="uav2_offboard_demo",
                output="screen",
                parameters=[{"use_sim_time": True}],
            ),
        ]
    )
