from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package = LaunchConfiguration("offboard_package")
    executable = LaunchConfiguration("offboard_executable")

    return LaunchDescription(
        [
            DeclareLaunchArgument("offboard_package", default_value="uav2_offboard"),
            DeclareLaunchArgument("offboard_executable", default_value="px4_offboard_demo"),
            Node(
                package=package,
                executable=executable,
                name="px4_offboard_demo",
                output="screen",
                parameters=[
                    {"use_sim_time": True},
                    {"vehicle_namespace": "x500"},
                    {"vehicle_id": 2},
                ],
            ),
        ]
    )
