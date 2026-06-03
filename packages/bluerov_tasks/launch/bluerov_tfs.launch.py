"""Static task TFs for BlueROV2 bin/torpedo missions.

Spawns:
- `frames/static_tfs_node.py` loaded with BlueROV-specific YAMLs from
  `bluerov_tasks/config/`.

"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "single_tfs_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("bluerov_tasks"),
                        "config",
                        "bluerov_single_tfs.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument(
                "grouped_tfs_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("bluerov_tasks"),
                        "config",
                        "bluerov_grouped_tfs.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument("default_suffix", default_value="view"),
            Node(
                package="frames",
                executable="static_tfs_node.py",
                name="static_tfs_node",
                output="screen",
                parameters=[
                    {
                        "single_tfs_file": LaunchConfiguration("single_tfs_file"),
                        "grouped_tfs_file": LaunchConfiguration("grouped_tfs_file"),
                        "default_suffix": LaunchConfiguration("default_suffix"),
                        "use_sim_time": True,
                    }
                ],
            ),
        ]
    )
