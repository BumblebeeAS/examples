"""BlueROV2 cluster_poses action + service servers."""

from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description() -> LaunchDescription:
    sim_time_param = {"use_sim_time": True}

    cluster_servers = GroupAction(
        [
            PushRosNamespace("bluerov"),
            Node(
                package="bb_filters",
                executable="cluster_poses_action_node.py",
                name="cluster_poses_action_node",
                output="screen",
                parameters=[sim_time_param],
            ),
            Node(
                package="bb_filters",
                executable="cluster_poses_service_node.py",
                name="cluster_poses_service_node",
                output="screen",
                parameters=[sim_time_param],
            ),
        ]
    )

    return LaunchDescription([cluster_servers])
