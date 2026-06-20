"""BlueROV2 controls + TFs + actuators stack."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    bluerov_share = get_package_share_directory("bluerov_tasks")

    tfs_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bluerov_share, "launch", "bluerov_tfs.launch.py")
        )
    )

    locomotion_server = Node(
        package="bluerov_tasks",
        executable="locomotion_action_server.py",
        name="locomotion_action_server",
        output="screen",
    )

    convert_to_controls_pose = Node(
        package="frames",
        executable="convert_to_controls_pose.py",
        name="convert_to_controls_pose",
        output="screen",
        parameters=[
            {
                "controls_frame": "map",
                "base_frame": "base_link",
                "odom_topic": "/mavros/odometry/out",
            }
        ],
        remappings=[("convert_to_controls_pose", "/bluerov/convert_to_controls_pose")],
    )

    actuators = Node(
        package="bluerov_tasks",
        executable="actuators_sim.py",
        name="actuators_sim",
        output="screen",
    )

    return LaunchDescription(
        [
            tfs_launch,
            locomotion_server,
            convert_to_controls_pose,
            actuators,
        ]
    )
