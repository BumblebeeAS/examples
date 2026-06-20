"""BlueROV2 controls + TFs + actuators stack."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    bluerov_share = get_package_share_directory("bluerov_tasks")
    use_sim_time = LaunchConfiguration("use_sim_time")

    tfs_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bluerov_share, "launch", "bluerov_tfs.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    locomotion_server = Node(
        package="bluerov_tasks",
        executable="locomotion_action_server.py",
        name="locomotion_action_server",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
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
                "use_sim_time": use_sim_time,
            }
        ],
        remappings=[("convert_to_controls_pose", "/bluerov/convert_to_controls_pose")],
    )

    actuators = Node(
        package="bluerov_tasks",
        executable="actuators_sim.py",
        name="actuators_sim",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            tfs_launch,
            locomotion_server,
            convert_to_controls_pose,
            actuators,
        ]
    )
