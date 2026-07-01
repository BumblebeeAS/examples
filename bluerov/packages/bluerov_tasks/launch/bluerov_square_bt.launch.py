from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch the BlueROV2 square-mission behaviour tree.

    Starts four nodes:
      1. locomotion_action_server — bb_controls_msgs/Locomotion server on /bluerov/controls
      2. convert_to_controls_pose — frames package service for base_link→map conversion
      3. bluerov_square_mission_tree — py_trees behaviour tree (mission driver)

    Requires the simulation to already be running:
        ros2 launch bluerov_sim bluerov_sim.launch.py
    """
    use_sim_time = LaunchConfiguration("use_sim_time")

    action_server_node = Node(
        package="bluerov_tasks",
        executable="locomotion_action_server.py",
        name="locomotion_action_server",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    convert_to_controls_pose_node = Node(
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
        remappings=[
            ("convert_to_controls_pose", "/bluerov/convert_to_controls_pose"),
        ],
    )

    mission_tree_node = Node(
        package="bluerov_tasks",
        executable="bluerov_square_mission_tree.py",
        name="bluerov_square_mission_tree",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    shutdown_when_mission_finishes = RegisterEventHandler(
        OnProcessExit(
            target_action=mission_tree_node,
            on_exit=[
                EmitEvent(
                    event=Shutdown(
                        reason="Square mission finished",
                    )
                )
            ],
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            action_server_node,
            convert_to_controls_pose_node,
            mission_tree_node,
            shutdown_when_mission_finishes,
        ]
    )
