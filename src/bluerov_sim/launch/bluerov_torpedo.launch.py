"""BlueROV2 torpedo-task mission launch — full vision + controls + BT stack.

Same shape as `bluerov_bin.launch.py` but for the torpedo mission:
- front-camera YOLO + tracking + torpedo + hole pose estimators
- no image_brighten/simple_matcher (the AUV4 torpedo launch leaves them
  commented out — same here)
- BT entry runs `bluerov_torpedo_mission_tree.py`

Prereq: launch `bluerov_sim.launch.py world_name:=robosub_2025_pool` in
another terminal first.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, PushRosNamespace, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import LifecycleNode, Node


def generate_launch_description() -> LaunchDescription:
    bluerov_share = get_package_share_directory("bluerov_sim")
    vision_cfg = os.path.join(
        bluerov_share, "config", "vision_pipeline", "torpedo.yaml"
    )

    tfs_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bluerov_share, "launch", "bluerov_tfs.launch.py")
        )
    )
    locomotion_server = Node(
        package="bluerov_sim",
        executable="locomotion_action_server.py",
        name="locomotion_action_server",
        output="screen",
    )
    convert_to_controls_pose = Node(
        package="frames",
        executable="convert_to_controls_pose.py",
        name="convert_to_controls_pose",
        output="screen",
        parameters=[{
            "controls_frame": "map",
            "base_frame": "base_link",
            "odom_topic": "/mavros/odometry/out",
        }],
        remappings=[("convert_to_controls_pose", "/bluerov/convert_to_controls_pose")],
    )
    actuators = Node(
        package="bluerov_sim",
        executable="actuators_stub.py",
        name="actuators_stub",
        output="screen",
    )

    cluster_servers = GroupAction([
        PushRosNamespace("bluerov"),
        Node(
            package="bb_filters",
            executable="cluster_tf_action_server.py",
            name="cluster_tf_action_server",
            output="screen",
        ),
        Node(
            package="bb_filters",
            executable="cluster_tf_service_server.py",
            name="cluster_tf_service_server",
            output="screen",
        ),
    ])

    vision = GroupAction([
        PushRosNamespace("/bluerov/torpedo"),
        LifecycleNode(
            package="yolo_ros_trt",
            executable="yolo_node",
            name="torpedo_yolo_node",
            namespace="",
            parameters=[vision_cfg],
        ),
        LifecycleNode(
            package="yolo_ros_trt",
            executable="tracking_node",
            name="torpedo_hole_yolo_node",
            namespace="",
            parameters=[vision_cfg],
        ),
        Node(
            package="pose_estimator",
            executable="torpedo_pose_estimator_node",
            name="torpedo_pose_estimator_node",
            parameters=[vision_cfg],
        ),
        Node(
            package="pose_estimator",
            executable="red_circle_pose_estimator_node",
            name="torpedo_hole_pose_estimator_node",
            parameters=[vision_cfg],
        ),
        Node(
            package="vision_pipeline",
            executable="lifecycle_manager_node",
            name="lifecycle_manager_node",
            parameters=[vision_cfg],
        ),
    ])

    bt = Node(
        package="bluerov_sim",
        executable="bluerov_torpedo_mission_tree.py",
        name="bluerov_torpedo_mission_tree",
        output="screen",
    )

    return LaunchDescription([
        tfs_launch,
        locomotion_server,
        convert_to_controls_pose,
        actuators,
        cluster_servers,
        vision,
        bt,
    ])
