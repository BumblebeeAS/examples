"""BlueROV2 bin-task mission launch — full vision + controls + BT stack.

Composes everything the bin behaviour tree needs on top of an already-running
`bluerov_sim.launch.py`:

- frames service (`convert_to_controls_pose`) + locomotion action server +
  static TFs + choice server + actuators stub (Stage 2)
- `bb_filters` cluster_tf action and service servers under /bluerov
- vision_pipeline YOLO + image_matching + pose_estimator nodes under
  /bluerov/bin namespace
- the BT mission tree entry point

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
        bluerov_share, "config", "vision_pipeline", "bin.yaml"
    )

    # Stage 2 — controls + TFs + actuators
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

    # cluster_tf — action server + synchronous service server, both under /bluerov
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

    # Vision pipeline — namespaced /bluerov/bin
    vision = GroupAction([
        PushRosNamespace("/bluerov/bin"),
        LifecycleNode(
            package="yolo_ros_trt",
            executable="yolo_node",
            name="bin_yolo_node",
            namespace="",
            parameters=[vision_cfg],
        ),
        Node(
            package="pose_estimator",
            executable="bin_pose_estimator_node",
            name="bin_pose_estimator_node",
            parameters=[vision_cfg],
        ),
        Node(
            package="vision_pipeline",
            executable="lifecycle_manager_node",
            name="lifecycle_manager_node",
            parameters=[vision_cfg],
        ),
        Node(
            package="image_processing",
            executable="image_brighten_node",
            name="image_brighten_node",
            parameters=[vision_cfg],
        ),
        Node(
            package="image_matching",
            executable="simple_matcher_node",
            name="simple_matcher_node",
            parameters=[vision_cfg],
        ),
        Node(
            package="pose_estimator",
            executable="points_pose_estimator_node",
            name="points_pose_estimator_node",
            parameters=[vision_cfg],
        ),
    ])

    bt = Node(
        package="bluerov_sim",
        executable="bluerov_bin_mission_tree.py",
        name="bluerov_bin_mission_tree",
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
