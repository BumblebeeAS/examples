"""BlueROV2 bin vision."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node, PushRosNamespace


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    sim_time_param = {"use_sim_time": use_sim_time}

    cfg = os.path.join(
        get_package_share_directory("bluerov_tasks"),
        "config",
        "vision_pipeline",
        "bin.yaml",
    )

    vision = GroupAction(
        [
            PushRosNamespace("/bluerov/bin"),
            LifecycleNode(
                package="yolo_ros_trt",
                executable="yolo_node",
                name="bin_yolo_node",
                namespace="",
                parameters=[cfg, sim_time_param],
            ),
            Node(
                package="pose_estimator",
                executable="bin_pose_estimator_node",
                name="bin_pose_estimator_node",
                parameters=[cfg, sim_time_param],
            ),
            Node(
                package="vision_pipeline",
                executable="lifecycle_manager_node",
                name="lifecycle_manager_node",
                parameters=[cfg, sim_time_param],
            ),
            Node(
                package="image_processing",
                executable="image_brighten_node",
                name="image_brighten_node",
                parameters=[cfg, sim_time_param],
            ),
            Node(
                package="image_matching",
                executable="simple_matcher_node",
                name="simple_matcher_node",
                parameters=[cfg, sim_time_param],
            ),
            Node(
                package="pose_estimator",
                executable="points_pose_estimator_node",
                name="points_pose_estimator_node",
                parameters=[cfg, sim_time_param],
            ),
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            vision,
        ]
    )
