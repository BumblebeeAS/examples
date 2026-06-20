"""BlueROV2 torpedo vision."""

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
        "torpedo.yaml",
    )

    vision = GroupAction(
        [
            PushRosNamespace("/bluerov/torpedo"),
            LifecycleNode(
                package="yolo_ros_trt",
                executable="yolo_node",
                name="torpedo_yolo_node",
                namespace="",
                parameters=[cfg, sim_time_param],
            ),
            Node(
                package="pose_estimator",
                executable="torpedo_pose_estimator_node",
                name="torpedo_pose_estimator_node",
                parameters=[cfg, sim_time_param],
            ),
            Node(
                package="vision_pipeline",
                executable="lifecycle_manager_node",
                name="lifecycle_manager_node",
                parameters=[cfg, sim_time_param],
            ),
            # Image matcher: provides the toggle_template service (BT picks
            # Task04_Tagging_01.png vs _02.png) and the point_correspondences
            # publisher (BT compares lengths to decide which template the
            # camera is actually seeing). Hardcodes the publish topic to
            # `image_matching/point_correspondences` inside its namespace so
            # the PushRosNamespace above resolves it to
            # `/bluerov/torpedo/image_matching/point_correspondences`.
            Node(
                package="image_matching",
                executable="simple_matcher_node",
                name="simple_matcher_node",
                parameters=[cfg, sim_time_param],
            ),
            # Subscribes to image_matching/point_correspondences, runs PnP, and
            # publishes a PoseStamped topic for object_frame_id (set by
            # simple_matcher to "Task04_Tagging_<NN>_optical"). cluster_poses
            # subscribes to that topic to build the clustered torpedo_<NN> frame.
            Node(
                package="pose_estimator",
                executable="points_pose_estimator_node",
                name="torpedo_points_pose_estimator_node",
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
