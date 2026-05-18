"""BlueROV2 torpedo vision-only smoke test (front camera).

Same shape as bluerov_bin_vision.launch.py — perception nodes only, no BT /
locomotion / actuators.

Two YOLO lifecycle nodes run here (torpedo + torpedo_hole tracker). After
launch, walk both through configure -> activate via the manage_nodes service:

  ros2 service call /bluerov/torpedo/manage_nodes \
      lifecycle_msgs/srv/ChangeState "{transition: {id: 1}}"
  ros2 service call /bluerov/torpedo/manage_nodes \
      lifecycle_msgs/srv/ChangeState "{transition: {id: 3}}"

Verify:

  ros2 topic hz /bluerov/torpedo/torpedo/yolo/detections
  ros2 topic hz /bluerov/torpedo/torpedo/hole/yolo/detections
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import LifecycleNode, Node, PushRosNamespace


def generate_launch_description() -> LaunchDescription:
    cfg = os.path.join(
        get_package_share_directory("bluerov_sim"),
        "config", "vision_pipeline", "torpedo.yaml",
    )

    vision = GroupAction([
        PushRosNamespace("/bluerov/torpedo"),
        LifecycleNode(
            package="yolo_ros_trt",
            executable="yolo_node",
            name="torpedo_yolo_node",
            namespace="",
            parameters=[cfg],
        ),
        LifecycleNode(
            package="yolo_ros_trt",
            executable="tracking_node",
            name="torpedo_hole_yolo_node",
            namespace="",
            parameters=[cfg],
        ),
        Node(
            package="pose_estimator",
            executable="torpedo_pose_estimator_node",
            name="torpedo_pose_estimator_node",
            parameters=[cfg],
        ),
        Node(
            package="pose_estimator",
            executable="red_circle_pose_estimator_node",
            name="torpedo_hole_pose_estimator_node",
            parameters=[cfg],
        ),
        Node(
            package="vision_pipeline",
            executable="lifecycle_manager_node",
            name="lifecycle_manager_node",
            parameters=[cfg],
        ),
    ])

    return LaunchDescription([vision])
