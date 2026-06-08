"""BlueROV2 torpedo-task: perception pipeline under /bluerov/torpedo.

  - torpedo_yolo_node (lifecycle) — bin/panel YOLO detector
  - torpedo_pose_estimator_node — publishes the `torpedo/yolo` PoseStamped
    topic (PosePubNode; cluster_poses subscribes to it; required for the BT
    to advance past the search leg)
  - lifecycle_manager_node — drives the YOLO lifecycle node through
    configure -> activate when the BT calls /bluerov/torpedo/manage_nodes
  - simple_matcher_node — image_matching package; exposes the
    /bluerov/torpedo/image_matching/{toggle_template,point_correspondences}
    service + topic that the BT uses to confirm which torpedo template
    (Task04_Tagging_01.png vs _02.png) is in view
  - torpedo_points_pose_estimator_node — consumes simple_matcher's
    point_correspondences and publishes the Task04_Tagging_<NN>_optical
    PoseStamped topic that the BT's cluster_poses action subscribes to.
    Without this node the cluster_poses action server has no poses to
    cluster and the BT stalls in the search leg.

No BT, no locomotion, no actuators — split out per pane so vision can be
restarted independently of the BT and cluster panes.

After launch (the BT does this automatically, but for manual smoke tests):

  ros2 service call /bluerov/torpedo/manage_nodes \
      lifecycle_msgs/srv/ChangeState "{transition: {id: 1}}"   # configure
  ros2 service call /bluerov/torpedo/manage_nodes \
      lifecycle_msgs/srv/ChangeState "{transition: {id: 3}}"   # activate

Verify:

  ros2 topic hz /bluerov/torpedo/torpedo/yolo/detections
  ros2 topic list | grep image_matching
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import LifecycleNode, Node, PushRosNamespace


def generate_launch_description() -> LaunchDescription:
    cfg = os.path.join(
        get_package_share_directory("bluerov_tasks"),
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
        Node(
            package="pose_estimator",
            executable="torpedo_pose_estimator_node",
            name="torpedo_pose_estimator_node",
            parameters=[cfg],
        ),
        Node(
            package="vision_pipeline",
            executable="lifecycle_manager_node",
            name="lifecycle_manager_node",
            parameters=[cfg],
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
            parameters=[cfg],
        ),
        # Subscribes to image_matching/point_correspondences, runs PnP, and
        # publishes a PoseStamped topic for object_frame_id (set by
        # simple_matcher to "Task04_Tagging_<NN>_optical"). cluster_poses
        # subscribes to that topic to build the clustered torpedo_<NN> frame.
        Node(
            package="pose_estimator",
            executable="points_pose_estimator_node",
            name="torpedo_points_pose_estimator_node",
            parameters=[cfg],
        ),
    ])

    return LaunchDescription([vision])
