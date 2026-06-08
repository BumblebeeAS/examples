"""BlueROV2 cluster_poses action + service servers (under /bluerov).

Shared by every task BT (bin, torpedo, …). Split out from the controls
launch so the cluster layer can be restarted / log-isolated on its own pane.

The cluster_poses servers subscribe to PoseStamped TOPICS published by the
per-task pose estimators (e.g. /bluerov/bin/bin/yolo/pose,
/bluerov/torpedo/torpedo/yolo/pose) synchronised against /mavros/odometry/out,
DBSCAN-cluster them, and broadcast the clustered result as a TF child frame.

  - cluster_poses_action_node.py  — async ClusterPosesAction goal handler
    (resolves to /bluerov/cluster_poses)
  - cluster_poses_service_node.py — ClusterPosesSrv handler
    (resolves to /bluerov/cluster_poses_srv); when cluster_interval > 0 it
    publishes a ClusterPoseResultArray on the default `cluster_pose_results`
    topic (→ /bluerov/cluster_pose_results) consumed by the spike search.

Task BTs (and bluerov_tasks/torpedo/move_and_shoot_seq.py) submit cluster
goals/requests built via mission_planner_release pose_utils/detection_utils.
"""

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
