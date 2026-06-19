from bb_perception_msgs.msg import ClusterPoseResultArray

# Minimum poses in the top cluster before we exit the search and goto it.
MIN_CLUSTER_POSES_TO_EXIT = 8


def cluster_decision(msg: ClusterPoseResultArray) -> str:
    """Return "exit" while the top cluster is strong enough, else "continue"."""
    if not msg.results:
        return "continue"

    if msg.results[0].num_cluster_poses >= MIN_CLUSTER_POSES_TO_EXIT:
        return "exit"

    return "continue"
