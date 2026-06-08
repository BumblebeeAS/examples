"""Spike-search decision function for the BlueROV pose-based clustering.

The cluster_poses service node publishes a ``ClusterPoseResultArray`` on
``/bluerov/cluster_pose_results`` every ``cluster_interval`` seconds while a
spike search is running. ``cluster_decision`` inspects each snapshot and tells
the shared ``create_new_search_bot_layered_square_root`` builder what to do:

  - "continue" — no usable cluster yet; keep flying the search pattern.
  - "exit"     — a strong-enough cluster exists; stop searching and goto it.

We do NOT use the "relocate" path here (no relocate_frame is wired in), so
this only ever returns "continue" or "exit".
"""

from bb_perception_msgs.msg import ClusterPoseResultArray

# Minimum number of poses in the top cluster before we trust it enough to
# stop searching and fly to the clustered pose. Tunable.
MIN_CLUSTER_POSES_TO_EXIT = 8

def cluster_decision(msg: ClusterPoseResultArray) -> str:
    """Decide search action from a ClusterPoseResultArray snapshot.

    Returns "exit" on every tick while the top cluster is strong enough, so the
    search exits as soon as the parallel sees it. We deliberately do NOT dedupe
    on header stamp: with the search parallel re-ticking faster than the ~1 Hz
    result publish, a one-shot "exit" gets overwritten by "continue" on the next
    same-stamp tick before the search can act on it — the cluster never wins.
    Re-emitting "exit" is idempotent (the search exits once), so consistency
    beats deduping here.
    """
    if not msg.results:
        return "continue"

    if msg.results[0].num_cluster_poses >= MIN_CLUSTER_POSES_TO_EXIT:
        return "exit"

    return "continue"
