"""BlueROV-local shared tree helpers.

Small BlueROV-specific subtrees that don't belong in mission_planner_release:
  - choice.py — fish/shark choice selector
  - cluster_decision.py — decision_func over ClusterPoseResultArray for the
    pose-based spike search

Generic tree infrastructure (BumbleTree, hooks, pose_utils, cluster_goto,
blackboard, tf_checker, goto, search) is imported from mission_planner_release.
"""
