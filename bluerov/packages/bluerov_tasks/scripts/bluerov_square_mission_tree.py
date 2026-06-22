#!/usr/bin/env python3
"""Behaviour tree to move BlueROV2 in a square pattern, for testing."""

import math
import traceback

import py_trees
import py_trees.console as console
import rclpy
from bluerov_tasks.arm_and_set_mode import ArmAndSetMode
from bluerov_tasks.goto import FromConstant as Goto
from bluerov_tasks.node_registry import BlueROVTreeNode
from geometry_msgs.msg import PoseStamped
from mission_planner_2.common.core.bumble_tree import BumbleTree
from mission_planner_2.common.core.hooks import stop_on_success_or_failure

SQUARE_SIDE_M = 2.0
TICK_PERIOD_MS = 100  # 10 Hz — fast poll of action-client futures


def _make_pose(x: float, y: float, z: float = 0.0, yaw_deg: float = 0.0) -> PoseStamped:
    """Build a PoseStamped in base_link (FLU) frame."""
    pose = PoseStamped()
    pose.header.frame_id = "base_link"
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    yaw = math.radians(yaw_deg)
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def create_square_mission_root() -> py_trees.behaviour.Behaviour:
    root = py_trees.composites.Sequence(name="square_mission", memory=True)
    root.add_children(
        [
            ArmAndSetMode(name="arm_and_set_mode"),
            Goto(
                name="leg1_forward",
                pose=_make_pose(x=SQUARE_SIDE_M, y=0.0),
                is_relative_movement=True,
                ignore_depth=True,
            ),
            Goto(
                name="leg2_left",
                pose=_make_pose(x=0.0, y=SQUARE_SIDE_M),
                is_relative_movement=True,
                ignore_depth=True,
            ),
            Goto(
                name="leg3_backward",
                pose=_make_pose(x=-SQUARE_SIDE_M, y=0.0),
                is_relative_movement=True,
                ignore_depth=True,
            ),
            Goto(
                name="leg4_right",
                pose=_make_pose(x=0.0, y=-SQUARE_SIDE_M),
                is_relative_movement=True,
                ignore_depth=True,
            ),
        ]
    )
    return root


def main(args=None) -> None:
    rclpy.init(args=args)

    py_trees.logging.level = py_trees.logging.Level.INFO

    root = create_square_mission_root()
    tree = BumbleTree(root=root)
    node = BlueROVTreeNode("bluerov_square_mission_tree")

    try:
        tree.setup(node=node, timeout=30.0)
    except Exception:
        console.logerror(
            console.red
            + "Failed to set up the behaviour tree:\n"
            + traceback.format_exc()
            + console.reset
        )
        tree.shutdown()
        rclpy.try_shutdown()
        return

    tree.add_post_tick_handler(stop_on_success_or_failure)
    tree.tick_tock(period_ms=TICK_PERIOD_MS)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        console.loginfo(console.yellow + "interrupted" + console.reset)
    except SystemExit:
        console.loginfo(console.yellow + "mission finished — exiting" + console.reset)
    except Exception:
        console.logfatal(console.red + traceback.format_exc() + console.reset)
    finally:
        console.loginfo(console.reset + "cleaning up")
        tree.shutdown()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
