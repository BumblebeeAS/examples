#!/usr/bin/env python3
"""
BlueROV2 goto behaviours — self-contained Locomotion action client built on
mission_planner_release's generic goto_base (the engine was folded in here so the
BlueROV goto no longer depends on the AUV-specific goto).

Routes to /bluerov/controls (BlueROVSharedAction.LOCOMOTION) and converts poses
via /bluerov/convert_to_controls_pose (frames package). The anchor frame defaults
to 'base_link' (FLU body frame).
"""

import os
import sys

_DIR = os.path.dirname(os.path.abspath(__file__))
if _DIR not in sys.path:
    sys.path.insert(0, _DIR)

import math
import time
import uuid
from typing import Any, Callable

import action_msgs.msg as action_msgs
import py_trees
from bb_controls_msgs.action import Locomotion
from bb_planner_msgs.srv import GetPoseToControlsFrame
from geometry_msgs.msg import PoseStamped
from numpy import rad2deg
from rclpy.task import Future
from transforms3d.euler import quat2euler

from bluerov_tasks.node_registry import BlueROVSharedAction, BlueROVSharedService
from mission_planner_release.vehicles.shared.trees.blackboard import (
    convert_to_safe_name,
)
from mission_planner_release.vehicles.shared.trees.goto import goto_base


class FromBlackboard(goto_base.FromBlackboard):
    """
    Locomotion action client for BlueROV2, reading a PoseStamped from the blackboard.

    """

    def __init__(
        self,
        name: str,
        pose_key: str,
        anchor_frame_name: str = "base_link",
        specified_heading: bool = True,
        ignore_depth: bool = False,
        x_threshold: float = 0.2,
        y_threshold: float = 0.2,
        z_threshold: float = 0.2,
        yaw_threshold: float = 5.0,
        stabilize_duration: int = 60,
        generate_feedback_message: Callable[[Any], str] = None,
        wait_for_server_timeout_sec: int = -3,
        wait_for_service_timeout_sec: int = -3,
        is_relative_movement: bool = False,
        depth_override_value: float | None = None,
    ):
        # Bind the BlueROV registries: /bluerov/controls (LOCOMOTION) and
        # /bluerov/convert_to_controls_pose (CONVERT_TO_CONTROLS_POSE).
        goto_base.FromBlackboard.__init__(
            self,
            name=name,
            pose_key=pose_key,
            action_client_type=BlueROVSharedAction.LOCOMOTION,
            service_client_type=BlueROVSharedService.CONVERT_TO_CONTROLS_POSE,
            anchor_frame_name=anchor_frame_name,
            generate_feedback_message=generate_feedback_message,
            wait_for_server_timeout_sec=wait_for_server_timeout_sec,
            wait_for_service_timeout_sec=wait_for_service_timeout_sec,
        )
        self.specified_heading = specified_heading
        self.ignore_depth = ignore_depth
        self.x_threshold = x_threshold
        self.y_threshold = y_threshold
        self.z_threshold = z_threshold
        self.yaw_threshold = yaw_threshold
        self.stabilize_duration = stabilize_duration
        self.is_relative_movement = is_relative_movement
        self.depth_override_value = depth_override_value

    # ── Locomotion engine (folded from the AUV goto so BlueROV owns its goto) ──
    def setup(self, **kwargs):
        """
        Call the grandparent setup method.
        """
        super(goto_base.FromBlackboard, self).setup(**kwargs)

        if self.is_relative_movement:
            return

        self.service_client = self.node.service_clients[self.service_client_type.name]
        self._check_srv_setup()

    def initialise(self):
        """
        Reset internal variables and start new request

        We dont call the action clients initialise here so we have to handle resetting the vars
        """
        self.logger.debug("{}.initialise()".format(self.qualified_name))

        self._reset_internal_vars()

        poses = self.blackboard.get("request")
        # self.node.get_logger().info(f"poses: {poses}")

        if self.is_relative_movement:
            self._initialise_relative(poses)
        else:
            self._initialise_absolute(poses)

    def _initialise_relative(self, poses):
        self.service_future = Future()
        result = GetPoseToControlsFrame.Response()
        result.tf_success = True
        result.output_poses = poses
        # we set as done for the update method
        self.service_future.set_result(result)
        self._send_goal_request(poses)

    def _initialise_absolute(self, poses):
        # this used to be the old intialise method that always calls the convert pose service
        try:
            if self.service_client.service_is_ready():
                self.service_future = self.service_client.call_async(
                    self._gen_srv_req(poses)
                )
                self.feedback_message = "sent service request"
                self.service_future.add_done_callback(self._srv_done_callback)
        except (KeyError, TypeError):
            pass

    def _gen_srv_req(self, poses: list[PoseStamped] | PoseStamped):
        request = GetPoseToControlsFrame.Request()
        request.input_poses = poses if isinstance(poses, list) else [poses]
        request.anchor_frame_name = self.anchor_frame_name
        return request

    def _gen_goal(self, poses: list[PoseStamped]):
        output_poses = [p.pose for p in poses]

        goal_msg = Locomotion.Goal()

        if self.ignore_depth and self.depth_override_value is not None:
            raise ValueError("ignore_depth is True and depth_override_value provided")

        # Set the required fields
        goal_msg.move_rel = self.is_relative_movement
        goal_msg.depth_rel = self.ignore_depth
        goal_msg.heading_rel = self.is_relative_movement

        try:
            goal_msg.depth_ctrl = Locomotion.Goal.DEPTH_MODE_DEPTH
        except Exception as e:
            print(e)
            goal_msg.depth_ctrl = 0

        goal_msg.specified_heading = self.specified_heading

        forward_setpoints = []
        sidemove_setpoints = []
        depth_setpoints = []
        heading_setpoints = []

        for output_pose in output_poses:
            _, _, yaw = rad2deg(
                quat2euler(
                    [
                        output_pose.orientation.w,
                        output_pose.orientation.x,
                        output_pose.orientation.y,
                        output_pose.orientation.z,
                    ]
                )
            )

            forward_setpoints.append(output_pose.position.x)
            sidemove_setpoints.append(output_pose.position.y)
            if self.depth_override_value is not None:
                depth_setpoints.append(self.depth_override_value)
            elif self.ignore_depth:
                depth_setpoints.append(0.0)
            else:
                depth_setpoints.append(output_pose.position.z)
            heading_setpoints.append(yaw)

        # Set the setpoints
        goal_msg.forward_setpoints = forward_setpoints
        goal_msg.sidemove_setpoints = sidemove_setpoints
        goal_msg.depth_setpoints = depth_setpoints
        goal_msg.heading_setpoints = heading_setpoints

        # Lists that need to be populated but aren't used
        goal_msg.roll_setpoints = [0.0 for i in range(len(output_poses))]
        goal_msg.pitch_setpoints = [0.0 for i in range(len(output_poses))]
        goal_msg.altitude_setpoints = []

        goal_msg.forward_tolerance = self.x_threshold
        goal_msg.sidemove_tolerance = self.y_threshold
        goal_msg.depth_tolerance = self.z_threshold
        goal_msg.heading_tolerance = self.yaw_threshold
        goal_msg.max_correction_time = self.stabilize_duration

        return goal_msg


class FromConstant(FromBlackboard):
    """
    Locomotion action client for BlueROV2, using a constant PoseStamped goal.

    Set pose.header.frame_id = 'base_link' for body-relative movement.
    Pose coordinates follow FLU convention: +x=forward, +y=left, +z=up.
    The ConvertToControlsPose service converts to the map frame before dispatch.

    Example — move 2 m forward:
        Goto('leg1', pose=_make_pose(x=2.0, y=0.0))
    """

    def __init__(
        self,
        name: str,
        pose: PoseStamped | list[PoseStamped],
        anchor_frame_name: str = "base_link",
        specified_heading: bool = True,
        ignore_depth: bool = False,
        x_threshold: float = 0.2,
        y_threshold: float = 0.2,
        z_threshold: float = 0.2,
        yaw_threshold: float = 5.0,
        stabilize_duration: int = 60,
        generate_feedback_message: Callable[[Any], str] = None,
        wait_for_server_timeout_sec: int = -3,
        wait_for_service_timeout_sec: int = -3,
        is_relative_movement: bool = False,
        depth_override_value: float | None = None,
    ):
        if not isinstance(pose, list):
            pose = [pose]

        namespace = py_trees.blackboard.Blackboard.absolute_name(
            "/", convert_to_safe_name(name)
        )
        pose_key = py_trees.blackboard.Blackboard.absolute_name(
            namespace, f'pose_{str(uuid.uuid4()).replace("-", "")}'
        )

        super().__init__(
            name=name,
            pose_key=pose_key,
            anchor_frame_name=anchor_frame_name,
            specified_heading=specified_heading,
            ignore_depth=ignore_depth,
            x_threshold=x_threshold,
            y_threshold=y_threshold,
            z_threshold=z_threshold,
            yaw_threshold=yaw_threshold,
            stabilize_duration=stabilize_duration,
            generate_feedback_message=generate_feedback_message,
            wait_for_server_timeout_sec=wait_for_server_timeout_sec,
            wait_for_service_timeout_sec=wait_for_service_timeout_sec,
            is_relative_movement=is_relative_movement,
            depth_override_value=depth_override_value,
        )

        self.blackboard.register_key(
            key="request",
            access=py_trees.common.Access.WRITE,
            remap_to=py_trees.blackboard.Blackboard.absolute_name(
                namespace="/", key=pose_key
            ),
        )
        self.blackboard.set(name="request", value=pose)


class NFromBlackboard(FromBlackboard):
    """
    Sequential N-pose Locomotion action client for BlueROV2, reading list[PoseStamped] from blackboard.

    Same Locomotion engine as FromBlackboard, but ticks through a list of poses
    one at a time with an optional wait between moves:
    - Connects to /bluerov/controls (BlueROVSharedAction.LOCOMOTION)
    - Uses /bluerov/convert_to_controls_pose for frame conversion
    - anchor_frame_name defaults to 'base_link'
    - Relaxed tolerances for MAVROS-based control

    Overrides _start_goal/initialise/update for the wait-between-moves sequencing.
    """

    def __init__(
        self,
        name: str,
        pose_key: str,
        anchor_frame_name: str = "base_link",
        specified_heading: bool = True,
        ignore_depth: bool = False,
        x_threshold: float = 0.2,
        y_threshold: float = 0.2,
        z_threshold: float = 0.2,
        yaw_threshold: float = 5.0,
        stabilize_duration: int = 60,
        generate_feedback_message: Callable[[Any], str] = None,
        wait_for_server_timeout_sec: int = -3,
        wait_for_service_timeout_sec: int = -3,
        wait_between_moves_sec: float = 10.0,
        is_relative_movement: bool = False,
        depth_override_value: float | None = None,
    ):
        # Call goto_base.FromBlackboard.__init__ directly (skip FromBlackboard's
        # own __init__, whose signature has no wait_between_moves_sec) so the
        # BlueROV registries are bound exactly once.
        goto_base.FromBlackboard.__init__(
            self,
            name=name,
            pose_key=pose_key,
            action_client_type=BlueROVSharedAction.LOCOMOTION,
            service_client_type=BlueROVSharedService.CONVERT_TO_CONTROLS_POSE,
            anchor_frame_name=anchor_frame_name,
            generate_feedback_message=generate_feedback_message,
            wait_for_server_timeout_sec=wait_for_server_timeout_sec,
            wait_for_service_timeout_sec=wait_for_service_timeout_sec,
        )

        # Attributes the Locomotion engine (_gen_goal/_initialise_*) reads.
        self.specified_heading = specified_heading
        self.ignore_depth = ignore_depth
        self.x_threshold = x_threshold
        self.y_threshold = y_threshold
        self.z_threshold = z_threshold
        self.yaw_threshold = yaw_threshold
        self.stabilize_duration = stabilize_duration
        self.is_relative_movement = is_relative_movement
        self.depth_override_value = depth_override_value

        # Attributes for multi-pose sequencing.
        self.wait_between_moves_sec = wait_between_moves_sec
        self._waiting = False
        self._wait_start_time = None

    # setup remains unchanged as it is just linking to the pose conversion service

    def _start_goal(self):
        self.logger.debug(
            "{}.initialise() or sending {} goal".format(
                self.qualified_name, self.current_idx
            )
        )

        # Temporary variable
        self.service_future = None

        # None declarations from super.initialise
        self.goal_handle = None
        self.send_goal_future = None
        self.get_result_future = None

        self.result_message = None
        self.result_status = None
        self.result_status_string = None

        if self.is_relative_movement:
            self._initialise_relative([self.poses_list[self.current_idx]])
        else:
            self._initialise_absolute(self.poses_list[self.current_idx])

    # change initialise abit because the service request is one pose
    def initialise(self):
        """
        Reset internal variables and start new request

        We dont call the action clients initialise here so we have to handle resetting the vars
        """
        # Read the list of poses to go through
        self.poses_list = self.blackboard.get("request")
        if not isinstance(self.poses_list, list):
            self.poses_list = [self.poses_list]
        self.num_poses = len(self.poses_list)
        self.current_idx = 0
        self._start_goal()

    # in update, we check shit, if not all poses complete, start a new goal
    def update(self):
        """
        Check whether if underlying service server has succeeded, is running,
        or has cancelled/aborted and map these to behaviour return states
        """
        self.logger.debug("{}.update()".format(self.qualified_name))

        # New waiting state before moves
        if self._waiting:
            elapsed = time.monotonic() - self._wait_start_time
            if elapsed < self.wait_between_moves_sec:
                self.feedback_message = (
                    f"waiting {self.wait_between_moves_sec:.2f} before the next move"
                )
            else:
                self._waiting = False
                self._wait_start_time = None
                self._start_goal()
            return py_trees.common.Status.RUNNING

        if self.service_future is None:
            # No request on blackboard or wrong request type or unready server
            self.feedback_message = "no service request to send"
            return py_trees.common.Status.FAILURE
        elif not self.service_future.done():
            # service has been called but has yet to return a result
            return py_trees.common.Status.RUNNING

        # at this point service is done
        if not self.service_future.result().tf_success:
            return py_trees.common.Status.FAILURE

        # check that in the callback attached the new attr has been set
        # also checks if the attr has been set to True which implies that the send_goal_req has been
        # run to completion
        # this ensures that the call to send_goal_request has completed
        if not self.is_goal_sent:
            return py_trees.common.Status.RUNNING

        # no race condition cuz is RW WR either way the code wont break
        if self.send_goal_future is None:
            self.feedback_message = "no goal to send"
            return py_trees.common.Status.FAILURE
        if self.goal_handle is not None and not self.goal_handle.accepted:
            # goal was rejected
            self.feedback_message = "goal rejected"
            return py_trees.common.Status.FAILURE
        if self.result_status is None:
            return py_trees.common.Status.RUNNING
        elif not self.get_result_future.done():
            # should never get here
            self.node.get_logger().warn(
                "got result, but future not yet done [{}]".format(self.qualified_name)
            )
            return py_trees.common.Status.RUNNING
        else:
            self.node.get_logger().debug("goal result [{}]".format(self.qualified_name))
            self.node.get_logger().debug(
                "  status: {}".format(self.result_status_string)
            )
            self.node.get_logger().debug("  message: {}".format(self.result_message))
            if (
                self.result_status == action_msgs.GoalStatus.STATUS_SUCCEEDED
                and self.current_idx == self.num_poses - 1
            ):
                self.feedback_message = f"all {self.num_poses} moves success"
                return py_trees.common.Status.SUCCESS
            elif (
                self.result_status == action_msgs.GoalStatus.STATUS_SUCCEEDED
                and self.current_idx < self.num_poses
            ):  # noqa
                self.feedback_message = (
                    f"successfully completed move to pose {self.current_idx}"
                )
                self.current_idx += 1

                # Start waiting before next move
                if self.wait_between_moves_sec > 0.0:
                    self._waiting = True
                    self._wait_start_time = time.monotonic()
                else:
                    self._start_goal()
                return py_trees.common.Status.RUNNING
            else:
                self.feedback_message = "failed"
                return py_trees.common.Status.FAILURE


class NFromConstant(NFromBlackboard):
    """
    Sequential N-pose Locomotion action client for BlueROV2, using constant poses.

    Same surface as auv.goto.NFromConstant but BlueROV-routed.
    """

    def __init__(
        self,
        name: str,
        poses: list[PoseStamped],
        anchor_frame_name: str = "base_link",
        specified_heading: bool = True,
        ignore_depth: bool = False,
        x_threshold: float = 0.2,
        y_threshold: float = 0.2,
        z_threshold: float = 0.2,
        yaw_threshold: float = 5.0,
        stabilize_duration: int = 60,
        generate_feedback_message: Callable[[Any], str] = None,
        wait_for_server_timeout_sec: int = -3,
        wait_for_service_timeout_sec: int = -3,
        wait_between_moves_sec: float = 10.0,
        is_relative_movement: bool = False,
        depth_override_value: float | None = None,
    ):
        if not isinstance(poses, list):
            poses = [poses]

        namespace = py_trees.blackboard.Blackboard.absolute_name(
            "/", convert_to_safe_name(name)
        )
        pose_key = py_trees.blackboard.Blackboard.absolute_name(
            namespace, f'pose_{str(uuid.uuid4()).replace("-", "")}'
        )

        super().__init__(
            name=name,
            pose_key=pose_key,
            anchor_frame_name=anchor_frame_name,
            specified_heading=specified_heading,
            ignore_depth=ignore_depth,
            x_threshold=x_threshold,
            y_threshold=y_threshold,
            z_threshold=z_threshold,
            yaw_threshold=yaw_threshold,
            stabilize_duration=stabilize_duration,
            generate_feedback_message=generate_feedback_message,
            wait_for_server_timeout_sec=wait_for_server_timeout_sec,
            wait_for_service_timeout_sec=wait_for_service_timeout_sec,
            wait_between_moves_sec=wait_between_moves_sec,
            is_relative_movement=is_relative_movement,
            depth_override_value=depth_override_value,
        )

        self.blackboard.register_key(
            key="request",
            access=py_trees.common.Access.WRITE,
            remap_to=py_trees.blackboard.Blackboard.absolute_name(
                namespace="/", key=pose_key
            ),
        )
        self.blackboard.set(name="request", value=poses)
