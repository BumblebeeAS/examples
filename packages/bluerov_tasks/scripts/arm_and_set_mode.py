#!/usr/bin/env python3
"""
py_trees behaviour: ArmAndSetMode

One-shot behaviour that arms the vehicle and sets GUIDED mode via MAVROS
service calls, using the async pattern from bluerov_movement.py.

Returns SUCCESS once both services confirm.
Returns RUNNING while waiting for the FCU or retrying a rejected request.
"""

import time

import py_trees
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from rclpy.node import Node

RETRY_INTERVAL_SEC = 2.0


class ArmAndSetMode(py_trees.behaviour.Behaviour):
    """
    Send arm + set-GUIDED requests to MAVROS and wait for confirmation.

    Follows the TurtleMove.setup(**kwargs) pattern from
    mission_planner_2/vehicles/turtlesim/turtle_circle.py.
    """

    def __init__(self, name: str = "arm_and_set_mode") -> None:
        super().__init__(name)
        self.node: Node = None
        self._arm_client = None
        self._mode_client = None
        self._arm_future = None
        self._mode_future = None
        self._armed = False
        self._guided = False
        self._mavros_connected = False
        self._waiting_for_connection_logged = False
        self._state_subscription = None
        self._next_request_time = 0.0

    def setup(self, **kwargs) -> None:
        self.logger.debug(f"{self.qualified_name}.setup()")
        try:
            self.node = kwargs["node"]
        except KeyError:
            raise KeyError(f"{self.qualified_name} did not find 'node' in kwargs")

        self._arm_client = self.node.create_client(CommandBool, "/mavros/cmd/arming")
        self._mode_client = self.node.create_client(SetMode, "/mavros/set_mode")
        self._state_subscription = self.node.create_subscription(
            State,
            "/mavros/state",
            self._state_callback,
            10,
        )

    def initialise(self) -> None:
        self.logger.debug(f"{self.qualified_name}.initialise()")
        self._arm_future = None
        self._mode_future = None
        self._armed = False
        self._guided = False
        self._waiting_for_connection_logged = False
        self._next_request_time = 0.0

    def update(self) -> py_trees.common.Status:
        self.logger.debug(f"{self.qualified_name}.update()")

        if not self._mavros_connected:
            if not self._waiting_for_connection_logged:
                self.logger.info(
                    f"{self.qualified_name}: waiting for MAVROS FCU connection"
                )
                self._waiting_for_connection_logged = True
            return py_trees.common.Status.RUNNING

        if self._armed and self._guided:
            return py_trees.common.Status.SUCCESS

        # Set GUIDED before attempting to arm. A newly connected ArduSub may
        # transiently reject either request while its pre-arm checks settle.
        if not self._guided:
            self._update_mode_request()
            return py_trees.common.Status.RUNNING

        self._update_arm_request()
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status: py_trees.common.Status) -> None:
        self.logger.debug(
            f"{self.qualified_name}.terminate({self.status} -> {new_status})"
        )

    def _update_mode_request(self) -> None:
        if self._mode_future is not None and self._mode_future.done():
            try:
                result = self._mode_future.result()
                if not result.mode_sent:
                    self.logger.warning(
                        f"{self.qualified_name}: GUIDED request rejected; retrying"
                    )
            except Exception as e:
                self.logger.warning(
                    f"{self.qualified_name}: set_mode service error: {e}; retrying"
                )
            self._mode_future = None
            self._next_request_time = time.monotonic() + RETRY_INTERVAL_SEC

        if self._mode_future is not None or time.monotonic() < self._next_request_time:
            return

        if self._mode_client.service_is_ready():
            req = SetMode.Request()
            req.custom_mode = "GUIDED"
            self._mode_future = self._mode_client.call_async(req)
            self.logger.info(f"{self.qualified_name}: set GUIDED mode request sent")

    def _update_arm_request(self) -> None:
        if self._arm_future is not None and self._arm_future.done():
            try:
                result = self._arm_future.result()
                if not result.success:
                    self.logger.warning(
                        f"{self.qualified_name}: arm request rejected; retrying"
                    )
            except Exception as e:
                self.logger.warning(
                    f"{self.qualified_name}: arm service error: {e}; retrying"
                )
            self._arm_future = None
            self._next_request_time = time.monotonic() + RETRY_INTERVAL_SEC

        if self._arm_future is not None or time.monotonic() < self._next_request_time:
            return

        if self._arm_client.service_is_ready():
            req = CommandBool.Request()
            req.value = True
            self._arm_future = self._arm_client.call_async(req)
            self.logger.info(f"{self.qualified_name}: arm request sent")

    def _state_callback(self, msg: State) -> None:
        if msg.connected and not self._mavros_connected:
            self.logger.info(f"{self.qualified_name}: MAVROS FCU connected")
        if msg.mode == "GUIDED" and not self._guided:
            self.logger.info(f"{self.qualified_name}: GUIDED mode confirmed")
            self._next_request_time = 0.0
        if msg.armed and not self._armed:
            self.logger.info(f"{self.qualified_name}: vehicle armed")
        self._mavros_connected = msg.connected
        self._guided = msg.mode == "GUIDED"
        self._armed = msg.armed
