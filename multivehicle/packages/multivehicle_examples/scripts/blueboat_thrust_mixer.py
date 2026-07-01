#!/usr/bin/env python3
"""BlueBoat differential-thrust mixer.

Provenance: Adapted from https://github.com/BumblebeeAS/bring-up/tree/main/etc/uav2_sim
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

from multivehicle_examples.math_helpers import clamp_symmetric


class BlueBoatThrustMixer(Node):
    def __init__(self):
        super().__init__("blueboat_thrust_mixer")

        self.declare_parameter("track_width", 0.59)
        self.declare_parameter("max_thrust", 30.0)
        self.declare_parameter("surge_drag", 58.42)
        self.declare_parameter("surge_lin", 0.0)
        self.declare_parameter("v_max", 2.0)
        self.declare_parameter("yaw_gain", 30.0)
        self.declare_parameter("w_max", 1.0)
        self.declare_parameter("invert_yaw", False)
        self.declare_parameter("cmd_vel_topic", "/blueboat/cmd_vel")
        self.declare_parameter("left_thrust_topic", "/blueboat/thrusters/left/thrust")
        self.declare_parameter("right_thrust_topic", "/blueboat/thrusters/right/thrust")
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("cmd_timeout", 0.5)

        self.track_width = self.get_parameter("track_width").value
        self.max_thrust = self.get_parameter("max_thrust").value
        self.surge_drag = self.get_parameter("surge_drag").value
        self.surge_lin = self.get_parameter("surge_lin").value
        self.v_max = self.get_parameter("v_max").value
        self.yaw_gain = self.get_parameter("yaw_gain").value
        self.w_max = self.get_parameter("w_max").value
        self.invert_yaw = self.get_parameter("invert_yaw").value
        self.cmd_timeout = self.get_parameter("cmd_timeout").value
        rate = self.get_parameter("publish_rate_hz").value

        self.half_track = max(self.track_width / 2.0, 1e-3)

        self.left_pub = self.create_publisher(
            Float64, self.get_parameter("left_thrust_topic").value, 10
        )
        self.right_pub = self.create_publisher(
            Float64, self.get_parameter("right_thrust_topic").value, 10
        )
        self.create_subscription(
            Twist, self.get_parameter("cmd_vel_topic").value, self.cmd_vel_sub, 10
        )

        self._last_cmd = Twist()
        self._last_cmd_time = None
        self.create_timer(1.0 / rate, self.publish_thrust)

    def cmd_vel_sub(self, msg: Twist):
        self._last_cmd = msg
        self._last_cmd_time = self.get_clock().now()

    def publish_thrust(self):
        stale = self._last_cmd_time is None or (
            (self.get_clock().now() - self._last_cmd_time).nanoseconds * 1e-9
            > self.cmd_timeout
        )
        if stale:
            self._send(0.0, 0.0)
            return

        u = clamp_symmetric(self._last_cmd.linear.x, self.v_max)
        w = clamp_symmetric(self._last_cmd.angular.z, self.w_max)
        if self.invert_yaw:
            w = -w

        f_surge = self.surge_drag * u * abs(u) + self.surge_lin * u
        m_z = self.yaw_gain * w

        f_left = f_surge / 2.0 - m_z / (2.0 * self.half_track)
        f_right = f_surge / 2.0 + m_z / (2.0 * self.half_track)

        peak = max(abs(f_left), abs(f_right), self.max_thrust)
        if peak > self.max_thrust:
            scale = self.max_thrust / peak
            f_left *= scale
            f_right *= scale

        self._send(f_left, f_right)

    def _send(self, left, right):
        self.left_pub.publish(Float64(data=float(left)))
        self.right_pub.publish(Float64(data=float(right)))


def main(args=None):
    rclpy.init(args=args)
    node = BlueBoatThrustMixer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
