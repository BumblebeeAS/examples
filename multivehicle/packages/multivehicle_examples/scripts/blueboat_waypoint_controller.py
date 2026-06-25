#!/usr/bin/env python3
"""BlueBoat LOS waypoint controller.

Provenance: Adapted from https://github.com/BumblebeeAS/bring-up/tree/main/etc/uav2_sim
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool

from multivehicle_examples.math_helpers import (
    clamp_symmetric,
    wrap_to_pi,
    yaw_from_quaternion,
)


class BlueBoatWaypointController(Node):
    def __init__(self):
        super().__init__("blueboat_waypoint_controller")

        self.declare_parameter("delta_min", 2.5)
        self.declare_parameter("delta_max", 8.0)
        self.declare_parameter("k_delta", 1.0)
        self.declare_parameter("acceptance_radius", 0.2)
        self.declare_parameter("u_cruise", 1.0)
        self.declare_parameter("turn_shaping_p", 1.5)
        self.declare_parameter("r_slow", 4.0)
        self.declare_parameter("turn_in_place", True)
        self.declare_parameter("arrival_radius", 0.6)
        self.declare_parameter("align_tol_deg", 3.0)
        self.declare_parameter("align_settle_degps", 5.0)
        self.declare_parameter("kp_pos", 0.7)
        self.declare_parameter("u_align_max", 0.5)
        self.declare_parameter("repoint_radius", 1.0)
        self.declare_parameter("v_settle", 0.1)
        self.declare_parameter("kp_yaw", 2.0)
        self.declare_parameter("ki_yaw", 0.0)
        self.declare_parameter("kd_yaw", 0.5)
        self.declare_parameter("w_max", 1.0)
        self.declare_parameter("i_max", 0.5)
        self.declare_parameter("control_rate_hz", 20.0)

        self.declare_parameter("odom_topic", "/blueboat/odom")
        self.declare_parameter("cmd_vel_topic", "/blueboat/cmd_vel")
        self.declare_parameter("goal_pose_topic", "/blueboat/goal_pose")

        self.delta_min = self.get_parameter("delta_min").value
        self.delta_max = self.get_parameter("delta_max").value
        self.k_delta = self.get_parameter("k_delta").value
        self.accept_r = self.get_parameter("acceptance_radius").value
        self.u_cruise = self.get_parameter("u_cruise").value
        self.shaping_p = self.get_parameter("turn_shaping_p").value
        self.r_slow = self.get_parameter("r_slow").value
        self.turn_in_place = self.get_parameter("turn_in_place").value
        self.arrival_radius = self.get_parameter("arrival_radius").value
        self.align_tol = math.radians(self.get_parameter("align_tol_deg").value)
        self.align_settle = math.radians(self.get_parameter("align_settle_degps").value)
        self.kp_pos = self.get_parameter("kp_pos").value
        self.u_align_max = self.get_parameter("u_align_max").value
        self.repoint_radius = self.get_parameter("repoint_radius").value
        self.v_settle = self.get_parameter("v_settle").value
        self.kp_yaw = self.get_parameter("kp_yaw").value
        self.ki_yaw = self.get_parameter("ki_yaw").value
        self.kd_yaw = self.get_parameter("kd_yaw").value
        self.w_max = self.get_parameter("w_max").value
        self.i_max = self.get_parameter("i_max").value
        self.dt = 1.0 / self.get_parameter("control_rate_hz").value

        self.waypoints = []
        self.wp_idx = 0
        self.seg_start = None
        self.first_leg_start = None

        self.odom = None
        self.yaw_integral = 0.0
        self.finished = False
        self.mode = "INIT"
        self.hold_xy = None
        self.target_heading = 0.0

        self.cmd_pub = self.create_publisher(
            Twist, self.get_parameter("cmd_vel_topic").value, 10
        )
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.reached_pub = self.create_publisher(Bool, "/blueboat/goal_reached", latched)
        self.create_subscription(
            Odometry, self.get_parameter("odom_topic").value, self.odom_sub, 10
        )
        self.create_subscription(
            PoseStamped, self.get_parameter("goal_pose_topic").value, self.goal_sub, 10
        )

        self.create_timer(self.dt, self.control_step)

    def odom_sub(self, msg: Odometry):
        self.odom = msg

    def goal_sub(self, msg: PoseStamped):
        self.waypoints.append((msg.pose.position.x, msg.pose.position.y))
        if self.finished:
            self.finished = False
            self.reached_pub.publish(Bool(data=False))

    def control_step(self):
        if self.odom is None or not self.waypoints:
            return

        x = self.odom.pose.pose.position.x
        y = self.odom.pose.pose.position.y
        yaw = yaw_from_quaternion(self.odom.pose.pose.orientation)
        r = self.odom.twist.twist.angular.z

        if self.first_leg_start is None:
            self.first_leg_start = (x, y)

        if self.mode == "INIT":
            self.seg_start = (x, y)
            self.hold_xy = (x, y)
            self.target_heading = self.leg_heading((x, y), self.waypoints[0])
            self.mode = "ALIGN" if self.turn_in_place else "DRIVE"

        if self.finished and self.wp_idx < len(self.waypoints):
            self.finished = False
            self.reached_pub.publish(Bool(data=False))
            self.seg_start = (x, y)
            self.hold_xy = (x, y)
            self.target_heading = self.leg_heading((x, y), self.waypoints[self.wp_idx])
            self.mode = "ALIGN" if self.turn_in_place else "DRIVE"

        if self.wp_idx >= len(self.waypoints):
            if not self.finished:
                self.finished = True
                self.reached_pub.publish(Bool(data=True))
            self.mode = "HOLD"
            self.station_keep(x, y, yaw, r, allow_repoint=True)
            return

        if self.mode == "ALIGN":
            u_now = self.odom.twist.twist.linear.x
            e_x = self.along_body_err(x, y, yaw)
            u_cmd = clamp_symmetric(self.kp_pos * e_x, self.u_align_max)
            if abs(u_now) > self.v_settle:
                self.publish_cmd(u_cmd, 0.0)
                return
            w_cmd, psi_err = self.heading_pid(self.target_heading, yaw, r)
            self.publish_cmd(u_cmd, w_cmd)
            if abs(psi_err) < self.align_tol and abs(r) < self.align_settle:
                self.mode = "DRIVE"
                self.yaw_integral = 0.0
            return

        sx, sy = self.seg_start if self.seg_start is not None else self.first_leg_start
        tx, ty = self.waypoints[self.wp_idx]
        seg_dx, seg_dy = tx - sx, ty - sy
        seg_len = math.hypot(seg_dx, seg_dy)
        dist_to_target = math.hypot(tx - x, ty - y)

        if seg_len < 1e-3:
            psi_d = math.atan2(ty - y, tx - x)
            along = 0.0
        else:
            gamma = math.atan2(seg_dy, seg_dx)
            e = -(x - sx) * math.sin(gamma) + (y - sy) * math.cos(gamma)
            along = (x - sx) * math.cos(gamma) + (y - sy) * math.sin(gamma)
            delta = max(
                self.delta_min,
                min(self.delta_max, self.delta_min + self.k_delta * abs(e)),
            )
            psi_d = gamma + math.atan2(-e, delta)

        passed = seg_len > 1e-3 and along >= seg_len
        if dist_to_target < self.arrival_radius or passed:
            self.seg_start = (tx, ty)
            self.hold_xy = (tx, ty)
            nxt = self.wp_idx + 1
            if nxt < len(self.waypoints):
                self.target_heading = self.leg_heading((tx, ty), self.waypoints[nxt])
            else:
                self.target_heading = yaw
            self.wp_idx = nxt
            self.yaw_integral = 0.0
            if self.turn_in_place and nxt < len(self.waypoints):
                self.mode = "ALIGN"
            return

        w_cmd, psi_err = self.heading_pid(psi_d, yaw, r)
        turn_factor = max(0.0, math.cos(psi_err)) ** self.shaping_p
        slow = min(1.0, dist_to_target / self.r_slow)
        u_cmd = self.u_cruise * turn_factor * slow
        self.publish_cmd(u_cmd, w_cmd)

    def publish_cmd(self, u, w):
        cmd = Twist()
        cmd.linear.x = float(u)
        cmd.angular.z = float(w)
        self.cmd_pub.publish(cmd)

    def heading_pid(self, psi_d, yaw, r):
        psi_err = wrap_to_pi(psi_d - yaw)
        if self.ki_yaw > 0.0:
            self.yaw_integral += psi_err * self.dt
            self.yaw_integral = clamp_symmetric(self.yaw_integral, self.i_max / self.ki_yaw)
        else:
            self.yaw_integral = 0.0
        w_cmd = clamp_symmetric(
            self.kp_yaw * psi_err + self.ki_yaw * self.yaw_integral - self.kd_yaw * r,
            self.w_max,
        )
        return w_cmd, psi_err

    def along_body_err(self, x, y, yaw):
        dx = self.hold_xy[0] - x
        dy = self.hold_xy[1] - y
        return dx * math.cos(yaw) + dy * math.sin(yaw)

    @staticmethod
    def leg_heading(a, b):
        return math.atan2(b[1] - a[1], b[0] - a[0])

    def station_keep(self, x, y, yaw, r, allow_repoint):
        rng = math.hypot(self.hold_xy[0] - x, self.hold_xy[1] - y)
        if rng < self.accept_r:
            w_cmd, _ = self.heading_pid(self.target_heading, yaw, r)
            self.publish_cmd(0.0, w_cmd)
            return
        if allow_repoint and rng > self.repoint_radius:
            psi_d = math.atan2(self.hold_xy[1] - y, self.hold_xy[0] - x)
        else:
            psi_d = self.target_heading
        w_cmd, _ = self.heading_pid(psi_d, yaw, r)
        u_cmd = clamp_symmetric(self.kp_pos * self.along_body_err(x, y, yaw), self.u_align_max)
        self.publish_cmd(u_cmd, w_cmd)


def main(args=None):
    rclpy.init(args=args)
    node = BlueBoatWaypointController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
