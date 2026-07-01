#!/usr/bin/env python3
"""BlueBoat square mission publisher.

Provenance: Adapted from https://github.com/BumblebeeAS/bring-up/tree/main/etc/uav2_sim
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped


class BlueBoatSquareMission(Node):
    def __init__(self):
        super().__init__("blueboat_square_mission")

        self.declare_parameter("side", 10.0)
        self.declare_parameter("frame_id", "odom")
        self.side = self.get_parameter("side").value
        self.frame_id = self.get_parameter("frame_id").value

        self.goal_pub = self.create_publisher(PoseStamped, "/blueboat/goal_pose", 10)
        self.create_subscription(Odometry, "/blueboat/odom", self.save_odom, 10)
        self._start = None
        self._published = False
        self.create_timer(0.2, self.try_publish)

    def save_odom(self, msg: Odometry):
        if self._start is None:
            self._start = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def try_publish(self):
        if self._published or self._start is None:
            return
        if self.goal_pub.get_subscription_count() == 0:
            return

        x0, y0 = self._start
        s = self.side
        waypoints = [
            (x0 + s, y0),
            (x0 + s, y0 + s),
            (x0, y0 + s),
            (x0, y0),
        ]

        for wx, wy in waypoints:
            msg = PoseStamped()
            msg.header.frame_id = self.frame_id
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.pose.position.x = wx
            msg.pose.position.y = wy
            msg.pose.orientation.w = 1.0
            self.goal_pub.publish(msg)

        self._published = True


def main(args=None):
    rclpy.init(args=args)
    node = BlueBoatSquareMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
