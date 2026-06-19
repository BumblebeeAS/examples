#!/usr/bin/env python3
"""Real actuator node for the BlueROV2 Gazebo (gz Harmonic) sim.

Serves the dropper/torpedo std_srvs/Trigger services the bin and torpedo
behaviour trees call. Each Trigger spawns a real entity in Gazebo at the actuator
link's world pose (looked up from TF) via the `gz` CLI: the dropper marker sinks
into the bin and persists; torpedoes are driven forward by their VelocityControl
plugin and auto-despawn after `torpedo_lifetime` seconds.
"""

import re
import subprocess
import time

import rclpy
import tf2_ros
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.time import Time
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker

MARKER_TOPIC = "/bluerov/actuation/markers"
MARKER_LIFETIME_SEC = 5
GZ_TIMEOUT_S = 5.0

DROPPER_FRAME = "dropper_link"
TORPEDO_FRAMES = {
    "left": "torpedo_shooter_left_link",
    "right": "torpedo_shooter_right_link",
}


def make_marker(node: Node, marker_id: int, frame_id: str, color) -> Marker:
    msg = Marker()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = frame_id
    msg.ns = "bluerov_actuation"
    msg.id = marker_id
    msg.type = Marker.SPHERE
    msg.action = Marker.ADD
    msg.scale.x = msg.scale.y = msg.scale.z = 0.1
    msg.color.r, msg.color.g, msg.color.b = color
    msg.color.a = 1.0
    msg.lifetime.sec = MARKER_LIFETIME_SEC
    return msg


class ActuatorsSim(Node):
    def __init__(self) -> None:
        super().__init__("actuators_sim")

        self.declare_parameter("world_name", "")
        self.declare_parameter("world_frame", "map")
        self.declare_parameter("torpedo_speed", 4.0)
        self.declare_parameter("torpedo_lifetime", 8.0)

        self.world_frame = self.get_parameter("world_frame").value
        self.torpedo_speed = float(self.get_parameter("torpedo_speed").value)
        self.torpedo_lifetime = float(self.get_parameter("torpedo_lifetime").value)

        bb_worlds_share = get_package_share_directory("bb_worlds")
        self.marker_sdf = f"{bb_worlds_share}/models/dropper_marker/model.sdf"
        self.torpedo_sdf = f"{bb_worlds_share}/models/torpedo/model.sdf"

        self.world = self.get_parameter("world_name").value or self.detect_world()

        # The actuator links live in TF (static off base_link); world_frame ->
        # base_link is published live. We look up the actuator link directly.
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.counter = 0
        self.despawn_timers = []
        self.marker_pub = self.create_publisher(Marker, MARKER_TOPIC, 10)

        self.create_service(Trigger, "/bluerov/actuation/dropper", self.handle_dropper)
        self.create_service(
            Trigger,
            "/bluerov/actuation/torpedo/left",
            lambda req, res: self.handle_torpedo(req, res, "left"),
        )
        self.create_service(
            Trigger,
            "/bluerov/actuation/torpedo/right",
            lambda req, res: self.handle_torpedo(req, res, "right"),
        )

        self.get_logger().info(
            f"actuators_sim ready — world='{self.world}' world_frame='{self.world_frame}' "
            "— /bluerov/actuation/{dropper,torpedo/left,torpedo/right} (Trigger)"
        )

    def detect_world(self) -> str:
        """Find the world name from the /world/<world>/create gz service."""
        try:
            out = subprocess.run(
                ["gz", "service", "-l"],
                capture_output=True,
                text=True,
                timeout=GZ_TIMEOUT_S,
            ).stdout
        except (subprocess.SubprocessError, FileNotFoundError) as exc:
            self.get_logger().error(f"could not list gz services: {exc}")
            return ""
        match = re.search(r"/world/([^/]+)/create", out)
        if not match:
            self.get_logger().error("no /world/<world>/create service found yet")
            return ""
        return match.group(1)

    def gz(self, args: list, what: str) -> bool:
        """Run a `gz` command; return True iff it replied `data: true`."""
        try:
            result = subprocess.run(
                args,
                capture_output=True,
                text=True,
                timeout=GZ_TIMEOUT_S,
            )
        except (subprocess.SubprocessError, FileNotFoundError) as exc:
            self.get_logger().error(f"{what} failed: {exc}")
            return False
        ok = result.returncode == 0 and "data:true" in result.stdout.replace(" ", "")
        if not ok:
            self.get_logger().warning(
                f"{what}: rc={result.returncode} out={result.stdout.strip()} "
                f"err={result.stderr.strip()}"
            )
        return ok

    def world_service(self, verb: str, reqtype: str, req: str, what: str) -> bool:
        """Call /world/<world>/<verb> (reply type gz.msgs.Boolean)."""
        return self.gz(
            [
                "gz",
                "service",
                "-s",
                f"/world/{self.world}/{verb}",
                "--reqtype",
                reqtype,
                "--reptype",
                "gz.msgs.Boolean",
                "--timeout",
                "3000",
                "--req",
                req,
            ],
            what,
        )

    def spawn(self, name: str, sdf_path: str, frame: str) -> bool:
        """Spawn the SDF at the world pose of `frame`, taken straight from TF."""
        try:
            tf = self.tf_buffer.lookup_transform(self.world_frame, frame, Time())
        except tf2_ros.TransformException as exc:
            self.get_logger().warning(
                f"no TF {self.world_frame}->{frame} yet — cannot place {name}: {exc}"
            )
            return False
        t = tf.transform.translation
        q = tf.transform.rotation
        req = (
            f'sdf_filename: "{sdf_path}" name: "{name}" allow_renaming: true '
            f"pose: {{position: {{x: {t.x} y: {t.y} z: {t.z}}} "
            f"orientation: {{x: {q.x} y: {q.y} z: {q.z} w: {q.w}}}}}"
        )
        return self.world_service(
            "create", "gz.msgs.EntityFactory", req, f"spawn {name}"
        )

    def remove(self, name: str) -> None:
        self.world_service(
            "remove", "gz.msgs.Entity", f'name: "{name}" type: MODEL', f"remove {name}"
        )

    def drive_forward(self, name: str) -> None:
        """Command a persistent forward body-frame velocity via VelocityControl.

        Published a few times since the freshly spawned model's plugin may not be
        subscribed yet; VelocityControl latches the last command it receives."""
        twist = f"linear: {{x: {self.torpedo_speed}}}"
        for _ in range(3):
            time.sleep(0.2)
            subprocess.run(
                [
                    "gz",
                    "topic",
                    "-t",
                    f"/model/{name}/cmd_vel",
                    "-m",
                    "gz.msgs.Twist",
                    "-p",
                    twist,
                ],
                capture_output=True,
                text=True,
                timeout=GZ_TIMEOUT_S,
            )

    def handle_dropper(self, request, response):
        self.counter += 1
        name = f"dropper_marker_{self.counter}"
        self.get_logger().info(f"dropper fired -> spawning {name}")
        self.marker_pub.publish(make_marker(self, 0, DROPPER_FRAME, (1.0, 0.8, 0.0)))
        ok = self.spawn(name, self.marker_sdf, DROPPER_FRAME)
        response.success = ok
        response.message = f"{name} dropped" if ok else f"failed to spawn {name}"
        return response

    def handle_torpedo(self, request, response, side):
        self.counter += 1
        name = f"torpedo_{side}_{self.counter}"
        frame = TORPEDO_FRAMES[side]
        color = (1.0, 0.2, 0.2) if side == "left" else (0.2, 0.4, 1.0)
        self.get_logger().info(f"torpedo/{side} fired -> launching {name}")
        self.marker_pub.publish(
            make_marker(self, 1 if side == "left" else 2, frame, color)
        )
        ok = self.spawn(name, self.torpedo_sdf, frame)
        if ok:
            self.drive_forward(name)
            self.schedule_despawn(name)
        response.success = ok
        response.message = f"{name} launched" if ok else f"failed to spawn {name}"
        return response

    def schedule_despawn(self, name: str) -> None:
        def fire():
            timer.cancel()
            if timer in self.despawn_timers:
                self.despawn_timers.remove(timer)
            self.remove(name)

        timer = self.create_timer(self.torpedo_lifetime, fire)
        self.despawn_timers.append(timer)


def main(argv=None) -> None:
    rclpy.init(args=argv)
    node = ActuatorsSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
