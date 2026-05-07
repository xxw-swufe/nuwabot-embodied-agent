from __future__ import annotations

from dataclasses import asdict, dataclass, field
from math import atan2, cos, hypot, sin
import os
from threading import Event, Lock, Thread
from time import monotonic, sleep
from typing import Any

try:
    import rclpy
    from geometry_msgs.msg import Twist
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import Odometry
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from tf2_ros import TransformBroadcaster
except ImportError:  # pragma: no cover - allows import outside ROS env
    rclpy = None
    SingleThreadedExecutor = object  # type: ignore[assignment]
    Twist = object  # type: ignore[assignment]
    TransformStamped = object  # type: ignore[assignment]
    Odometry = object  # type: ignore[assignment]
    Node = object  # type: ignore[assignment]
    TransformBroadcaster = object  # type: ignore[assignment]


@dataclass
class OdomSnapshot:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    linear_x: float = 0.0
    linear_y: float = 0.0
    angular_z: float = 0.0
    stamp: float = 0.0
    frame_id: str = ""
    child_frame_id: str = ""
    source_topic: str = ""


class _CmdVelNode(Node):  # type: ignore[misc]
    def __init__(self, state: "RosBridgeState", odom_topic: str) -> None:
        super().__init__("nuwabot_cmd_vel_bridge")
        self._state = state
        self._odom_topic = odom_topic
        self._publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self._subscription = self.create_subscription(Odometry, odom_topic, self._on_odom, 10)
        self._tf_broadcaster = TransformBroadcaster(self)
        self._world_frame = "world"
        self._base_footprint_frame = "base_footprint"
        self._last_stamp = self.get_clock().now().to_msg()

    def _on_odom(self, msg: Odometry) -> None:
        pose = msg.pose.pose.position
        twist = msg.twist.twist
        yaw = self._quat_to_yaw(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        self._last_stamp = msg.header.stamp
        self._state.odom = OdomSnapshot(
            x=float(pose.x),
            y=float(pose.y),
            yaw=yaw,
            linear_x=float(twist.linear.x),
            linear_y=float(twist.linear.y),
            angular_z=float(twist.angular.z),
            stamp=monotonic(),
            frame_id=str(msg.header.frame_id),
            child_frame_id=str(msg.child_frame_id),
            source_topic=self._odom_topic,
        )
        self._publish_world_transform(float(pose.x), float(pose.y), float(yaw))

    @staticmethod
    def _quat_to_yaw(z: float, w: float) -> float:
        # Standard yaw extraction for quaternions when roll and pitch are not needed.
        siny_cosp = 2.0 * (w * z)
        cosy_cosp = 1.0 - 2.0 * (z * z)
        return float(atan2(siny_cosp, cosy_cosp))

    def _publish_world_transform(self, x: float, y: float, yaw: float) -> None:
        transform = TransformStamped()
        transform.header.stamp = self._last_stamp
        transform.header.frame_id = self._world_frame
        transform.child_frame_id = self._base_footprint_frame
        transform.transform.translation.x = float(x)
        transform.transform.translation.y = float(y)
        transform.transform.translation.z = 0.0
        half_yaw = float(yaw) / 2.0
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = sin(half_yaw)
        transform.transform.rotation.w = cos(half_yaw)
        self._tf_broadcaster.sendTransform(transform)


@dataclass
class RosBridgeState:
    connected: bool = False
    last_command: dict[str, Any] | None = None
    odom: OdomSnapshot = field(default_factory=OdomSnapshot)
    last_error: str | None = None
    odom_topic: str = ""


class RosBridge:
    def __init__(self, odom_topic: str | None = None) -> None:
        self.odom_topic = odom_topic or os.getenv("ROBOT_ODOM_TOPIC", "/odom/ground_truth")
        self._state = RosBridgeState(odom_topic=self.odom_topic)
        self._lock = Lock()
        self._stop_event = Event()
        self._thread: Thread | None = None
        self._node: _CmdVelNode | None = None
        self._rclpy_started = False

    def start(self) -> None:
        if rclpy is None:
            self._state.last_error = "rclpy is not available in this environment"
            return
        if self._thread and self._thread.is_alive():
            return
        try:
            if not rclpy.ok():
                rclpy.init(args=None)
                self._rclpy_started = True
            self._node = _CmdVelNode(self._state, self.odom_topic)
            self._state.connected = True
            self._thread = Thread(target=self._spin, daemon=True)
            self._thread.start()
        except Exception as exc:  # pragma: no cover - depends on ROS runtime
            self._node = None
            self._state.connected = False
            self._state.last_error = f"failed to start ROS bridge: {exc}"

    def _spin(self) -> None:
        assert self._node is not None
        executor = SingleThreadedExecutor()
        executor.add_node(self._node)
        try:
            executor.spin()
        except Exception as exc:  # pragma: no cover - runtime safeguard
            self._state.last_error = str(exc)
        finally:
            executor.remove_node(self._node)
            self._state.connected = False

    def publish_twist(
        self,
        linear_x: float,
        angular_z: float,
        linear_y: float = 0.0,
        duration_sec: float = 0.2,
        rate_hz: float = 10.0,
        stop_after: bool = True,
    ) -> dict[str, Any]:
        if self._node is None:
            self.start()
        if self._node is None:
            return self._error_result("ROS bridge is unavailable")

        with self._lock:
            twist = Twist()
            twist.linear.x = float(linear_x)
            twist.linear.y = float(linear_y)
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = float(angular_z)

            start = monotonic()
            if duration_sec <= 0:
                self._node._publisher.publish(twist)
            else:
                period = 1.0 / max(rate_hz, 1.0)
                while monotonic() - start < duration_sec:
                    self._node._publisher.publish(twist)
                    sleep(period)

            if stop_after and (linear_x != 0.0 or linear_y != 0.0 or angular_z != 0.0):
                self._publish_zero_twist()

            self._state.last_command = {
                "linear_x": float(linear_x),
                "linear_y": float(linear_y),
                "angular_z": float(angular_z),
                "duration_sec": float(duration_sec),
                "stop_after": bool(stop_after),
            }
            return {
                "success": True,
                "message": "cmd_vel published",
                "command": self._state.last_command,
            }

    def drive_straight(
        self,
        distance_m: float,
        speed_mps: float,
        rate_hz: float = 20.0,
        timeout_sec: float | None = None,
    ) -> dict[str, Any]:
        if self._node is None:
            self.start()
        if self._node is None:
            return self._error_result("ROS bridge is unavailable")

        target_distance = abs(float(distance_m))
        speed = abs(float(speed_mps))
        if speed <= 0:
            return self._error_result("speed_mps must be positive")

        direction = 1.0 if distance_m >= 0 else -1.0
        linear_x = direction * speed
        estimated_duration = target_distance / speed if target_distance > 0 else 0.0
        # Simulated motion is often slower than the nominal speed setting, so keep a
        # generous default timeout and only fail when the robot is clearly stuck.
        max_duration = timeout_sec if timeout_sec is not None else max(estimated_duration * 10.0, estimated_duration + 5.0, 10.0)

        with self._lock:
            start_odom = self._state.odom
            start_x = float(start_odom.x)
            start_y = float(start_odom.y)
            period = 1.0 / max(rate_hz, 1.0)
            start_time = monotonic()
            traveled = 0.0

            twist = Twist()
            twist.linear.x = float(linear_x)
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 0.0

            timed_out = False
            try:
                while traveled < target_distance:
                    if monotonic() - start_time >= max_duration:
                        timed_out = True
                        break
                    self._node._publisher.publish(twist)
                    sleep(period)
                    odom = self._state.odom
                    traveled = hypot(float(odom.x) - start_x, float(odom.y) - start_y)
            finally:
                self._publish_zero_twist()

            elapsed = monotonic() - start_time
            self._state.last_command = {
                "linear_x": float(linear_x),
                "linear_y": 0.0,
                "angular_z": 0.0,
                "target_distance_m": float(distance_m),
                "traveled_distance_m": float(traveled),
                "duration_sec": float(elapsed),
                "stop_after": True,
                "closed_loop": True,
                "timed_out": bool(timed_out),
            }
            return {
                "success": not timed_out,
                "message": "target distance reached" if not timed_out else "forward command timed out before target distance",
                "command": self._state.last_command,
            }

    def turn_relative(
        self,
        angle_rad: float,
        angular_speed_rps: float,
        rate_hz: float = 20.0,
        tolerance_rad: float = 0.03,
        timeout_sec: float | None = None,
    ) -> dict[str, Any]:
        if self._node is None:
            self.start()
        if self._node is None:
            return self._error_result("ROS bridge is unavailable")

        current_yaw = float(self._state.odom.yaw)
        target_yaw = self._normalize_angle(current_yaw + float(angle_rad))
        return self.turn_to_yaw(target_yaw, angular_speed_rps, rate_hz, tolerance_rad, timeout_sec)

    def turn_to_yaw(
        self,
        target_yaw: float,
        angular_speed_rps: float,
        rate_hz: float = 20.0,
        tolerance_rad: float = 0.03,
        timeout_sec: float | None = None,
    ) -> dict[str, Any]:
        if self._node is None:
            self.start()
        if self._node is None:
            return self._error_result("ROS bridge is unavailable")

        speed = abs(float(angular_speed_rps))
        if speed <= 0:
            return self._error_result("angular_speed_rps must be positive")

        target = self._normalize_angle(float(target_yaw))
        initial_error = self._normalize_angle(target - float(self._state.odom.yaw))
        estimated_duration = abs(initial_error) / speed if speed > 0 else 0.0
        max_duration = timeout_sec if timeout_sec is not None else max(estimated_duration * 8.0, estimated_duration + 3.0, 5.0)

        with self._lock:
            period = 1.0 / max(rate_hz, 1.0)
            start_time = monotonic()
            error = initial_error
            timed_out = False

            twist = Twist()
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0

            try:
                while abs(error) > tolerance_rad:
                    if monotonic() - start_time >= max_duration:
                        timed_out = True
                        break
                    error = self._normalize_angle(target - float(self._state.odom.yaw))
                    twist.angular.z = speed if error >= 0 else -speed
                    self._node._publisher.publish(twist)
                    sleep(period)
                    error = self._normalize_angle(target - float(self._state.odom.yaw))
            finally:
                self._publish_zero_twist()

            elapsed = monotonic() - start_time
            final_yaw = float(self._state.odom.yaw)
            final_error = self._normalize_angle(target - final_yaw)
            self._state.last_command = {
                "angular_z": speed if initial_error >= 0 else -speed,
                "target_yaw": float(target),
                "final_yaw": float(final_yaw),
                "yaw_error_rad": float(final_error),
                "duration_sec": float(elapsed),
                "stop_after": True,
                "closed_loop": True,
                "timed_out": bool(timed_out),
            }
            return {
                "success": not timed_out,
                "message": "target yaw reached" if not timed_out else "turn command timed out before target yaw",
                "command": self._state.last_command,
            }

    def stop(self) -> dict[str, Any]:
        return self.publish_twist(0.0, 0.0, duration_sec=0.1, stop_after=False)

    def get_status(self) -> dict[str, Any]:
        return {
            "connected": self._state.connected,
            "last_error": self._state.last_error,
            "last_command": self._state.last_command,
            "odom_topic": self._state.odom_topic,
            "odom": asdict(self._state.odom),
        }

    def get_pose(self) -> dict[str, Any]:
        odom = self._state.odom
        return {
            "x": float(odom.x),
            "y": float(odom.y),
            "yaw": float(odom.yaw),
            "frame_id": odom.frame_id,
            "child_frame_id": odom.child_frame_id,
            "source_topic": odom.source_topic or self.odom_topic,
        }

    def shutdown(self) -> None:
        if rclpy is None:
            return
        if self._node is not None:
            self._node.destroy_node()
            self._node = None
        if self._rclpy_started and rclpy.ok():
            rclpy.shutdown()
        self._rclpy_started = False
        self._state.connected = False

    def _publish_zero_twist(self, repeat: int = 3) -> None:
        if self._node is None:
            return
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        for _ in range(max(repeat, 1)):
            self._node._publisher.publish(twist)
            sleep(0.02)

    @staticmethod
    def _normalize_angle(angle_rad: float) -> float:
        return atan2(sin(angle_rad), cos(angle_rad))

    def _error_result(self, message: str) -> dict[str, Any]:
        self._state.last_error = message
        return {"success": False, "message": message}
