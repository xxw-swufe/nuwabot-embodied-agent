from __future__ import annotations

from dataclasses import dataclass
from math import atan2
import os
from threading import Lock
from time import monotonic
from typing import Any

try:
    import rclpy
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import PoseStamped
    from nav2_msgs.action import NavigateToPose
    from rclpy.action import ActionClient
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
except ImportError:  # pragma: no cover - allows import outside ROS/Nav2 env
    rclpy = None
    GoalStatus = None  # type: ignore[assignment]
    PoseStamped = object  # type: ignore[assignment]
    NavigateToPose = object  # type: ignore[assignment]
    ActionClient = object  # type: ignore[assignment]
    SingleThreadedExecutor = object  # type: ignore[assignment]
    Node = object  # type: ignore[assignment]


@dataclass
class Nav2BridgeState:
    connected: bool = False
    action_name: str = ""
    frame_id: str = ""
    last_goal: dict[str, Any] | None = None
    last_result: dict[str, Any] | None = None
    last_error: str | None = None
    navigation_count: int = 0


class _Nav2Node(Node):  # type: ignore[misc]
    def __init__(self, state: Nav2BridgeState, action_name: str, frame_id: str) -> None:
        super().__init__("nuwabot_nav2_bridge")
        self._state = state
        self._action_name = action_name
        self._frame_id = frame_id
        self._client = ActionClient(self, NavigateToPose, action_name)
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self)

    def navigate_to_pose(
        self,
        x: float,
        y: float,
        yaw: float,
        wait_timeout_sec: float,
        result_timeout_sec: float,
    ) -> dict[str, Any]:
        if not self._client.wait_for_server(timeout_sec=wait_timeout_sec):
            return {
                "success": False,
                "available": False,
                "message": f"Nav2 action server '{self._action_name}' is not available",
            }

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self._make_pose_stamped(x, y, yaw)

        send_future = self._client.send_goal_async(goal_msg)
        self._executor.spin_until_future_complete(send_future, timeout_sec=wait_timeout_sec)
        goal_handle = send_future.result()
        if goal_handle is None:
            return {
                "success": False,
                "available": True,
                "message": "Nav2 goal send timed out",
            }
        if not goal_handle.accepted:
            return {
                "success": False,
                "available": True,
                "message": "Nav2 goal was rejected",
            }

        result_future = goal_handle.get_result_async()
        started = monotonic()
        while not result_future.done():
            if monotonic() - started > result_timeout_sec:
                cancel_future = goal_handle.cancel_goal_async()
                self._executor.spin_until_future_complete(cancel_future, timeout_sec=2.0)
                return {
                    "success": False,
                    "available": True,
                    "accepted": True,
                    "timed_out": True,
                    "message": "Nav2 navigation timed out",
                }
            self._executor.spin_once(timeout_sec=0.1)

        result = result_future.result()
        status = int(result.status)
        succeeded = bool(GoalStatus is not None and status == GoalStatus.STATUS_SUCCEEDED)
        return {
            "success": succeeded,
            "available": True,
            "accepted": True,
            "status": status,
            "status_label": self._status_label(status),
            "message": "Nav2 target reached" if succeeded else f"Nav2 finished with status {status}",
        }

    def _make_pose_stamped(self, x: float, y: float, yaw: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self._frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0
        pose.pose.orientation.z = float(atan2(0.0, 1.0))
        pose.pose.orientation.w = 1.0

        half_yaw = float(yaw) / 2.0
        from math import cos, sin

        pose.pose.orientation.z = sin(half_yaw)
        pose.pose.orientation.w = cos(half_yaw)
        return pose

    @staticmethod
    def _status_label(status: int) -> str:
        if GoalStatus is None:
            return "unknown"
        labels = {
            GoalStatus.STATUS_UNKNOWN: "unknown",
            GoalStatus.STATUS_ACCEPTED: "accepted",
            GoalStatus.STATUS_EXECUTING: "executing",
            GoalStatus.STATUS_CANCELING: "canceling",
            GoalStatus.STATUS_SUCCEEDED: "succeeded",
            GoalStatus.STATUS_CANCELED: "canceled",
            GoalStatus.STATUS_ABORTED: "aborted",
        }
        return labels.get(status, f"status_{status}")

    def destroy_node(self) -> bool:
        self._executor.remove_node(self)
        return super().destroy_node()


class Nav2Bridge:
    def __init__(
        self,
        action_name: str | None = None,
        frame_id: str | None = None,
    ) -> None:
        self.action_name = action_name or os.getenv("ROBOT_NAV2_ACTION", "navigate_to_pose")
        self.frame_id = frame_id or os.getenv("ROBOT_NAV2_FRAME", "map")
        self._state = Nav2BridgeState(action_name=self.action_name, frame_id=self.frame_id)
        self._lock = Lock()
        self._node: _Nav2Node | None = None
        self._rclpy_started = False

    def start(self) -> None:
        if rclpy is None:
            self._state.last_error = "Nav2 Python messages are not available in this environment"
            return
        if self._node is not None:
            return
        try:
            if not rclpy.ok():
                rclpy.init(args=None)
                self._rclpy_started = True
            self._node = _Nav2Node(self._state, self.action_name, self.frame_id)
            self._state.connected = True
        except Exception as exc:  # pragma: no cover - depends on ROS runtime
            self._node = None
            self._state.connected = False
            self._state.last_error = f"failed to start Nav2 bridge: {exc}"

    def navigate_to_pose(
        self,
        x: float,
        y: float,
        yaw: float = 0.0,
        wait_timeout_sec: float = 1.0,
        result_timeout_sec: float = 120.0,
    ) -> dict[str, Any]:
        with self._lock:
            if self._node is None:
                self.start()
            if self._node is None:
                result = {
                    "success": False,
                    "available": False,
                    "message": self._state.last_error or "Nav2 bridge is unavailable",
                }
                self._state.last_result = result
                return result

            self._state.last_goal = {
                "x": float(x),
                "y": float(y),
                "yaw": float(yaw),
                "frame_id": self.frame_id,
                "action_name": self.action_name,
            }
            result = self._node.navigate_to_pose(
                x=float(x),
                y=float(y),
                yaw=float(yaw),
                wait_timeout_sec=float(wait_timeout_sec),
                result_timeout_sec=float(result_timeout_sec),
            )
            self._state.navigation_count += 1
            self._state.last_result = result
            if not result.get("success", False):
                self._state.last_error = result.get("message")
            return result

    def get_status(self) -> dict[str, Any]:
        return {
            "connected": self._state.connected,
            "action_name": self._state.action_name,
            "frame_id": self._state.frame_id,
            "last_goal": self._state.last_goal,
            "last_result": self._state.last_result,
            "last_error": self._state.last_error,
            "navigation_count": self._state.navigation_count,
        }

    def shutdown(self) -> None:
        if self._node is not None:
            self._node.destroy_node()
            self._node = None
        if self._rclpy_started and rclpy is not None and rclpy.ok():
            rclpy.shutdown()
        self._rclpy_started = False
        self._state.connected = False
