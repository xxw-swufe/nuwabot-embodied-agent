from __future__ import annotations

from dataclasses import dataclass, asdict, field
from math import atan2, cos, hypot, sin
from uuid import uuid4
from time import sleep
from typing import Any

try:
    from .location_store import LocationStore
    from .nav2_bridge import Nav2Bridge
    from .path_planner import AStarPlanner, PathPlan, StraightLinePlanner, Waypoint, WorldMap
    from .ros_bridge import RosBridge
    from .task_logger import TaskLogger
except ImportError:  # pragma: no cover - fallback for direct uvicorn usage
    from location_store import LocationStore
    from nav2_bridge import Nav2Bridge
    from path_planner import AStarPlanner, PathPlan, StraightLinePlanner, Waypoint, WorldMap
    from ros_bridge import RosBridge
    from task_logger import TaskLogger


@dataclass
class RobotPose:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


@dataclass
class RobotStatus:
    state: str = "idle"
    battery: float = 100.0
    pose: RobotPose = field(default_factory=RobotPose)
    last_task_id: str | None = None
    last_message: str = "ready"


class RobotController:
    def __init__(
        self,
        logger: TaskLogger | None = None,
        bridge: RosBridge | None = None,
        planner: StraightLinePlanner | None = None,
        nav2: Nav2Bridge | None = None,
        locations: LocationStore | None = None,
    ) -> None:
        self.logger = logger or TaskLogger()
        self.bridge = bridge or RosBridge()
        self.planner = planner or StraightLinePlanner()
        self.world_map = WorldMap.from_file()
        self.nav2 = nav2 or Nav2Bridge()
        self.locations = locations or LocationStore()
        self.bridge.start()
        self.status = RobotStatus()

    def get_status(self) -> dict[str, Any]:
        bridge_status = self.bridge.get_status()
        return {
            "state": self.status.state,
            "battery": self.status.battery,
            "pose": asdict(self.status.pose),
            "last_task_id": self.status.last_task_id,
            "last_message": self.status.last_message,
            "bridge": bridge_status,
            "nav2": self.nav2.get_status(),
        }

    def send_twist(
        self,
        linear_x: float,
        angular_z: float,
        duration_sec: float = 0.2,
        linear_y: float = 0.0,
    ) -> dict[str, Any]:
        task_id = str(uuid4())
        self.status.state = "executing"
        self.status.last_task_id = task_id
        bridge_result = self.bridge.publish_twist(
            linear_x=linear_x,
            angular_z=angular_z,
            linear_y=linear_y,
            duration_sec=duration_sec,
        )
        self.status.state = "idle"
        self.status.last_message = bridge_result.get("message", "cmd_vel published")
        self.status.pose = RobotPose(
            x=self.bridge.get_status()["odom"]["x"],
            y=self.bridge.get_status()["odom"]["y"],
            yaw=self.bridge.get_status()["odom"]["yaw"],
        )

        result = {
            "task_id": task_id,
            "success": bool(bridge_result.get("success", False)),
            "command": {
                "linear_x": linear_x,
                "linear_y": linear_y,
                "angular_z": angular_z,
                "duration_sec": duration_sec,
            },
            "final_position": asdict(self.status.pose),
            "bridge": bridge_result,
            "message": self.status.last_message,
        }
        self.logger.write(
            task_id,
            "send_twist",
            "success" if result["success"] else "failed",
            self.status.last_message,
            result,
        )
        return result

    def forward(self, distance_m: float, speed_mps: float = 0.2) -> dict[str, Any]:
        if speed_mps <= 0:
            raise ValueError("speed_mps must be positive")
        duration_sec = abs(distance_m) / speed_mps
        task_id = str(uuid4())
        self.status.state = "executing"
        self.status.last_task_id = task_id
        bridge_result = self.bridge.drive_straight(distance_m=distance_m, speed_mps=speed_mps)
        self.status.state = "idle"
        self.status.last_message = bridge_result.get("message", "forward completed")
        self.status.pose = RobotPose(
            x=self.bridge.get_status()["odom"]["x"],
            y=self.bridge.get_status()["odom"]["y"],
            yaw=self.bridge.get_status()["odom"]["yaw"],
        )
        result = {
            "task_id": task_id,
            "success": bool(bridge_result.get("success", False)),
            "command": {
                "distance_m": float(distance_m),
                "speed_mps": float(speed_mps),
                "duration_sec": float(duration_sec),
                "closed_loop": True,
            },
            "final_position": asdict(self.status.pose),
            "bridge": bridge_result,
            "message": self.status.last_message,
        }
        self.logger.write(
            task_id,
            "forward",
            "success" if result["success"] else "failed",
            self.status.last_message,
            result,
        )
        return result

    def turn(self, angle_rad: float, angular_speed_rps: float = 0.5) -> dict[str, Any]:
        if angular_speed_rps <= 0:
            raise ValueError("angular_speed_rps must be positive")
        duration_sec = abs(angle_rad) / angular_speed_rps
        task_id = str(uuid4())
        self.status.state = "executing"
        self.status.last_task_id = task_id
        bridge_result = self.bridge.turn_relative(angle_rad=angle_rad, angular_speed_rps=angular_speed_rps)
        self.status.state = "idle"
        self.status.last_message = bridge_result.get("message", "turn completed")
        self.status.pose = RobotPose(
            x=self.bridge.get_status()["odom"]["x"],
            y=self.bridge.get_status()["odom"]["y"],
            yaw=self.bridge.get_status()["odom"]["yaw"],
        )
        result = {
            "task_id": task_id,
            "success": bool(bridge_result.get("success", False)),
            "command": {
                "angle_rad": float(angle_rad),
                "angular_speed_rps": float(angular_speed_rps),
                "duration_sec": float(duration_sec),
            },
            "final_position": asdict(self.status.pose),
            "bridge": bridge_result,
            "message": self.status.last_message,
        }
        self.logger.write(
            task_id,
            "turn",
            "success" if result["success"] else "failed",
            self.status.last_message,
            result,
        )
        return result

    def go_to(
        self,
        x: float,
        y: float,
        speed_mps: float = 0.2,
        angular_speed_rps: float = 0.5,
        tolerance_m: float = 0.15,
        max_steps: int = 100,
        waypoint_spacing_m: float = 0.5,
        planner: str = "astar",
        nav2_timeout_sec: float = 120.0,
    ) -> dict[str, Any]:
        if speed_mps <= 0:
            raise ValueError("speed_mps must be positive")
        if angular_speed_rps <= 0:
            raise ValueError("angular_speed_rps must be positive")
        if tolerance_m <= 0:
            raise ValueError("tolerance_m must be positive")
        if waypoint_spacing_m <= 0:
            raise ValueError("waypoint_spacing_m must be positive")
        if nav2_timeout_sec <= 0:
            raise ValueError("nav2_timeout_sec must be positive")

        task_id = str(uuid4())
        self.status.state = "executing"
        self.status.last_task_id = task_id
        target_x = float(x)
        target_y = float(y)
        planner_name = self._normalize_planner_name(planner)

        if planner_name in ("auto", "nav2"):
            nav2_result = self._go_to_with_nav2(
                task_id=task_id,
                target_x=target_x,
                target_y=target_y,
                tolerance_m=tolerance_m,
                timeout_sec=nav2_timeout_sec,
                requested_planner=planner_name,
                record_result=planner_name == "nav2",
            )
            if planner_name == "nav2" or nav2_result.get("success", False):
                if not nav2_result.get("recorded", False):
                    self._record_go_to_result(task_id, nav2_result)
                return nav2_result

        local_planner = "astar" if planner_name in {"auto", "astar"} else planner_name
        return self._go_to_with_waypoints(
            task_id=task_id,
            target_x=target_x,
            target_y=target_y,
            speed_mps=speed_mps,
            angular_speed_rps=angular_speed_rps,
            tolerance_m=tolerance_m,
            max_steps=max_steps,
            waypoint_spacing_m=waypoint_spacing_m,
            planner=local_planner,
            fallback_from="nav2" if planner_name == "auto" else None,
            fallback_result=nav2_result if planner_name == "auto" else None,
        )

    def _go_to_with_waypoints(
        self,
        task_id: str,
        target_x: float,
        target_y: float,
        speed_mps: float,
        angular_speed_rps: float,
        tolerance_m: float,
        max_steps: int,
        waypoint_spacing_m: float,
        planner: str,
        fallback_from: str | None = None,
        fallback_result: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        self.status.state = "executing"
        steps: list[dict[str, Any]] = []
        success = False
        message = "go_to did not reach target"
        start_pose = self.bridge.get_pose()
        try:
            plan = self._make_local_plan(
                planner=planner,
                start_x=start_pose["x"],
                start_y=start_pose["y"],
                target_x=target_x,
                target_y=target_y,
                waypoint_spacing_m=waypoint_spacing_m,
            )
        except ValueError as exc:
            message = str(exc)
            self.status.state = "idle"
            self.status.last_message = message
            result = {
                "task_id": task_id,
                "success": False,
                "target": {
                    "x": target_x,
                    "y": target_y,
                    "tolerance_m": float(tolerance_m),
                },
                "planner_request": planner,
                "fallback_from": fallback_from,
                "fallback_result": fallback_result,
                "plan": None,
                "final_position": start_pose,
                "final_distance_m": float(hypot(target_x - start_pose["x"], target_y - start_pose["y"])),
                "steps": steps,
                "message": message,
            }
            self.logger.write(task_id, "go_to", "failed", message, result)
            return result

        allowed_steps = max(int(max_steps), 1)
        planned_waypoint_count = len(plan.waypoints)
        executed_waypoints = plan.waypoints[:allowed_steps]
        truncated_by_max_steps = planned_waypoint_count > allowed_steps

        for waypoint_index, waypoint in enumerate(executed_waypoints):
            pose = self.bridge.get_pose()
            dx = waypoint.x - pose["x"]
            dy = waypoint.y - pose["y"]
            remaining = hypot(dx, dy)
            waypoint_tolerance = tolerance_m if waypoint == plan.waypoints[-1] else min(tolerance_m, waypoint_spacing_m / 2.0)
            if remaining <= waypoint_tolerance:
                steps.append(self._skipped_waypoint_step(waypoint_index + 1, waypoint, pose, remaining))
                continue

            target_yaw = atan2(dy, dx)
            yaw_error = self._normalize_angle(target_yaw - pose["yaw"])
            turn_result = self.bridge.turn_relative(angle_rad=yaw_error, angular_speed_rps=angular_speed_rps)
            step = {
                "step": waypoint_index + 1,
                "waypoint": {
                    "x": float(waypoint.x),
                    "y": float(waypoint.y),
                    "tolerance_m": float(waypoint_tolerance),
                },
                "start_pose": pose,
                "target_yaw": float(target_yaw),
                "yaw_error": float(yaw_error),
                "remaining_m": float(remaining),
                "turn": turn_result,
            }

            if not turn_result.get("success", False):
                message = turn_result.get("message", "go_to waypoint turn failed")
                steps.append(step)
                break

            forward_result = self.bridge.drive_straight(distance_m=remaining, speed_mps=speed_mps)
            step["forward"] = forward_result
            steps.append(step)

            if not forward_result.get("success", False):
                message = forward_result.get("message", "go_to waypoint forward failed")
                break

        final_pose_dict = self.bridge.get_pose()
        final_distance = hypot(target_x - final_pose_dict["x"], target_y - final_pose_dict["y"])
        if final_distance <= tolerance_m:
            success = True
            message = "target position reached"
        elif truncated_by_max_steps:
            message = f"go_to stopped after max_steps={allowed_steps} before reaching target"

        self.status.state = "idle"
        self.status.last_message = message
        self.status.pose = RobotPose(
            x=final_pose_dict["x"],
            y=final_pose_dict["y"],
            yaw=final_pose_dict["yaw"],
        )
        result = {
            "task_id": task_id,
            "success": success,
            "target": {
                "x": target_x,
                "y": target_y,
                "tolerance_m": float(tolerance_m),
            },
            "planner_request": planner,
            "fallback_from": fallback_from,
            "fallback_result": fallback_result,
            "plan": plan.to_dict(),
            "progress": {
                "planned_waypoints": planned_waypoint_count,
                "executed_waypoints": len(steps),
                "max_steps": allowed_steps,
                "truncated_by_max_steps": truncated_by_max_steps,
            },
            "final_position": asdict(self.status.pose),
            "final_distance_m": float(final_distance),
            "steps": steps,
            "message": message,
        }
        self.logger.write(
            task_id,
            "go_to",
            "success" if success else "failed",
            message,
            result,
        )
        return result

    def _go_to_with_nav2(
        self,
        task_id: str,
        target_x: float,
        target_y: float,
        tolerance_m: float,
        timeout_sec: float,
        requested_planner: str,
        record_result: bool = True,
    ) -> dict[str, Any]:
        start_pose = self.bridge.get_pose()
        target_yaw = atan2(target_y - start_pose["y"], target_x - start_pose["x"])
        nav2_result = self.nav2.navigate_to_pose(
            x=target_x,
            y=target_y,
            yaw=target_yaw,
            wait_timeout_sec=1.0,
            result_timeout_sec=timeout_sec,
        )
        final_pose_dict = self.bridge.get_pose()
        final_distance = hypot(target_x - final_pose_dict["x"], target_y - final_pose_dict["y"])
        success = bool(nav2_result.get("success", False)) or final_distance <= tolerance_m
        message = "target position reached" if success else nav2_result.get("message", "Nav2 navigation failed")

        self.status.state = "idle"
        self.status.last_message = message
        self.status.pose = RobotPose(
            x=final_pose_dict["x"],
            y=final_pose_dict["y"],
            yaw=final_pose_dict["yaw"],
        )
        result = {
            "task_id": task_id,
            "success": success,
            "recorded": False,
            "target": {
                "x": target_x,
                "y": target_y,
                "tolerance_m": float(tolerance_m),
            },
            "planner_request": requested_planner,
            "plan": {
                "planner": "nav2",
                "start": {
                    "x": start_pose["x"],
                    "y": start_pose["y"],
                },
                "goal": {
                    "x": target_x,
                    "y": target_y,
                },
                "waypoints": [],
                "path_length_m": float(hypot(target_x - start_pose["x"], target_y - start_pose["y"])),
                "metadata": {
                    "frame": self.nav2.frame_id,
                    "action_name": self.nav2.action_name,
                    "nav2": nav2_result,
                },
            },
            "final_position": asdict(self.status.pose),
            "final_distance_m": float(final_distance),
            "steps": [
                {
                    "step": 1,
                    "planner": "nav2",
                    "start_pose": start_pose,
                    "target_yaw": float(target_yaw),
                    "nav2": nav2_result,
                }
            ],
            "message": message,
        }
        if record_result:
            self._record_go_to_result(task_id, result)
        return result

    def _record_go_to_result(self, task_id: str, result: dict[str, Any]) -> None:
        result["recorded"] = True
        self.logger.write(
            task_id,
            "go_to",
            "success" if result.get("success", False) else "failed",
            str(result.get("message", "")),
            result,
        )

    def list_locations(self) -> list[dict]:
        return self.locations.list_locations()

    def get_map(self) -> dict[str, Any]:
        return self.world_map.to_dict()

    def plan_path(
        self,
        x: float,
        y: float,
        waypoint_spacing_m: float = 0.5,
        planner: str = "astar",
    ) -> dict[str, Any]:
        if waypoint_spacing_m <= 0:
            raise ValueError("waypoint_spacing_m must be positive")
        planner_name = self._normalize_planner_name(planner)
        if planner_name in {"auto", "nav2"}:
            planner_name = "astar"
        start_pose = self.bridge.get_pose()
        plan = self._make_local_plan(
            planner=planner_name,
            start_x=start_pose["x"],
            start_y=start_pose["y"],
            target_x=float(x),
            target_y=float(y),
            waypoint_spacing_m=waypoint_spacing_m,
        )
        return {
            "success": True,
            "planner_request": planner,
            "plan": plan.to_dict(),
        }

    def go_to_location(
        self,
        name: str,
        speed_mps: float = 0.2,
        angular_speed_rps: float = 0.5,
        tolerance_m: float = 0.15,
        max_steps: int = 100,
        waypoint_spacing_m: float = 0.5,
        planner: str = "astar",
        nav2_timeout_sec: float = 120.0,
    ) -> dict[str, Any]:
        location = self.locations.get(name)
        result = self.go_to(
            location.x,
            location.y,
            speed_mps,
            angular_speed_rps,
            tolerance_m,
            max_steps,
            waypoint_spacing_m,
            planner,
            nav2_timeout_sec,
        )
        result["location"] = location.to_dict()
        return result

    def patrol_once(
        self,
        forward_distance_m: float = 0.5,
        forward_speed_mps: float = 0.2,
        turn_angle_rad: float = 1.5707963267948966,
        turn_speed_rps: float = 0.5,
        pause_sec: float = 0.2,
    ) -> dict[str, Any]:
        task_id = str(uuid4())
        steps: list[dict[str, Any]] = []

        self.status.state = "executing"
        self.status.last_task_id = task_id

        step1 = self.forward(forward_distance_m, forward_speed_mps)
        steps.append(step1)
        if pause_sec > 0:
            sleep(pause_sec)
        step2 = self.turn(turn_angle_rad, turn_speed_rps)
        steps.append(step2)
        if pause_sec > 0:
            sleep(pause_sec)
        step3 = self.forward(forward_distance_m, forward_speed_mps)
        steps.append(step3)

        success = all(step.get("success", False) for step in steps)
        self.status.state = "idle"
        self.status.last_message = "patrol completed" if success else "patrol failed"
        self.status.pose = RobotPose(
            x=self.bridge.get_status()["odom"]["x"],
            y=self.bridge.get_status()["odom"]["y"],
            yaw=self.bridge.get_status()["odom"]["yaw"],
        )
        result = {
            "task_id": task_id,
            "success": success,
            "steps": steps,
            "final_position": asdict(self.status.pose),
            "message": self.status.last_message,
        }
        self.logger.write(
            task_id,
            "patrol_once",
            "success" if success else "failed",
            self.status.last_message,
            result,
        )
        return result

    def stop(self) -> dict[str, Any]:
        task_id = str(uuid4())
        self.status.state = "executing"
        self.status.last_task_id = task_id
        bridge_result = self.bridge.stop()
        self.status.state = "idle"
        self.status.last_message = bridge_result.get("message", "robot stopped")
        self.status.pose = RobotPose(
            x=self.bridge.get_status()["odom"]["x"],
            y=self.bridge.get_status()["odom"]["y"],
            yaw=self.bridge.get_status()["odom"]["yaw"],
        )
        result = {
            "task_id": task_id,
            "success": bool(bridge_result.get("success", False)),
            "bridge": bridge_result,
            "message": "robot stopped",
        }
        self.logger.write(task_id, "stop", "success", "robot stopped", result)
        return result

    def capture_image(self) -> dict[str, Any]:
        task_id = str(uuid4())
        result = {
            "task_id": task_id,
            "success": True,
            "image_uri": "sim://camera/front",
            "message": "mock image captured",
        }
        self.logger.write(task_id, "capture_image", "success", "mock image captured", result)
        return result

    def close(self) -> None:
        self.nav2.shutdown()
        self.bridge.shutdown()

    @staticmethod
    def _normalize_angle(angle_rad: float) -> float:
        return atan2(sin(angle_rad), cos(angle_rad))

    @staticmethod
    def _normalize_planner_name(planner: str) -> str:
        normalized = str(planner).strip().lower()
        if normalized not in {"auto", "nav2", "astar", "straight_line"}:
            raise ValueError("planner must be one of: auto, nav2, astar, straight_line")
        return normalized

    def _make_local_plan(
        self,
        planner: str,
        start_x: float,
        start_y: float,
        target_x: float,
        target_y: float,
        waypoint_spacing_m: float,
    ) -> PathPlan:
        if planner == "straight_line":
            return StraightLinePlanner(waypoint_spacing_m=waypoint_spacing_m).plan(
                start_x,
                start_y,
                target_x,
                target_y,
            )
        if planner == "astar":
            return AStarPlanner(
                world_map=self.world_map,
                waypoint_spacing_m=waypoint_spacing_m,
            ).plan(start_x, start_y, target_x, target_y)
        raise ValueError("local waypoint planner must be astar or straight_line")

    @staticmethod
    def _skipped_waypoint_step(
        step_index: int,
        waypoint: Waypoint,
        pose: dict[str, float],
        remaining_m: float,
    ) -> dict[str, Any]:
        return {
            "step": step_index,
            "waypoint": {
                "x": float(waypoint.x),
                "y": float(waypoint.y),
            },
            "start_pose": pose,
            "remaining_m": float(remaining_m),
            "skipped": True,
            "message": "already within waypoint tolerance",
        }
