from __future__ import annotations

import atexit

from fastapi import FastAPI
from pydantic import BaseModel, Field

try:
    from .robot_controller import RobotController
except ImportError:  # pragma: no cover - fallback for direct uvicorn usage
    from robot_controller import RobotController


app = FastAPI(title="Nuwabot Sim Agent Robot Service", version="0.1.0")
controller = RobotController()
atexit.register(controller.close)


class TwistRequest(BaseModel):
    linear_x: float = Field(..., description="Forward velocity in m/s")
    angular_z: float = Field(..., description="Yaw rate in rad/s")
    linear_y: float = Field(0.0, description="Lateral velocity in m/s")
    duration_sec: float = Field(0.2, description="How long to publish the twist command")


class ForwardRequest(BaseModel):
    distance_m: float = Field(..., description="Distance to move in meters")
    speed_mps: float = Field(0.2, description="Forward speed in meters per second")


class TurnRequest(BaseModel):
    angle_rad: float = Field(..., description="Yaw rotation angle in radians")
    angular_speed_rps: float = Field(0.5, description="Angular speed in radians per second")


class GoToRequest(BaseModel):
    x: float = Field(..., description="Target x position in odom/map frame")
    y: float = Field(..., description="Target y position in odom/map frame")
    speed_mps: float = Field(0.2, description="Forward speed in meters per second")
    angular_speed_rps: float = Field(0.5, description="Angular speed in radians per second")
    tolerance_m: float = Field(0.15, description="Accepted distance to target in meters")
    max_steps: int = Field(100, description="Maximum turn-and-forward correction steps")
    waypoint_spacing_m: float = Field(0.5, description="Distance between planned waypoints")
    planner: str = Field("auto", description="Planner: auto, nav2, astar, or straight_line")
    nav2_timeout_sec: float = Field(120.0, description="Maximum time to wait for Nav2 navigation")


class GoToLocationRequest(BaseModel):
    name: str = Field(..., description="Named location, for example entrance or exit")
    speed_mps: float = Field(0.2, description="Forward speed in meters per second")
    angular_speed_rps: float = Field(0.5, description="Angular speed in radians per second")
    tolerance_m: float = Field(0.15, description="Accepted distance to target in meters")
    max_steps: int = Field(100, description="Maximum waypoint correction steps")
    waypoint_spacing_m: float = Field(0.5, description="Distance between planned waypoints")
    planner: str = Field("auto", description="Planner: auto, nav2, astar, or straight_line")
    nav2_timeout_sec: float = Field(120.0, description="Maximum time to wait for Nav2 navigation")


class PlanPathRequest(BaseModel):
    x: float = Field(..., description="Target x position in world/map frame")
    y: float = Field(..., description="Target y position in world/map frame")
    waypoint_spacing_m: float = Field(0.5, description="Distance between returned waypoints")
    planner: str = Field("astar", description="Planner: astar or straight_line")


class PatrolRequest(BaseModel):
    forward_distance_m: float = Field(0.5, description="Distance for each forward segment")
    forward_speed_mps: float = Field(0.2, description="Forward speed in meters per second")
    turn_angle_rad: float = Field(1.5707963267948966, description="Turn angle in radians")
    turn_speed_rps: float = Field(0.5, description="Turn speed in radians per second")
    pause_sec: float = Field(0.2, description="Pause between motion steps")


@app.get("/healthz")
def healthz() -> dict[str, str]:
    return {"status": "ok"}


@app.get("/robot/status")
def robot_status() -> dict:
    return controller.get_status()


@app.get("/robot/pose")
def robot_pose() -> dict:
    return controller.bridge.get_pose()


@app.get("/robot/locations")
def robot_locations() -> dict:
    return {"items": controller.list_locations()}


@app.get("/robot/map")
def robot_map() -> dict:
    return controller.get_map()


@app.post("/robot/plan")
def robot_plan(req: PlanPathRequest) -> dict:
    return controller.plan_path(req.x, req.y, req.waypoint_spacing_m, req.planner)


@app.post("/robot/cmd_vel")
def robot_cmd_vel(req: TwistRequest) -> dict:
    return controller.send_twist(req.linear_x, req.angular_z, req.duration_sec, req.linear_y)


@app.post("/robot/forward")
def robot_forward(req: ForwardRequest) -> dict:
    return controller.forward(req.distance_m, req.speed_mps)


@app.post("/robot/turn")
def robot_turn(req: TurnRequest) -> dict:
    return controller.turn(req.angle_rad, req.angular_speed_rps)


@app.post("/robot/go_to")
def robot_go_to(req: GoToRequest) -> dict:
    return controller.go_to(
        req.x,
        req.y,
        req.speed_mps,
        req.angular_speed_rps,
        req.tolerance_m,
        req.max_steps,
        req.waypoint_spacing_m,
        req.planner,
        req.nav2_timeout_sec,
    )


@app.post("/robot/go_to_location")
def robot_go_to_location(req: GoToLocationRequest) -> dict:
    return controller.go_to_location(
        req.name,
        req.speed_mps,
        req.angular_speed_rps,
        req.tolerance_m,
        req.max_steps,
        req.waypoint_spacing_m,
        req.planner,
        req.nav2_timeout_sec,
    )


@app.post("/robot/patrol_once")
def robot_patrol_once(req: PatrolRequest) -> dict:
    return controller.patrol_once(
        req.forward_distance_m,
        req.forward_speed_mps,
        req.turn_angle_rad,
        req.turn_speed_rps,
        req.pause_sec,
    )


@app.post("/robot/stop")
def robot_stop() -> dict:
    return controller.stop()


@app.post("/robot/capture_image")
def robot_capture_image() -> dict:
    return controller.capture_image()


@app.get("/robot/task_log")
def robot_task_log() -> dict:
    return {"items": controller.logger.read_all()}
