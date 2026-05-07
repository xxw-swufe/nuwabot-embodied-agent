from __future__ import annotations

import heapq
import json
from dataclasses import asdict, dataclass, field
from math import ceil, hypot, sqrt
from pathlib import Path
from typing import Any


DEFAULT_WORLD_MAP_PATH = Path(__file__).resolve().parents[2] / "maps" / "world_map.json"


@dataclass(frozen=True)
class Waypoint:
    x: float
    y: float


@dataclass(frozen=True)
class PathPlan:
    planner: str
    start: Waypoint
    goal: Waypoint
    waypoints: list[Waypoint]
    path_length_m: float
    metadata: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict:
        return {
            "planner": self.planner,
            "start": asdict(self.start),
            "goal": asdict(self.goal),
            "waypoints": [asdict(waypoint) for waypoint in self.waypoints],
            "path_length_m": self.path_length_m,
            "metadata": self.metadata,
        }


@dataclass(frozen=True)
class BoxObstacle:
    center_x: float
    center_y: float
    size_x: float
    size_y: float

    def contains(self, x: float, y: float, inflation_m: float = 0.0) -> bool:
        half_x = self.size_x / 2.0 + inflation_m
        half_y = self.size_y / 2.0 + inflation_m
        return abs(x - self.center_x) <= half_x and abs(y - self.center_y) <= half_y


@dataclass(frozen=True)
class WorldMap:
    frame: str
    resolution_m: float
    min_x: float
    max_x: float
    min_y: float
    max_y: float
    robot_radius_m: float
    obstacles: list[BoxObstacle]

    @classmethod
    def from_file(cls, path: Path | str = DEFAULT_WORLD_MAP_PATH) -> "WorldMap":
        with Path(path).open("r", encoding="utf-8") as f:
            data = json.load(f)

        bounds = data["bounds"]
        obstacles = [
            BoxObstacle(
                center_x=float(obstacle["center_x"]),
                center_y=float(obstacle["center_y"]),
                size_x=float(obstacle["size_x"]),
                size_y=float(obstacle["size_y"]),
            )
            for obstacle in data.get("obstacles", [])
            if obstacle.get("type", "box") == "box"
        ]

        return cls(
            frame=str(data.get("frame", "world")),
            resolution_m=float(data.get("resolution_m", 0.25)),
            min_x=float(bounds["min_x"]),
            max_x=float(bounds["max_x"]),
            min_y=float(bounds["min_y"]),
            max_y=float(bounds["max_y"]),
            robot_radius_m=float(data.get("robot_radius_m", 0.35)),
            obstacles=obstacles,
        )

    @property
    def width(self) -> int:
        return int(ceil((self.max_x - self.min_x) / self.resolution_m)) + 1

    @property
    def height(self) -> int:
        return int(ceil((self.max_y - self.min_y) / self.resolution_m)) + 1

    def world_to_grid(self, x: float, y: float) -> tuple[int, int]:
        gx = int(round((x - self.min_x) / self.resolution_m))
        gy = int(round((y - self.min_y) / self.resolution_m))
        return gx, gy

    def grid_to_world(self, gx: int, gy: int) -> Waypoint:
        x = self.min_x + gx * self.resolution_m
        y = self.min_y + gy * self.resolution_m
        return Waypoint(float(x), float(y))

    def in_bounds(self, cell: tuple[int, int]) -> bool:
        gx, gy = cell
        return 0 <= gx < self.width and 0 <= gy < self.height

    def is_blocked_world(self, x: float, y: float) -> bool:
        if x < self.min_x or x > self.max_x or y < self.min_y or y > self.max_y:
            return True
        return any(obstacle.contains(x, y, self.robot_radius_m) for obstacle in self.obstacles)

    def is_blocked_cell(self, cell: tuple[int, int]) -> bool:
        waypoint = self.grid_to_world(*cell)
        return self.is_blocked_world(waypoint.x, waypoint.y)

    def to_dict(self) -> dict[str, Any]:
        return {
            "frame": self.frame,
            "resolution_m": self.resolution_m,
            "bounds": {
                "min_x": self.min_x,
                "max_x": self.max_x,
                "min_y": self.min_y,
                "max_y": self.max_y,
            },
            "robot_radius_m": self.robot_radius_m,
            "obstacle_count": len(self.obstacles),
        }


class StraightLinePlanner:
    def __init__(self, waypoint_spacing_m: float = 0.5) -> None:
        if waypoint_spacing_m <= 0:
            raise ValueError("waypoint_spacing_m must be positive")
        self.waypoint_spacing_m = float(waypoint_spacing_m)

    def plan(self, start_x: float, start_y: float, goal_x: float, goal_y: float) -> PathPlan:
        start = Waypoint(float(start_x), float(start_y))
        goal = Waypoint(float(goal_x), float(goal_y))
        distance = hypot(goal.x - start.x, goal.y - start.y)

        if distance == 0:
            waypoints = [goal]
        else:
            segments = max(1, int(ceil(distance / self.waypoint_spacing_m)))
            waypoints = [
                Waypoint(
                    x=start.x + (goal.x - start.x) * step / segments,
                    y=start.y + (goal.y - start.y) * step / segments,
                )
                for step in range(1, segments + 1)
            ]

        return PathPlan(
            planner="straight_line",
            start=start,
            goal=goal,
            waypoints=waypoints,
            path_length_m=float(distance),
            metadata={"waypoint_spacing_m": self.waypoint_spacing_m},
        )


class AStarPlanner:
    _NEIGHBORS = (
        (-1, -1, sqrt(2.0)),
        (-1, 0, 1.0),
        (-1, 1, sqrt(2.0)),
        (0, -1, 1.0),
        (0, 1, 1.0),
        (1, -1, sqrt(2.0)),
        (1, 0, 1.0),
        (1, 1, sqrt(2.0)),
    )

    def __init__(
        self,
        world_map: WorldMap | None = None,
        waypoint_spacing_m: float = 0.5,
        simplify_path: bool = True,
    ) -> None:
        if waypoint_spacing_m <= 0:
            raise ValueError("waypoint_spacing_m must be positive")
        self.world_map = world_map or WorldMap.from_file()
        self.waypoint_spacing_m = float(waypoint_spacing_m)
        self.simplify_path = bool(simplify_path)

    def plan(self, start_x: float, start_y: float, goal_x: float, goal_y: float) -> PathPlan:
        start = Waypoint(float(start_x), float(start_y))
        goal = Waypoint(float(goal_x), float(goal_y))
        start_cell = self.world_map.world_to_grid(start.x, start.y)
        goal_cell = self.world_map.world_to_grid(goal.x, goal.y)

        if not self.world_map.in_bounds(start_cell):
            raise ValueError(f"start position is outside map bounds: {start}")
        if not self.world_map.in_bounds(goal_cell):
            raise ValueError(f"goal position is outside map bounds: {goal}")
        if self.world_map.is_blocked_cell(start_cell):
            raise ValueError(f"start position is inside an obstacle or inflated obstacle: {start}")
        if self.world_map.is_blocked_cell(goal_cell):
            raise ValueError(f"goal position is inside an obstacle or inflated obstacle: {goal}")

        grid_path = self._search(start_cell, goal_cell)
        if not grid_path:
            raise ValueError("A* could not find a path to target")

        raw_waypoints = [self.world_map.grid_to_world(gx, gy) for gx, gy in grid_path[1:]]
        if not raw_waypoints:
            raw_waypoints = [goal]

        waypoints = self._sample_waypoints(raw_waypoints)
        if not waypoints or hypot(waypoints[-1].x - goal.x, waypoints[-1].y - goal.y) > self.world_map.resolution_m:
            waypoints.append(goal)
        else:
            waypoints[-1] = goal

        return PathPlan(
            planner="astar",
            start=start,
            goal=goal,
            waypoints=waypoints,
            path_length_m=self._path_length([start, *waypoints]),
            metadata={
                "frame": self.world_map.frame,
                "map": self.world_map.to_dict(),
                "grid_start": {"x": start_cell[0], "y": start_cell[1]},
                "grid_goal": {"x": goal_cell[0], "y": goal_cell[1]},
                "grid_path_cells": len(grid_path),
                "raw_waypoints": len(raw_waypoints),
                "waypoint_spacing_m": self.waypoint_spacing_m,
            },
        )

    def _search(self, start: tuple[int, int], goal: tuple[int, int]) -> list[tuple[int, int]]:
        frontier: list[tuple[float, tuple[int, int]]] = []
        heapq.heappush(frontier, (0.0, start))
        came_from: dict[tuple[int, int], tuple[int, int] | None] = {start: None}
        cost_so_far: dict[tuple[int, int], float] = {start: 0.0}

        while frontier:
            _, current = heapq.heappop(frontier)
            if current == goal:
                return self._reconstruct_path(came_from, current)

            for dx, dy, step_cost in self._NEIGHBORS:
                neighbor = (current[0] + dx, current[1] + dy)
                if not self.world_map.in_bounds(neighbor) or self.world_map.is_blocked_cell(neighbor):
                    continue
                if dx != 0 and dy != 0:
                    side_a = (current[0] + dx, current[1])
                    side_b = (current[0], current[1] + dy)
                    if self.world_map.is_blocked_cell(side_a) or self.world_map.is_blocked_cell(side_b):
                        continue

                new_cost = cost_so_far[current] + step_cost
                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    priority = new_cost + self._heuristic(neighbor, goal)
                    heapq.heappush(frontier, (priority, neighbor))
                    came_from[neighbor] = current

        return []

    @staticmethod
    def _heuristic(a: tuple[int, int], b: tuple[int, int]) -> float:
        return hypot(a[0] - b[0], a[1] - b[1])

    @staticmethod
    def _reconstruct_path(
        came_from: dict[tuple[int, int], tuple[int, int] | None],
        current: tuple[int, int],
    ) -> list[tuple[int, int]]:
        path = [current]
        while came_from[current] is not None:
            current = came_from[current]  # type: ignore[assignment]
            path.append(current)
        path.reverse()
        return path

    def _sample_waypoints(self, raw_waypoints: list[Waypoint]) -> list[Waypoint]:
        if not raw_waypoints:
            return []
        if not self.simplify_path:
            return raw_waypoints

        sampled: list[Waypoint] = []
        last = raw_waypoints[0]
        sampled.append(last)
        for waypoint in raw_waypoints[1:]:
            if hypot(waypoint.x - last.x, waypoint.y - last.y) >= self.waypoint_spacing_m:
                sampled.append(waypoint)
                last = waypoint
        if sampled[-1] != raw_waypoints[-1]:
            sampled.append(raw_waypoints[-1])
        return sampled

    @staticmethod
    def _path_length(points: list[Waypoint]) -> float:
        return float(sum(hypot(b.x - a.x, b.y - a.y) for a, b in zip(points, points[1:])))
