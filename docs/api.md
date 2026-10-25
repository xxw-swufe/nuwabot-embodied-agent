# API

## `GET /healthz`

返回服务健康状态。

## `GET /robot/status`

返回当前机器人状态。

## `GET /robot/pose`

返回当前导航使用的位姿。默认位姿来源是 `/odom/ground_truth`，对应 Gazebo `world` 坐标；也可以通过环境变量 `ROBOT_ODOM_TOPIC` 切换，例如改回 `/odom`。

```json
{
  "x": 1.2,
  "y": 0.4,
  "yaw": 0.1,
  "frame_id": "world",
  "child_frame_id": "base_link",
  "source_topic": "/odom/ground_truth"
}
```

## `GET /robot/locations`

返回语义点位列表。点位优先来自 `maps/world_map.json` 的 `locations` 字段；没有该字段时回退到 `maps/locations.json`。

返回示例：

```json
{
  "items": [
    {
      "name": "entrance",
      "label": "入口",
      "x": -2.0,
      "y": 0.0
    },
    {
      "name": "exit",
      "label": "出口",
      "x": 1.5,
      "y": 1.0
    }
  ]
}
```

## `GET /robot/map`

返回当前本地地图配置摘要，包括坐标系、边界、分辨率、机器人半径和障碍物数量。

## `POST /robot/plan`

只规划路径，不执行运动。用于先检查 A* 或直线路径。

请求体：

```json
{
  "x": 1.5,
  "y": 1.0,
  "waypoint_spacing_m": 0.5,
  "planner": "astar"
}
```

`planner` 支持：

- `astar`
- `straight_line`

## `POST /robot/cmd_vel`

请求体：

```json
{
  "linear_x": 0.2,
  "angular_z": 0.0,
  "linear_y": 0.0,
  "duration_sec": 0.2
}
```

返回：

- `task_id`
- `success`
- `command`
- `final_position`
- `bridge`
- `message`

## `POST /robot/forward`

请求体：

```json
{
  "distance_m": 0.5,
  "speed_mps": 0.2
}
```

## `POST /robot/turn`

请求体：

```json
{
  "angle_rad": 1.5707963267948966,
  "angular_speed_rps": 0.5
}
```

## `POST /robot/go_to`

坐标导航到目标点。当前版本默认使用 `/odom/ground_truth` 作为当前位姿来源。`planner` 默认是 `astar`：先跑本地 A* 栅格规划，再逐个 waypoint 执行闭环转向和闭环前进。正式地图导航时，`nav2` 会把目标点按 `map` frame 发送给 Nav2；`odom` 只用于调试链路。

请求体：

```json
{
  "x": 1.0,
  "y": 2.0,
  "speed_mps": 0.2,
  "angular_speed_rps": 0.5,
  "tolerance_m": 0.15,
  "max_steps": 100,
  "waypoint_spacing_m": 0.5,
  "planner": "astar",
  "nav2_timeout_sec": 120.0
}
```

`planner` 支持：

- `auto`：先 Nav2，失败或不可用时回退 A*
- `nav2`：只用 Nav2，不回退
- `astar`：只用本地 A*
- `straight_line`：只用原直线路径点

返回重点字段：

- `success`
- `target`
- `planner_request`
- `fallback_from`
- `fallback_result`
- `plan`
- `progress`
- `final_position`
- `final_distance_m`
- `steps`
- `message`

## `POST /robot/go_to_location`

按语义点位名称导航。后端先从 `maps/world_map.json` 或 `maps/locations.json` 查出坐标，再调用 `go_to(x, y)`。

请求体：

```json
{
  "name": "exit",
  "speed_mps": 0.2,
  "angular_speed_rps": 0.5,
  "tolerance_m": 0.15,
  "max_steps": 100,
  "waypoint_spacing_m": 0.5,
  "planner": "astar",
  "nav2_timeout_sec": 120.0
}
```

`name` 支持英文 key，例如 `entrance`、`exit`，也支持中文 label，例如 `入口`、`出口`。

## 地图与障碍物

本地 A* 地图配置在 `maps/world_map.json`。当前支持矩形障碍物：

```json
{
  "type": "box",
  "center_x": -0.5,
  "center_y": 0.3,
  "size_x": 1.0,
  "size_y": 0.5
}
```

A* 会按 `robot_radius_m` 对障碍物做膨胀，避免路径贴边。

## Nav2

Nav2 action 客户端使用 `nav2_msgs/action/NavigateToPose`。默认 action 名称是 `navigate_to_pose`，正式地图导航的目标 frame 使用 `map`，调试时可切到 `odom`。

正式建图和导航应使用 `maps/nav2_params.yaml`，并由 SLAM/AMCL 发布 `map -> odom`。如果只是联调控制器和行为树，可以临时使用 `maps/nav2_params_odom.yaml`。

可用环境变量：

```bash
export ROBOT_NAV2_ACTION=navigate_to_pose
export ROBOT_NAV2_FRAME=map
```

当前如果没有启动 Nav2 节点，`planner=auto` 会回退到 `astar`；`planner=nav2` 会直接返回 Nav2 不可用。

## `POST /robot/patrol_once`

请求体：

```json
{
  "forward_distance_m": 0.5,
  "forward_speed_mps": 0.2,
  "turn_angle_rad": 1.5707963267948966,
  "turn_speed_rps": 0.5,
  "pause_sec": 0.2
}
```

## `POST /robot/stop`

停止机器人。

## `POST /robot/capture_image`

采集图像，返回一个仿真图像 URI。

## `GET /robot/task_log`

返回任务日志列表。
