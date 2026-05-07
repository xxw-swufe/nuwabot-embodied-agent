# Robot Skills

这些技能给女娲 OS Agent 调用。

## `robot_go_to`

控制仿真机器狗移动到指定坐标。

输入：

- `x`: 目标 x 坐标
- `y`: 目标 y 坐标
- `planner`: 可选，`auto`、`nav2`、`astar` 或 `straight_line`，默认 `auto`

输出：

- `success`
- `task_id`
- `planner_request`
- `plan`
- `final_position`
- `distance`
- `message`

默认 `planner=auto`：优先尝试 Nav2；如果 Nav2 不在线，后端会自动回退到本地 A* 地图规划。

## `robot_stop`

停止当前动作。

## `robot_capture_image`

采集仿真相机图像，供后续视觉分析。
