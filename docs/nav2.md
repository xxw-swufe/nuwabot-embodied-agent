# Nav2 Bringup

当前后端已经接入 `nav2_msgs/action/NavigateToPose`。

## 当前状态

本机已安装 Nav2 包，但当前仿真里还没有启动 Nav2 节点。`planner=auto` 会先尝试 Nav2；如果 `navigate_to_pose` action server 不在线，会自动回退到本地 A*。

## 两种模式

当前保留两套 Nav2 参数：

- `maps/nav2_params_odom.yaml`：WORLD 验证模式。不需要 `map -> odom`，适合确认 Nav2 controller/planner 能启动，并且和 `/odom/ground_truth` 的 `world` 坐标对齐。
- `maps/nav2_params.yaml`：MAP 正式模式。用于建图、定位和正式导航，需要系统发布 `map -> odom -> base_link` TF。

建图时不是“用 ODOM 模式建图”。正确关系是：

```text
odom: 机器人局部里程计坐标
world: 仿真真值坐标，通常由 `/odom/ground_truth` 提供
map: 建图/定位后的全局地图坐标
SLAM 或 AMCL: 发布 map -> odom
Nav2 WORLD 验证: 目标点使用 world frame
Nav2 正式导航: 目标点使用 map frame
```

## 启动 WORLD 验证模式

先启动 Gazebo 和机器狗控制：

```bash
ros2 launch go2_config gazebo.launch.py rviz:=true
```

再启动 Nav2：

```bash
ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=true \
  params_file:=/home/xx/nuwabot-sim-agent/maps/nav2_params_odom.yaml
```

最后重启 API 服务：

```bash
cd /home/xx/nuwabot-sim-agent/backend/robot_service
export ROBOT_NAV2_FRAME=world
uvicorn main:app --reload --host 0.0.0.0 --port 8000
```

## 启动 MAP 正式模式

先启动 SLAM/AMCL 或其它定位模块，确认已经有 `map -> odom -> base_link` TF。然后启动：

```bash
ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=true \
  params_file:=/home/xx/nuwabot-sim-agent/maps/nav2_params.yaml
```

API 服务使用默认 `map` frame：

```bash
cd /home/xx/nuwabot-sim-agent/backend/robot_service
export ROBOT_NAV2_FRAME=map
uvicorn main:app --reload --host 0.0.0.0 --port 8000
```

## 验证

查看 Nav2 节点：

```bash
ros2 node list | grep -E 'bt_navigator|planner_server|controller_server'
```

调用 API 时显式要求 Nav2：

```bash
curl -X POST "https://appraiser-tactics-racing.ngrok-free.dev/robot/go_to_location" \
  -H "Content-Type: application/json" \
  -d '{"name":"exit","planner":"nav2","tolerance_m":0.25,"nav2_timeout_sec":120}'
```

如果要允许自动回退：

```bash
curl -X POST "https://appraiser-tactics-racing.ngrok-free.dev/robot/go_to_location" \
  -H "Content-Type: application/json" \
  -d '{"name":"exit","planner":"auto","tolerance_m":0.25}'
```

## 注意

当前如果没有 `map` frame，就用 WORLD 验证模式；要建图和正式导航，保留 MAP 模式，并让 SLAM/AMCL 发布 `map -> odom`。

## 当前简单场景

当前推荐的验证场景是一个简单房间：

- 入口：`(-2.6, 0.0)`
- 出口：`(2.6, 0.0)`
- 中间墙：`x = 0.0`，用于验证 Nav2 是否能绕障碍到达目标
