# Nav2 Bringup

当前后端已经接入 `nav2_msgs/action/NavigateToPose`。

## 当前状态

本机已安装 Nav2 包，但当前仿真里还没有启动 Nav2 节点。`planner=auto` 会先尝试 Nav2；如果 `navigate_to_pose` action server 不在线，会自动回退到本地 A*。

## 两种模式

当前保留两套 Nav2 参数：

- `maps/nav2_params.yaml`：MAP 正式模式。用于建图、定位和正式导航，需要系统发布 `map -> odom -> base_link` TF。
- `maps/nav2_params_odom.yaml`：调试用纯 `odom` 验证模式。适合确认控制器和规划器能否启动。

建图时不是“用 ODOM 模式建图”。正确关系是：

```text
odom: 机器人局部里程计坐标
world: 仿真真值坐标，通常由 `/odom/ground_truth` 提供
map: 建图/定位后的全局地图坐标
SLAM 或 AMCL: 发布 map -> odom
Nav2 正式导航: 目标点使用 map frame
Nav2 `odom` 验证: 只用于调试，不作为最终巡查方案
```

## 启动 `map` 正式模式

先启动 Gazebo 和机器狗控制：

```bash
ros2 launch go2_config gazebo.launch.py rviz:=true
```

再启动 Nav2：

```bash
ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=true \
  params_file:=/home/xx/nuwabot-sim-agent/maps/nav2_params.yaml
```

最后重启 API 服务：

```bash
cd /home/xx/nuwabot-sim-agent/backend/robot_service
export ROBOT_NAV2_FRAME=map
uvicorn main:app --reload --host 0.0.0.0 --port 8000
```

## 启动 `odom` 调试模式

先启动 SLAM/AMCL 或其它定位模块，确认已经有 `map -> odom -> base_link` TF。然后启动：

```bash
ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=true \
  params_file:=/home/xx/nuwabot-sim-agent/maps/nav2_params_odom.yaml
```

API 服务使用调试 frame：

```bash
cd /home/xx/nuwabot-sim-agent/backend/robot_service
export ROBOT_NAV2_FRAME=odom
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

正式巡查和语义导航应使用 `map` 模式，并让 SLAM/AMCL 发布 `map -> odom`。`odom` 模式只用于联调和排障。

## 当前简单场景

当前推荐的验证场景是一个简单房间：

- 入口：`(-2.6, 0.0)`
- 出口：`(3.0, 0.0)`
- 中间板：`x = 0.0`，长度 `1m`，用于验证 Nav2 是否能绕障碍到达目标
