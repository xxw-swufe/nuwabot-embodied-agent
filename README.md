# Nuwabot Sim Agent Hub

基于女娲 OS Agent 的仿真机器狗任务执行系统。

这个仓库当前定位很明确：

1. 让 Agent 理解自然语言任务。
2. 让 Agent 调用仿真机器狗技能。
3. 让系统记录执行过程、失败信息和结果。
4. 让系统自动生成任务报告。

现在先做的是最小可运行闭环，不碰真机底层控制。

## 当前结构

- `backend/robot_service/`：机器人任务执行 API
- `simulator/`：仿真侧示例和后续接入点
- `skills/`：给女娲 OS 用的技能说明
- `workflows/`：任务编排示例
- `docs/`：架构、API、路线图
- `reports/`：任务报告样例
- `scripts/`：本地联调和接口测试脚本

## 第一版目标

先跑通这条链路：

自然语言任务 -> API 调用 -> 仿真动作 -> 状态反馈 -> 日志 -> 报告

## 本地启动

```bash
# 终端 1: 启动仿真
ros2 launch go2_config gazebo.launch.py rviz:=true

# 终端 2: 启动桥接服务
cd backend/robot_service
uvicorn main:app --reload --host 0.0.0.0 --port 8000
```

导航默认使用 `/odom/ground_truth`，这样 API 坐标和 Gazebo `world` 坐标一致。需要切回里程计坐标时可以启动前设置：

```bash
export ROBOT_ODOM_TOPIC=/odom
```

默认导航策略是 `planner=auto`：

1. 如果 Nav2 的 `NavigateToPose` action server 在线，优先把目标点交给 Nav2。
2. 如果 Nav2 不在线，自动回退到本地 `AStarPlanner`。
3. 如果显式传 `planner=nav2`，则只走 Nav2，不回退。

本地 A* 地图在 `maps/world_map.json`，入口/出口点位和静态障碍物都可以放在这个文件里。

Nav2 保留两套参数：

- `maps/nav2_params_odom.yaml`：无 `map -> odom` 时验证 Nav2 是否能跑通。
- `maps/nav2_params.yaml`：正式 MAP 模式，用于后续 SLAM/AMCL 建图定位后的导航。

## 接口优先级

第一批只保留最必要的接口：

- `GET /healthz`
- `GET /robot/status`
- `POST /robot/cmd_vel`
- `POST /robot/stop`
- `POST /robot/capture_image`
- `GET /robot/task_log`

## 下一步

把 `RobotController` 从当前的桥接实现，继续扩展成任务执行层，比如巡航、定点转向、自动停靠和报告生成。
