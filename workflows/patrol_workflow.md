# Patrol Workflow

目标：让 Agent 依次执行巡检点位，并记录结果。

## 流程

1. 接收用户任务。
2. 解析巡检点列表。
3. 依次调用 `robot_go_to`。
4. 到点后调用 `robot_capture_image`。
5. 对图像做检测或描述。
6. 记录每一步结果。
7. 生成最终报告。

## 状态

- `PENDING`
- `PLANNING`
- `EXECUTING`
- `OBSERVING`
- `VERIFYING`
- `COMPLETED`
- `FAILED`

