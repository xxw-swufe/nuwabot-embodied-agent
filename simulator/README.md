# Simulator Notes

这里先放仿真适配层，不强依赖具体平台。

第一阶段建议先把接口抽象成：

- `go_to(x, y)`
- `stop()`
- `get_pose()`
- `capture_image()`

后续可以接：

- MuJoCo
- Isaac Sim
- Gazebo
- Webots

