# Gazebo 单机无人机仿真与控制验证平台

基于 ROS 2 Humble、PX4 SITL 和 Gazebo Harmonic，使用余老师的
`QuadrotorControllerGanYu` 完成单机键盘飞行控制，支持相机图像与 MID-360 仿真点云显示。
支持键盘飞行，以及 **给定路点 → 三维 A* 避障搜索 → 七次 B 样条 → GanYu 跟踪 → 降落**。
规划器使用 Walls 场景的静态碰撞地图；MID-360 点云用于显示，尚未接入雷达建图或动态避障。

## 航点规划与避障演示

```bash
./scripts/build_planner.sh
```

两个终端分别运行：

```bash
# 终端 1：等待 READY
./run_obstacle_scene.sh
```

```bash
# 终端 2：自动起飞、绕墙、依次到达路点、降落
./run_ganyu_planner.sh --start --rviz
```

不加 `--start` 只显示规划，不解锁。MID-360 机型两条启动命令都加 `--mid360`。
RViz 用橙色显示 A*、青色显示 B 样条、绿色显示实际飞行轨迹，并显示墙体与安全膨胀区域。
默认路点要求绕过两面墙；修改 [waypoints.yaml](uav_path_planner/config/waypoints.yaml) 可指定自己的 ENU 路点。
`Ctrl+C` 会先请求降落再退出。不要同时运行键盘控制入口。
详细原理、约束、日志与话题见 [规划包说明](uav_path_planner/README.md)。

## 编译与准备

在项目目录执行：

```bash
./scripts/build_planner.sh
source install/setup.bash
```

脚本使用独立英文构建缓存，兼容 ROS Humble 消息生成器与当前中文工作空间路径。

使用 MID-360 模型前，首次安装 RGL 依赖；本机已安装，重复执行会校验已有安装：

```bash
./scripts/setup_mid360_sim.sh
```

该雷达后端需要兼容的 NVIDIA RTX 显卡和驱动。详细依赖见
[可视化功能包说明](uav_sim_visualization/README.md)。

<a id="ganyu-quickstart"></a>

## 启动仿真环境

仿真环境统一使用脚本启动。终端 1 运行相机 + MID-360 模型：

```bash
./run_obstacle_scene.sh --mid360
```

等待 `READY` 后保持终端运行。脚本已启动 Gazebo、PX4、Agent、时钟及传感器桥接，
此时飞机未解锁。PX4 命名空间为 `/px4_1`，目标系统 ID 为 `2`。

只使用原相机模型时，将上面的命令替换为：

```bash
./run_obstacle_scene.sh
```

需要无界面运行时追加 `--headless`。两种模型的控制器质量参数不同，见下文。

<a id="manual-start"></a>

## 手动启动重要 ROS 节点

**下面每个新终端都先加载环境：**

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

使用下列手动方式时，不要再同时运行 `run_ganyu_keyboard.sh`，同一架飞机只启动一套控制器和键盘。

<a id="ganyu-controller"></a>

### 1. GanYu 控制器

终端 2：针对相机 + MID-360 模型启动控制节点：

```bash
ros2 run uav_ganyu_control ganyu_controller --ros-args \
  -p px4_namespace:=/px4_1 -p target_system:=2 \
  -p mass:=2.390307692307692
```

节点读取 PX4 里程计，运行 GanYu 位置闭环，向 PX4 发布角速度与推力设定值。
启动节点不会自动解锁，需等待键盘指令。

若终端 1 使用原相机模型，将 `mass` 改为 `2.125307692307692`。
可在同一命令末尾追加 `-p takeoff_height:=2.0 -p keyboard_step:=0.1`，
分别设置起飞高度（m）与单次移动步长（m）；默认值为 1.5 和 0.25。

### 2. 键盘客户端

终端 3：

```bash
ros2 run uav_ganyu_control ganyu_keyboard
```

保持键盘终端获得焦点，显示“就绪”后按 `t`，无需回车。
客户端默认提供本机仿真地面站心跳；使用外部地面站时，可替换为：

```bash
ros2 run uav_ganyu_control ganyu_keyboard --external-gcs
```

<a id="ganyu-keys"></a>

默认按键：

- `t`：起飞至相对高度 1.5 m。
- `w / s`：北 / 南；`a / d`：西 / 东，每次调整目标 0.25 m。
- `e / q`：上升 / 下降，每次调整目标 0.25 m。
- `空格`：将当前位置设为悬停目标，并平滑制动。
- `l`：请求降落，落地上锁后保留键盘会话。
- `x` / `Ctrl+C`：请求降落，确认未解锁后退出键盘。

方向使用固定世界坐标，不随机头转动。松开按键后保持最后目标；
水平目标范围为起飞点各方向 ±3 m，高度范围为 0.5～3 m。键盘模式没有自动避障；
自动航点避障使用上面的规划入口。

<a id="rviz-sensors"></a>

### 3. RViz2 相机与点云显示

终端 4：

```bash
ros2 launch uav_sim_visualization sensors.launch.py
```

该命令启动可视化 TF 节点和 RViz2，加载图像与点云显示配置：

- 相机图像：`/camera/image`；相机内参：`/camera/camera_info`。
- 雷达点云：`/lidar/points`，仅 `--mid360` 场景提供。
- 固定坐标系：`odom`；TF 链为 `odom → base_link → mid360_link`。
- 图像、点云订阅使用 Best Effort，显示节点使用仿真时间。

只需要 TF、不打开 RViz2 时，将上述命令替换为：

```bash
ros2 run uav_sim_visualization sensor_frames --ros-args \
  -p use_sim_time:=true -p px4_namespace:=/px4_1
```

TF 节点仅用于显示，不发送飞行控制指令。修改布局可编辑
[sensors.rviz](uav_sim_visualization/config/sensors.rviz)。

<a id="ganyu-topics"></a>

## 常用话题与检查

- `/ganyu/status`：控制阶段、就绪状态、位置和目标，JSON 格式。
- `/ganyu/control_debug`：推力、角速度、参考轨迹及反馈调试数据。
- `/px4_1/fmu/out/vehicle_odometry`：PX4 位置、速度和姿态反馈。
- `/px4_1/fmu/out/vehicle_status_v1`：解锁及飞行模式状态。
- `/clock`：仿真时钟。

加载环境后，可在另一个终端查看：

```bash
ros2 node list
ros2 topic echo /ganyu/status --once
ros2 topic echo /px4_1/fmu/out/vehicle_odometry \
  px4_msgs/msg/VehicleOdometry --qos-reliability best_effort --once
ros2 topic hz /camera/image
ros2 topic hz /lidar/points
```

最后一条仅用于 MID-360 场景。没有图像或点云时，先确认场景已输出 `READY`；
RViz2 提示缺少 TF 时，确认显示节点、PX4 里程计和 `/clock` 正在发布。
场景日志保存在 `log/obstacle_scene/latest/`，手动启动的 ROS 节点日志显示在各自终端。

<a id="ganyu-land-policy"></a>

## 退出顺序

1. 在键盘终端按 `x` 或 `Ctrl+C`，等待落地上锁并退出键盘。
2. 确认 `IDLE` 且未解锁后，在控制器终端按 `Ctrl+C`。
3. 结束 RViz2，最后在场景终端按 `Ctrl+C` 关闭本轮仿真。

手动启动时，退出键盘不会自动结束另一个终端中的控制器。
当前 GanYu 的附加自动降落条件已删除，主动降落由键盘请求；PX4 自身保护按原配置执行。

更多说明：

- [GanYu 输入输出、参数与验证记录](uav_ganyu_control/README.md)
- [A* + B 样条航点规划](uav_path_planner/README.md)
- [相机与 MID-360 可视化](uav_sim_visualization/README.md)
- [无人机模型](assets/gazebo_models/x500_mid360/README.md)
- [Walls 障碍场景](assets/gazebo_world/README.md)
- [保留的上游实验流程](docs/legacy_upstream_workflows.md)
