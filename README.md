# Gazebo 单机无人机仿真与控制验证平台

本工作空间基于 **ROS 2 Humble＋Gazebo Harmonic＋PX4 SITL定位＋GanYu控制器**，支持两类未知环境任务：

- **仓库探索与巡飞**：OctoMap＋三维Frontier＋EGO-Swarm，从空白地图探索，保存地图，完成至少四点巡飞、返航与降落。
- **树林A→B穿越**：EGO-Planner-v2原生滚动局部地图＋MINCO，实时避障完成80 m目标航程，悬停确认并降落，不建立全局地图。

三种仿真雷达为 **MID-360、前向Livox Avia、Ouster OS1-64**。外部PointCloud2无需登记型号或填写FOV、盲区、线数、量程；须提供正确XYZ、采样时间、TF与测量原点。

## 当前验收结果

**2026-11-16运行验收完成，30/30通过。** 本次已重新运行独立发布审计，`complete=true`、`issues=[]`。

- 18/18组正常飞行通过当前交付版本核验，4/4类物理故障响应通过，22/22份最新录包核验通过。
- 软件回归：任务17项、GanYu6项、索引1项；点云接口17/17，实际OS1 64→16通道回放1/1通过。
- 正常飞行零障碍碰撞；最小实测间距约1.215 m，最大单次跟踪RMSE 0.0823 m，最大航点误差0.213 m。
- 仓库保存地图覆盖率98.05%～99.82%；树林观测与原生环形缓存容量固定。

三组条件是固定场景下的 **0°／+45°／−45°初始航向**，不代表三个地图种子。本轮使用PX4仿真定位，不含真实硬件、无GPS定位或跨重启重定位；ODIN1不纳入。

当前验收批次为 `log/manual_acceptance/20261116_040954_1261551_scoring_lifecycle`。查看 [六种组合结果与逐次证据](log/manual_acceptance/20261116_040954_1261551_scoring_lifecycle/results/README.md)、[机器核验](log/manual_acceptance/20261116_040954_1261551_scoring_lifecycle/audit.json) 和 [本次独立复核](log/release_review_20261116/audit.json)。详细使用方式见 [未知环境任务README](uav_autonomy/README.md)，消息及坐标规则见 [接口说明](docs/autonomy_interfaces.md)，开发过程见 [实施记录](docs/autonomy_implementation.md)。

## 未知环境任务快速启动

所有命令在工作空间根目录执行。先在终端1启动一种场景：

```bash
./run_autonomy_scene.sh --world warehouse --lidar mid360
```

默认自动执行任务会录制 ROS bag，退出时压缩并校验。日常调试可加 `--no-record` 关闭录包，例如 `./run_autonomy.sh --task forest --start --no-record`；仍保留文字日志、配置、任务结果及评估输出。需要回放数据时加 `--record`（默认开启）。关闭录包的运行不具备录包回放验收证据。

等待 `verified; vehicle remains unarmed.`，保持场景运行，在终端2启动仓库任务：

```bash
./run_autonomy.sh --task explore --start
```

树林任务将终端1改为 `./run_autonomy_scene.sh --world forest_long --lidar avia`，终端2改为 `./run_autonomy.sh --task forest --start`。场景端 `--lidar`可选 `mid360`、`avia`、`os1_64`；每轮使用新的场景会话，避免同时运行多套控制器。地图巡飞入口为 `./run_map_patrol.sh`，外部点云接入见 [任务README](uav_autonomy/README.md#4-未登记雷达接入)。

场景和任务启动后，在第三个终端运行 `./view_autonomy.sh`，打开点云、障碍地图、EGO规划曲线、实际轨迹及速度／阶段显示。该入口只读，不参与控制。详见 [RViz说明](uav_autonomy/README.md#rviz可视化)。

调速修改 [warehouse.yaml](uav_autonomy/config/warehouse.yaml) 或 [forest.yaml](uav_autonomy/config/forest.yaml) 的 `max_velocity`，例如0.3 m/s；新任务会话生效，无需重新编译。默认0.5 m/s，实际速度可因避障和观测而降低，具体见 [速度说明](uav_autonomy/README.md#调整飞行速度)。

`./run_acceptance.sh status`查看当前完成结果；默认入口在本批完成后不会重复起飞。需要独立完整重跑时使用 `./run_acceptance.sh --fresh`。构建、续跑和磁盘要求见 [手动验收指南](docs/manual_acceptance.md)。

## 原有键盘与A*演示

原键盘、相机图像、MID-360点云显示和 **给定路点→三维A*避障→七次B样条→GanYu跟踪→降落**入口继续保留。旧A*演示使用Walls静态碰撞地图；`--mid360`启用实时点云动态避障和短时局部障碍物。新EGO任务使用各自原始轨迹，不重新拟合为七次曲线。

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
RViz 用橙色显示 A* 、青色显示 B 样条、绿色显示实际飞行轨迹，并显示墙体与安全膨胀区域。
默认路点要求绕过两面墙；修改 [waypoints.yaml](uav_path_planner/config/waypoints.yaml) 可指定自己的 ENU 路点。
`Ctrl+C` 会先请求降落再退出。不要同时运行键盘控制入口。
详细原理、约束、日志与话题见 [规划包说明](uav_path_planner/README.md)。

考察未知障碍物触发避障与重规划时，使用以下两条命令：

```bash
# 终端 1，等待 READY
./run_obstacle_scene.sh --mid360 --unknown-obstacles
# 终端 2
./run_ganyu_planner.sh --mid360 --start --rviz
```

三个航段开始跟踪后各生成一根橙红色障碍柱，位置分别为 ENU `(1.33,1.77)`、
`(1.77,9.33)`、`(8,5.5)`，尺寸均为 `0.6×0.6×5 m`，落在原全局规划路线中。
障碍物只通过 Gazebo 服务生成，不写入全局地图，规划器通过 MID-360 点云发现并绕行。
结果保存在 `log/obstacle_scene/latest/unknown_obstacles.json`。

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
