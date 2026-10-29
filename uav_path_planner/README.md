# A* + B 样条 + GanYu 航点避障

使用现有 Walls 场景与 `QuadrotorControllerGanYu`，实现给定 ENU 路点后的自动起飞、
三维 A* 搜索、七次 B 样条轨迹跟踪、逐点停稳及最终降落。GanYu 仍运行位置闭环，
只向 PX4 输出角速度与推力；没有将控制交给 PX4 的位置控制器。

## 运行

在仓库根目录编译：

```bash
./scripts/build_planner.sh
source install/setup.bash
```

沿用键盘入口的 `pymavlink` 地面站心跳依赖，本机已安装；新机器缺少时执行
`/usr/bin/python3 -m pip install --user pymavlink`。

终端 1 启动场景，等待 `READY`：

```bash
./run_obstacle_scene.sh
```

终端 2 执行航点任务并打开 RViz：

```bash
./run_ganyu_planner.sh --start --rviz
```

**`--start` 会自动解锁和起飞**。不加 `--start` 时只规划和显示，不解锁。
此入口不能与键盘控制入口或其他 Offboard 控制器同时运行。
完成全部航点后自动降落并退出；`Ctrl+C` 会先请求降落，等待上锁后退出。
`land_at_end: false` 时，末点持续悬停，需按 `Ctrl+C` 请求降落并退出。

相机 + MID-360 机型两边都加 `--mid360`，以匹配机体质量：

```bash
# 终端 1
./run_obstacle_scene.sh --mid360
# 终端 2
./run_ganyu_planner.sh --mid360 --start --rviz
```

无界面验证可用场景参数 `--headless`，任务入口省略 `--rviz`。

## 修改路点

编辑 [config/waypoints.yaml](config/waypoints.yaml)，或复制后通过 `--config /绝对路径/mission.yaml` 指定。
`waypoints` 是展平的 ENU 三元组：**东、北、上，单位米，相对原点起飞位置**。
默认 `[(0,8,2.5), (8,8,2.5), (8,0,2.5)]`；第一段必须绕过 box2，
第二段必须绕过 box1 北端，第三段飞入两墙之间。路点必须位于地图范围和膨胀障碍之外。
每个路点停稳后再执行下一段；高度可随路点变化，默认 2.5 m。

`takeoff_height` 同时由入口传给规划器和控制器。地图默认分辨率 0.4 m，
安全半径 0.65 m（含机体与余量），速度/加速度/jerk 限制分别为 1 m/s、0.7 m/s²、1.5 m/s³。
控制器另外拒绝超过 2 m/s、2 m/s²、5 m/s³ 的参考。

默认读取随包安装的 `walls_demo.sdf`。修改世界后需重新编译，或在配置中用
`world_file: /绝对路径/实际使用的世界.sdf` 指定同一世界。地图与 Gazebo 必须一致。
目前解析静态、轴对齐的 box collision；旋转障碍、SDF include 和其他静态碰撞形状会拒绝。
默认对齐关系对应原场景原点生成的单机；更改生成位置时需同步实现地图坐标对齐。

## 规划与控制

1. 读取场景碰撞几何，按无人机半径膨胀；26 邻域三维体素 A* 禁止穿角。
2. 可视线简化后建立端点钳制的七次 B 样条，航点处速度、加速度、jerk 为零，内部 C6 连续。
3. 对每个结点区间转为 Bézier 控制多边形并递归细分，用凸包性质校验整条轨迹，
   不仅检查离散采样点；不通过则加密控制点，仍不通过就拒绝执行。
4. 用导数控制多边形的保守界统一放慢时间，使速度、加速度和 jerk 满足限制。
5. 整条轨迹（结点、控制点、起始仿真时间）一次发送给 GanYu，控制器在 200 Hz 定时循环中
   解析计算位置、速度、加速度、jerk，再调用原 GanYu 控制律。无需依赖 Python 高频流式参考。
6. 任务检查轨迹接收、路点停稳、反馈时效与跟踪范围；失败则停止任务并请求降落。
   控制器原有键盘模式不受任务状态机影响，PX4 自身保护仍有效。

这是**已知静态地图避障**。MID-360 点云目前用于显示，尚未用于建图或动态障碍重规划。
跟踪包络基于 PX4 估计位置，不能当作实机或定位真值的避障保证。

## 可视化与日志

RViz：灰色为真实墙，红色半透明区域为安全膨胀，粉色球为路点，橙色为 A* 路径，
青色为 B 样条，绿色为实际飞行轨迹。可隐藏墙体以查看被高墙遮挡的轨迹。
规划视图默认关闭 MID-360 点云，避免遮挡路线；可勾选 `MID360 cloud (optional)` 显示。

- `/planning/astar_path`、`/planning/bspline_path`：规划路径。
- `/planning/actual_path`、`/planning/tracking_reference`：实际轨迹与执行参考。
- `/planning/markers`：墙体、安全区域与路点。
- `/planning/status`：任务状态、各段长度、约束、跟踪误差、最小墙面间距。
- `/planning/trajectory`：`uav_common_msg/BSplineTrajectory`，PX4 NED 坐标。
- `/ganyu/status`：含轨迹 ID、是否正在跟踪、反馈位置和速度。

`log/ganyu_planner/latest/` 保留 `controller.log`、`mission.log`、任务配置、
`result.json` 和逐采样 `result.csv`。`COMPLETE` 表示全部路点停稳且已落地上锁；
`PREVIEW` 只表示完成规划，不代表完成飞行。

```bash
colcon test --build-base "$HOME/.cache/gazebo_sim_planner/build" --packages-select uav_path_planner uav_ganyu_control \
  --event-handlers console_direct+
colcon test-result --test-result-base "$HOME/.cache/gazebo_sim_planner/build" --verbose
```

若设置了 `GAZEBO_BUILD_CACHE` 或构建了多个副本，以上路径改用构建脚本打印的缓存目录。

## 本机验证（2026-10-29）

- 编译成功，`colcon test-result` 汇总 12 项检查，0 失败。覆盖绕墙、不可达路点、
  碰撞区间、端点约束、解析导数及非法样条输入。
- `x500_depth`：三路点全部到达、落地上锁。1672 条跟踪样本，
  跟踪 RMSE 0.0175 m、最大误差 0.0399 m，最小估计墙面间距 0.9655 m。
  日志：`log/ganyu_planner/20261029_210609_dHxS/`。
- `x500_mid360`：三路点全部到达、落地上锁。1612 条跟踪样本，RMSE 0.0192 m、
  最大误差 0.0463 m，最小估计墙面间距 0.9721 m。
  日志：`log/ganyu_planner/20261029_211013_Xudg/`，含 RViz 截图。
- MID-360 全程独立观察到 11994 条角速度设定值、18539 条 body-rate 模式消息，
  0 条其他模式和 0 条 PX4 位置设定值；5550 帧 Gazebo 模型真值位姿的
  最小墙面间距为 0.9516 m。见该轮 `independent_observation.json`。

以上是本机静态地图理想仿真结果。可用 `scripts/observe_planner_flight.py` 在任务开始前
独立订阅检查控制输出；需先加载工作空间环境。真值检查另外需要只读桥接：

```bash
ros2 run ros_gz_bridge parameter_bridge \
  '/world/walls_demo/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V' \
  --ros-args -r /world/walls_demo/dynamic_pose/info:=/verification/gazebo_poses
```

另一个终端先启动观察器，再启动任务：

```bash
/usr/bin/python3 scripts/observe_planner_flight.py --require-truth --output /tmp/flight_observation.json
```

真值观察器只适用于当前单个动态无人机的 Walls 场景，使用 Gazebo 动态位姿列表中的
模型世界位姿；不会将后续 link 的相对位姿误作世界位置。
