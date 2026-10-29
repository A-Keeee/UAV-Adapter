# 单机 Walls 障碍场景

新增的 `walls_demo.sdf` 基于 PX4 官方 Walls 场景，适合后续验证低速规划绕飞。
原有 `grass_world.sdf`、`run_swarm.sh` 和 PX4 中原有场景均保留，没有改写。

## 来源与改动

- 上游：[PX4/PX4-gazebo-models](https://github.com/PX4/PX4-gazebo-models)。
- 固定版本：`e05f4312d3f28aa621157610584a4870406cb6d3` 的 `worlds/walls.sdf`。
- BSD-3-Clause 许可证随附于 `LICENSE.PX4-gazebo-models`。
- 保留原来的四面墙和对应 collision；世界命名为 `walls_demo`，增加显式的
  Harmonic 物理/传感器插件、原点起降区标记，地理参考高度与旧草地场景对齐为 488 m。
- `walls_demo.gui.config` 基于本机 Harmonic 默认 GUI 配置，设置俯瞰视角；不改用户的全局 GUI 配置。
- 场景只使用本地基础几何体，不需要联网下载 Fuel 模型。

世界采用 ENU：X 向东、Y 向北、Z 向上。墙体尺寸单位为米：

- `box1`：中心 `(5, 0, 7.5)`，尺寸 `(1, 20, 15)`。
- `box2`：中心 `(-3, 5, 7.5)`，尺寸 `(10, 1, 15)`。
- `box3`：中心 `(13, -10, 7.5)`，尺寸 `(17, 1, 15)`。
- `box4`：中心 `(12, 0, 7.5)`，尺寸 `(1, 20, 15)`。

无人机在原点生成，最近墙面约 4.5 m。四面高墙构成转角和通道；在约 2.5 m
高度飞行时需要水平绕行。现已接入 [A* + B 样条 + GanYu 航点规划](../../uav_path_planner/README.md)，
从同一 SDF 的 collision 建立静态地图，实现自动绕墙；键盘飞行模式不提供避障。

## 启动

```bash
cd /home/ake/SJTU/适配终端/gazebo_sim
./run_obstacle_scene.sh
```

入口自动加载 ROS 2 和当前工作空间环境，启动 Gazebo、一个 `x500_depth_1`、
MicroXRCEAgent 和相机/时钟桥接。默认 PX4 实例为 1、MAV_SYS_ID 为 2，ROS 命名空间为 `/px4_1`。
`PX4_DIR` 可以指定其他已经构建完成的 PX4 v1.16 工作目录。
新入口使用该机型默认的 GPS 高度参考 `EKF2_HGT_REF=1`，不沿用旧入口的测距仪高度参考；
当前 `x500_depth` 没有向下测距仪，选择测距仪参考会使全球高度无效并阻碍预检。

场景入口本身不启动控制器，不会自动解锁或起飞。GanYu 已通过独立入口接入：

```bash
# 场景保持运行时，在另一个交互终端执行
cd /home/ake/SJTU/适配终端/gazebo_sim
./run_ganyu_keyboard.sh
```

随后按 `t` 起飞。GanYu 客户端默认提供本机 MAVLink 地面站心跳，也可使用 QGroundControl；
是否能够解锁仍取决于 PX4 预检。按键、控制层级、参数和当前已删除的附加自动降落条件，
详见[主 README 的单机入口](../../README.md#ganyu-quickstart)与
[GanYu 功能包说明](../../uav_ganyu_control/README.md)。
新增的 MID-360 机型可用 `./run_obstacle_scene.sh --mid360` 启动，
再运行 `./run_rviz.sh` 同时查看相机和点云，详见[主 README 的 RViz2 章节](../../README.md#rviz-sensors)。
该可选入口在本次运行目录生成包含 RGL 插件的世界副本，原地图文件保持不变。
自动规划任务使用 `./run_ganyu_planner.sh --start --rviz`；MID-360 机型同时加 `--mid360`。

原双机仿真运行时，新入口会拒绝启动，不会杀掉旧进程。请先在原仿真所属终端退出。
退出新入口时按 `Ctrl+C`，只关闭本入口创建的进程组。

无图形界面检查，成功后自动关闭本次仿真：

```bash
./run_obstacle_scene.sh --headless --check
```

需要延长等待时间时追加 `--timeout 120`。`--headless` 单独使用则保持后台仿真运行。

## 数据与验证

新入口会实际订阅数据并检查：

- Gazebo 的四面墙、起降标记和无人机实体存在。
- `/clock` 的仿真时间持续推进。
- `/px4_1/fmu/out/vehicle_local_position` 的水平/垂直位置有效。
- `/px4_1/fmu/out/vehicle_odometry` 的位置、速度和四元数有效。
- `/px4_1/fmu/out/vehicle_status_v1` 持续发布。
- `/camera/image` 有完整图像，`/camera/camera_info` 的尺寸及内参匹配。

`READY` 表示场景和数据链路就绪，不代表 PX4 预检已经通过或已经完成飞行。
`verification.json` 单独记录预检状态、飞行模式和解锁状态。

每次运行创建独立的 `log/obstacle_scene/<时间>_<随机后缀>/`，其中包括：

- Gazebo、GUI、PX4、XRCE Agent 和桥接日志。
- `verification.json`：验证结果；`camera.png`：实际相机快照。
- `source.sha256`：场景和两个原有文件的校验值。
- `px4/`：本轮独立的 PX4 参数、dataman 和 ULog，不复用或清除旧仿真目录。

`log/obstacle_scene/latest` 指向最近一次运行。仿真保持运行时，可另开终端重新检查：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
/usr/bin/python3 scripts/check_obstacle_scene.py --output log/obstacle_scene/latest
```

返回原有草地双机场景仍按主 README 的 `run_swarm.sh` 流程启动。

## 本机验证记录（2026-10-25）

Ubuntu 22.04 / ROS 2 Humble / Gazebo Harmonic 8.15.0 / PX4 v1.16.0：

- 无界面启动与退出检查通过；图形界面启动检查通过。
- 确认四面墙、起降标记与无人机均存在，世界时钟推进，位置与里程计有效。
- RGB 图像为 `1920×1080 / rgb8`，相机内参同步输出。
- 修正高度参考后，使用 ROS 2 Offboard 位置指令完成一次原地起飞、悬停、降落验证。
  目标相对高度 2.5 m，实际位置估计最大上升 2.491 m；稳定悬停 5 s 的位置跟踪 RMSE
  约 0.054 m，最后收到落地标志并确认上锁。
- 本次飞行使用 PX4 自带位置控制，没有接入 GanYu，也没有验证自动绕障。
- 成功运行日志：`log/obstacle_scene/20261025_173931_VUqo/`，含
  `verification.json`、`flight_verification.json`、`flight_trace.csv` 和 `overview.png`。

验证期间使用的临时飞行客户端保存在该次日志内；默认启动入口只做只读检查，仍然不会自动起飞。
