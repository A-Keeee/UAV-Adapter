# C++ A* + B 样条 + GanYu 航点避障

使用现有 Walls 场景与 `QuadrotorControllerGanYu`，实现给定 ENU 路点后的自动起飞、
三维 A* 搜索、七次 B 样条轨迹跟踪、逐点停稳及最终降落。GanYu 仍运行位置闭环，
只向 PX4 输出角速度与推力；没有将控制交给 PX4 的位置控制器。
规划核心、点云处理、任务状态机和动态避障回归程序均为 C++17；样条求值复用 GanYu 的 C++ 实现。

## 运行

在仓库根目录编译：

```bash
./scripts/build_planner.sh
source install/setup.bash
```

地面站心跳由 C++ 节点通过 UDP 14550 发送 MAVLink HEARTBEAT；规划节点不再依赖 Python 规划代码或 `pymavlink`。
新增构建依赖为 Eigen3、TinyXML2、nlohmann-json 和 yaml-cpp；本机已安装。

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

相机 + MID-360 机型两边都加 `--mid360`，匹配机体质量并自动开启点云动态避障：

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
`world_file: /绝对路径/已知静态地图.sdf` 指定墙体地图。原有已知墙体应与 Gazebo 一致；
考察未知障碍物时，新增物体只进入 Gazebo 运行场景，不写入此地图文件。
目前解析静态、轴对齐的 box collision；旋转障碍、SDF include 和其他静态碰撞形状会拒绝。
默认对齐关系对应原场景原点生成的单机；更改生成位置时需同步实现地图坐标对齐。

## 规划与控制

1. 读取场景碰撞几何，按无人机半径膨胀；26 邻域三维体素 A* 对每条邻接边执行精确线段碰撞检查，禁止穿角。
   不再额外膨胀半个体素对角线，避免将可通行窄通道误封。
2. 可视线简化后建立端点钳制的七次 B 样条，航点处速度、加速度、jerk 为零，内部 C6 连续。
3. 对每个结点区间转为 Bézier 控制多边形并递归细分，用凸包性质校验整条轨迹，
   不仅检查离散采样点；不通过则加密控制点，仍不通过就拒绝执行。
4. 用导数控制多边形的保守界统一放慢时间，使速度、加速度和 jerk 满足限制。
5. 整条轨迹（结点、控制点、起始仿真时间）一次发送给 GanYu，控制器在 200 Hz 定时循环中
   解析计算位置、速度、加速度、jerk，再调用原 GanYu 控制律。
6. 任务检查轨迹接收、路点停稳、反馈时效与跟踪范围；失败则停止任务并请求降落。
   控制器原有键盘模式不受任务状态机影响，PX4 自身保护仍有效。

未加 `--mid360` 时使用已知静态地图避障；加上后同时使用实时雷达障碍物。
跟踪包络基于 PX4 估计位置，不能当作实机或定位真值的避障保证。

## MID-360 动态避障（不建图）

订阅 `/lidar/points`，按点云采样时刻的 TF 将 `mid360_link` 点转换到 `odom`，再对齐
航点使用的 ENU 起飞坐标。无 RViz 时也会启动 TF 节点。过滤非有限值、0.45 m 内的机体回波、
8 m 外的点和已知地面；已有 SDF 墙体附近 0.1 m 内的回波由精确墙体几何负责碰撞检查，
避免同一墙面重复膨胀。其他回波按 0.3 m 体素处理，增加 0.15 m 余量，再按机体安全半径膨胀。

当前仿真已知生成朝向为 ENU yaw=0，默认在未解锁时用这个已知朝向校准一次 EKF 初始航向偏差，
避免点云墙面相对 SDF 旋转。`lidar_spawn_yaw_enu` 单位为弧度；改变生成朝向时应同步修改。
已有准确世界航向的环境可设置 `lidar_align_spawn_yaw: false`。这项校准使用已知初始条件，不做建图。
默认同时启用 `lidar_register_known_walls`：每帧将观测与原有 SDF 竖直墙面做鲁棒配准，
持续修正航向漂移和小幅水平偏差，避免起飞前冻结的航向补偿把墙面点云投到通道里。
配准只对照已有墙体，不读取未知柱体配置，不新增地图几何；缺少足够墙面观测时保留已有对齐值。

只保留最近 0.6 s 的观测，不累计全局点云，不生成或保存雷达地图。障碍物移动或消失后，
旧观测会过期。约每 0.1 s 检查未来 6 s 的样条区间，使用 Bézier 凸包校验覆盖采样点之间的曲线。
发现碰撞后进入 `BRAKING`，取消旧轨迹并沿用 GanYu 的 2 s 平滑制动；停稳后进入
`REPLANNING`，在后台以“静态墙体 + 当前局部点云障碍物”重新执行 A* 和样条认证，
发送前再用最新观测复核，然后恢复 `TRACKING`，原航点顺序保持不变。
重规划默认额外给局部雷达障碍物预留 `lidar_replan_margin: 0.2` m，减少新回波落在路线边缘引发的反复制动。
若额外余量封住通路，自动用原碰撞边界重试；仍无路时把搜索栅格从 0.4 m 细化到 0.2 m 再试。
这些重试均保持机体安全半径和连续曲线碰撞认证。

启用避障后，有效点云和对应 TF 就绪才会起飞。航行中点云/TF 超过 0.5 s 未更新会制动等待，
恢复后重新规划；自最后有效帧起 10 s 仍未恢复，或 30 s 内无法找到安全路线，则沿用任务失败降落流程。
起飞期间发现爬升柱受阻或丢失雷达数据也会中止并请求降落。最终降落由 PX4 LAND 执行。
这是基于当前观测的低速反应式避障，没有障碍物速度估计或运动预测；默认适用于当前仿真速度，
不保证能躲开突然进入制动距离内的快速物体。

参数在 `config/waypoints.yaml` 中：`lidar_range`、`lidar_voxel_size`、`lidar_margin`、
`lidar_memory`、`lidar_timeout`、`lidar_lookahead`、`replan_timeout`。
`lidar_register_known_walls` 控制已有墙面配准，`lidar_replan_margin` 控制重规划时的额外局部余量。
`lidar_memory` 必须不小于 `lidar_timeout`，探测距离必须覆盖最大速度乘以前视时间再加安全半径。
`--mid360` 自动覆盖 `lidar_enabled: true`；直接启动任务节点时需自行启用该参数并启动同一 TF 链。

## 三航段未知障碍物场景

```bash
# 终端 1：启动场景并等待 READY
./run_obstacle_scene.sh --mid360 --unknown-obstacles
# 终端 2：执行默认三航点任务
./run_ganyu_planner.sh --mid360 --start --rviz
```

此模式额外启动 C++ `unknown_obstacle_scene` 场景节点。每个航段已经进入 `TRACKING`、
控制器确认正在执行轨迹后，场景节点通过 Gazebo `create` 服务生成该段的障碍柱。
初始三段全局路线已经算好，障碍物位置不发送给规划节点，也不写入 `walls_demo.sdf`。
原全局路径会穿过这些柱体；飞机依靠实时 MID-360 点云制动并重新规划。

默认三根柱体均为橙红色，尺寸 `0.6×0.6×5 m`，从地面竖起，各段中心 ENU 坐标为：

- 起飞点到 WP1：`(1.33, 1.77, 2.5)`。
- WP1 到 WP2：`(1.77, 9.33, 2.5)`。
- WP2 到 WP3：`(8.0, 5.5, 2.5)`。

这些位置针对默认航点配置，使用独立的
[unknown_obstacles.yaml](../assets/gazebo_world/unknown_obstacles.yaml) 配置，修改位置和尺寸后重新启动场景即可。
更换航点时也需相应调整此场景配置。障碍物生成后保持不动并一直保留到场景关闭，
所以这是“运行中出现的未知障碍物”测试，没有模拟连续移动的行人或车辆。
不要同时启动下面的单障碍物回归程序。

场景节点只订阅任务/控制状态并操作 Gazebo，不发布控制设定值。场景入口不会自动起飞。
无需图形界面时追加 `--headless`，规划入口省略 `--rviz`。

`log/obstacle_scene/latest/unknown_obstacles.json` 记录每根柱体的生成时刻、
该航段是否在生成后出现重规划、最近估计间距以及最终任务结果。
只有三根障碍物均已生成、各航段均观察到重规划、间距满足安全半径，且所有航点完成并落地上锁，
才标记 `passed: true`。间距基于 PX4 估计位置计算。
`unknown_obstacles.log` 保存生成与重规划响应日志，`unknown_obstacles.yaml` 保存本轮使用的配置。

场景节点状态也可从 `/simulation/unknown_obstacles/status` 读取；该话题只供观察，不接入全局规划。
重复评估时重新启动场景，以恢复原点起飞位置和三个障碍物的生成状态。

本机于 2026-11-10 完成无界面三柱测试：三段均在障碍物生成后观察到重规划，全部航点完成并落地上锁，
`passed: true`；三根柱体最近估计间距分别为 1.3281、1.3770、1.3885 m。
该轮共 17 次制动与重规划，其中第三段较频繁，适合继续评估点云变化下的重规划稳定性。
日志为 `log/obstacle_scene/20261110_163101_D0xR/unknown_obstacles.json` 和
`log/ganyu_planner/20261110_163152_RBfB/`。16 个 C++ 单元测试通过；新增测试确认
全局地图仍只有四面墙，而每段原全局路线会与对应测试柱体相交。

## 单障碍物回归

可在场景输出 `READY` 后，先在一个已加载 ROS/工作空间环境的终端运行回归检查，再在另一个终端
启动默认航点任务。C++ 检查程序会在第一段当前路线中加入一个不在 SDF 地图里的 0.6 m 障碍物，
确认制动、重规划、绕过障碍到达第一航点，然后将障碍移开，等待其余航点完成和落地上锁；
退出时删除本次创建的障碍物。该程序仅操作 Gazebo，不负责起飞。

```bash
# 终端 2，先运行
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run uav_path_planner lidar_avoidance_check --ros-args -p output:=/tmp/lidar_avoidance_check.json
# 终端 3
./run_ganyu_planner.sh --mid360 --start --rviz
```

## 可视化与日志

RViz：灰色为真实墙，红色半透明区域为安全膨胀，粉色球为路点，橙色为 A* 路径，
青色为 B 样条，绿色为实际飞行轨迹。可隐藏墙体以查看被高墙遮挡的轨迹。
规划视图默认关闭 MID-360 点云，避免遮挡路线；可勾选 `MID360 cloud (optional)` 显示。

- `/planning/astar_path`、`/planning/bspline_path`：规划路径。
- `/planning/actual_path`、`/planning/tracking_reference`：实际轨迹与执行参考。
- `/planning/markers`：墙体、安全区域与路点。
- `/planning/lidar_obstacles`：短时局部障碍物体素（橙色），默认显示。
- `/planning/status`：任务状态、各段长度、约束、跟踪误差、最小墙面间距。
- `/planning/trajectory`：`uav_common_msg/BSplineTrajectory`，PX4 NED 坐标。
- `/ganyu/status`：含轨迹 ID、是否正在跟踪、反馈位置和速度。

`log/ganyu_planner/latest/` 保留 `controller.log`、`mission.log`、任务配置、
`result.json` 和逐采样 `result.csv`。`COMPLETE` 表示全部路点停稳且已落地上锁；
`PREVIEW` 只表示完成规划，不代表完成飞行。
`result.json` 的 `lidar_avoidance` 记录有效帧数、局部体素数、制动次数、成功重规划次数及各次路线信息。
还记录墙面配准残差、内点数、水平补偿和最近一次重规划错误。
找不到路线时另外保存 `result.replan_failure.json`，包括该次搜索的起终点、原始障碍物快照和栅格参数，
便于区分实际堵塞、坐标偏差和栅格分辨率问题。

```bash
colcon test --build-base "$HOME/.cache/gazebo_sim_planner/build" --packages-select uav_path_planner uav_ganyu_control \
  --event-handlers console_direct+
colcon test-result --test-result-base "$HOME/.cache/gazebo_sim_planner/build" --verbose
```

若设置了 `GAZEBO_BUILD_CACHE` 或构建了多个副本，以上路径改用构建脚本打印的缓存目录。

## C++ 动态避障验证（2026-11-10）

针对持续提示 `A* found no collision-free route` 的修复验证：

- 24 个 C++ 单元测试通过，连同 GanYu 和测试框架汇总 26 项、0 失败。
  新增窄通道、细栅格重试、精确对角边、连续航向漂移、未知回波保留及报告中悬停位置的回归检查。
- 三柱场景完成三个航点并落地上锁，`unknown_obstacles.json` 为 `passed: true`。
  本轮无 `Waiting for a safe lidar route`，共 7 次制动和重规划；末段同时启动 Gazebo GUI 和 RViz 验证。
- 三根柱体最小估计间距为 1.3562、1.0182、1.0863 m，最小估计墙面间距 0.6892 m，
  安全半径仍为 0.65 m。3855 条跟踪样本，RMSE 0.0145 m，最大跟踪误差 0.0397 m。
  这些间距均基于 PX4 估计位置。
- 日志：`log/ganyu_planner/20261110_164852_gnr4/` 和
  `log/obstacle_scene/20261110_164752_RxrS/unknown_obstacles.json`。

以下为修复前的迁移验证记录：

- 规划器已编译为 ELF 可执行程序；15 个 C++ GoogleTest 用例通过，连同 GanYu 样条检查，
  `colcon test-result` 汇总 17 项、0 失败。覆盖原三段绕墙路线、导数、端点、不可达路线、
  连续碰撞认证、点云大小端与行填充、局部障碍物过期、容量拒绝和动态障碍绕行。
- 在 `--mid360 --headless` 场景中，由 C++ 回归程序在第一段正在执行的路线前方加入
  一个未写入 SDF 的 0.6 m 障碍物。检测后制动并重规划，绕过障碍到达第一航点，
  障碍物随后被移开。全部三个航点完成，自动落地上锁，回归结果 `passed: true`。
- 该轮共接收 910 帧有效点云、制动和重规划各 3 次；1914 条跟踪样本，RMSE 0.0175 m，
  最大误差 0.0458 m，最小估计墙面间距 0.9931 m，最小估计临时障碍物间距 1.2923 m。
  障碍物间距基于 PX4 估计位置与测试盒体计算，不是独立定位真值。
- 日志：`log/ganyu_planner/20261110_160809_7bg6/`，包括 `result.json`、`result.csv`、
  `mission.log`、`controller.log` 和 `dynamic_avoidance_check.json`。

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
