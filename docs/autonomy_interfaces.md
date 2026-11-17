# 通用点云、原生轨迹与 GanYu 接口

本说明对应当前源码的实际接口。任务入口见 [任务说明](../uav_autonomy/README.md)，验收范围见 [确认方案](autonomy_plan.md)。接口可用、任务完成和独立验收通过是三个不同状态。

## 外部 PointCloud2

通过 `./run_autonomy.sh --task forest --cloud-topic /my_lidar/points` 选择输入话题；仓库任务改为 `--task explore`。默认话题是 `/lidar/points`，订阅采用 ROS 2 `SensorDataQoS`。

输入类型为 `sensor_msgs/msg/PointCloud2`，最低要求为：

- 有标准名称 `x`、`y`、`z`，各为一个 FLOAT32 或 FLOAT64，单位为米；三个字段可以使用不同浮点类型。
- `header.stamp` 表示测量采样时间，并与 PX4 状态、TF 和当前 `/clock` 对应。
- `header.frame_id` 是 XYZ 实际所属的坐标系，TF 能在该采样时刻求得其到 `odom` 的变换。
- 有明确的真实测量原点；默认使用点云 frame 的原点，也可通过 `--sensor-origin-frame` 单独指定。

不需要雷达型号、FOV、盲区、量程、线数、`intensity`、`ring` 或逐点时间。支持字段重排、偏移、FLOAT32／FLOAT64、大小端、组织化点云及行填充。NaN／Inf 被过滤；机体回波按机体几何过滤；点数过多时空间降采样，保留真实端点，不截取消息前缀。

解析器有计算资源限制：单帧不超过 4,000,000 点、数据不超过 256 MiB，布局越界和超限整帧拒绝。这是软件预算，不是硬件量程或 FOV。具体检查见 [cloud.hpp](../uav_autonomy/include/uav_autonomy/cloud.hpp)。

如果驱动已把 XYZ 转换到 `odom`、`map` 或其他世界坐标，必须另给实际原点：

```bash
./run_autonomy.sh --task forest --cloud-topic /my_lidar/world_points \
  --sensor-origin-frame my_lidar
```

已转换到 `base_link` 等非传感器坐标的云，也应明确给出真正的传感器原点，不能把变换后的坐标原点误当成射线起点。累计地图和多源融合云不能在没有明确测量原点的情况下冒充单帧雷达测量。

新机体若不由本工作空间的场景入口启动，需给出其真实总质量 `--mass 千克数`。质量用于 GanYu 推力计算，与雷达能力无关。在已启动的本地仿真场景中会自动读取机体总质量。

不加 `--start` 仅预览。外部数据接入当前仿真闭环时，点云、位姿、时间和场景必须描述同一架飞机的同一次运动；不同真实场景采集的雷达数据与当前仿真位姿不能组成有效闭环。

## TF、坐标与时间

正常的 TF 关系为 `odom → base_link → 传感器坐标系`：

- `odom` 为 ENU，`base_link` 为 FLU。
- PX4 状态桥独占发布动态 `odom → base_link`，使用 `timestamp_sample` 及 DDS 时间同步关系，不把接收时间当成采样时间。
- 场景入口或外部驱动发布正确的安装 TF。已有正确 TF 时不重复配置，也不要启动第二个机体动态 TF 发布器。
- TF 查询使用点云采样时间，变换 XYZ 和测量原点后才更新地图。

`/autonomy/odom` 是标准 `nav_msgs/msg/Odometry`：位姿在 ENU `odom` 中，`twist` 在 FLU 子坐标系中。`/autonomy/planner_odom` 是专供 EGO 的适配输出，其线速度按上游接口约定放在世界 ENU 中；其他程序应使用前者。

输入频率由有效采样间隔估计，超时门槛随实际周期调整。暂停仿真时轨迹与仿真时间超时不前进；时钟回拨撤销旧任务并锁定故障，不能在新时间域继续旧轨迹。实现见 [状态桥](../uav_autonomy/scripts/px4_state_bridge.py) 和 [任务节点](../uav_autonomy/src/autonomy_node.cpp)。

## 给规划器的输入

- `/autonomy/cloud`：变换到 `odom` 的通用 XYZ 测量点；树林接入 EGO-v2 原生滚动地图。
- `/autonomy/occupied`：当前占据地图的障碍点；仓库接入 EGO-Swarm 的固定范围规划地图。
- `/autonomy/planner_odom`：EGO 所需状态。
- `/autonomy/goal`：ENU 目标 `geometry_msgs/msg/PoseStamped`。
- `/autonomy/speed_limit`：接入层要求规划器采用的速度约束；两后端按各自约束形式预留余量，不在输出后单独裁剪速度。

无回波方向保持未知。计算窗口不等于雷达量程；超窗有效射线可以更新窗内经过的空间，截断处不生成障碍。树林的观测体素和原生环形地图容量固定，旧观察位置也随窗口移出而清除。导航不订阅 Gazebo 真值，不读取场景碰撞几何或评分 ROI。

静态树林的执行地图与原生 v2 点云入口采用一致的占据保留规则：有效端点命中的体素在窗口内保留为障碍，后续穿越射线不直接清除它；移出窗口后删除，返回时重新观测，不恢复旧障碍。未知／空闲体素继续按有效射线更新。窗口内动态障碍移走后的即时清除不属于此模式；室内 OctoMap 仍采用概率更新。

## 原生轨迹传输

`/autonomy/native_trajectory` 使用 [NativeTrajectory.msg](../uav_common_msg/msg/NativeTrajectory.msg)。

- `start_time` 是原轨迹的仿真起始时间；`header.stamp` 是消息发布时刻，二者不能互换。
- `BSPLINE` 保留阶数、原始 knots 和按 `x,y,z` 展平的控制点。原始 knots 可以为负数。
- `POLYNOMIAL` 保留每段正时长；每段按 X、Y、Z 顺序存系数，每轴六项，按降幂排列，对应 EGO-v2 的五次多项式。
- p/v/a/jerk 直接由原表示求值和求导，正常任务轨迹不重新拟合。起降、制动和独立观察位移另有参考曲线。

轨迹求值见 [trajectory.hpp](../uav_autonomy/include/uav_autonomy/trajectory.hpp)，上游消息转换见 [bridge.cpp](../third_party/ego_bridge/src/bridge.cpp)。

## GanYu 外部参考

`/autonomy/reference` 使用 [ExternalReference.msg](../uav_common_msg/msg/ExternalReference.msg)：

- `header.frame_id` 必须为 `odom`，`header.stamp` 为当前参考采样时刻，`valid_until` 为可执行截止时间。
- `trajectory_id` 标识执行参考，时间戳递增，旧 ID、重复或过期样本拒绝。
- `position`、`velocity`、`acceleration`、`jerk` 均为世界 ENU 向量，单位分别为 m、m/s、m/s²、m/s³。
- `yaw` 为 ENU 航向，单位 rad；`yaw_rate` 为其导数，单位 rad/s。不要预先转为 NED 再发布到此话题。

ENU → NED 转换发生在 **GanYu 的 ROS 接入回调**中：三维向量转换为 `(y,x,-z)`，航向为 `π/2-yaw`，航向角速度变号。原平移控制律继续在 PX4 NED／FRD 状态下工作。消息的世界向量不是 FLU 机体系向量。

当前任务接入层约以 200 Hz 发布采样参考，有效期通常不超过 0.12 s，并受整条原生轨迹的执行窗口约束。GanYu 以 200 Hz 执行原控制律；检查样本年龄、截止时间、有限数值、切换连续性及完整制动可行性。V/A/J 上限为 1／0.7／1.5，航向角速度上限为 0.5 rad/s，具体任务速度可更低。无有效新参考时进入独立制动，不把运动导数突然置零，也不把裁剪后的速度冒充原轨迹。

控制器实际采样参考发布在 `/ganyu/control_reference`，包含内部起飞／制动参考，供跟踪与独立评分使用。PX4 输出为角速度与推力；同一输出只能由一个 GanYu 实例持有。人工退出 Offboard 后进入 `YIELDED`，任务停止接管。

## 接管反馈与 ID

[TrajectoryFeedback.msg](../uav_common_msg/msg/TrajectoryFeedback.msg) 包含时间、ID、`accepted`、`braking` 和 `reason`。

`/autonomy/controller_feedback` 确认的是执行参考 ID；`/autonomy/trajectory_feedback` 面向桥中的公共原生候选 ID。两者有显式映射，不能仅按数值相等判断原生轨迹已执行。只有 GanYu 真正接受参考后，任务层才确认公共候选；桥再通过 `/autonomy/native_accepted` 确认上游原生 ID。每个后端只允许一个等待接管的候选。

候选拒绝（`accepted=false, braking=false`）与执行停止（`braking=true`）分别处理。新候选无效时，只要当前已接管轨迹仍满足实时数据、原执行期限、有效轨迹余量、机体与完整制动走廊检查，且本次拒绝没有降低执行速度上限，就保留其原始参考和控制器 ID；不重新拟合或接管无效候选。桥暂撤销上游 `/autonomy/execution_allowed` 以重置规划状态，等待 0.1 仿真秒后重新发送同一任务目标，原控制器参考继续由任务层独立检查。候选时长上限仍为 300 秒，超限候选继续拒绝。

真正制动、观测／数据失效或人工退出会取消待重试目标并保持撤销状态，桥不会自行恢复；需要任务层之后明确发布新目标才能重新规划。转向、扫描、旧目标阶段或重置前的迟到候选不能重新启动执行。完整闭环应使用任务入口和桥，不能仅监听一条轨迹消息就假定控制器已接管。

## 同会话巡检

`./run_map_patrol.sh` 向当前探索会话的 `/autonomy/mission_command` 发布巡检请求，保留地图和定位原点。

```bash
./run_map_patrol.sh --points /绝对路径/points.json
```

文件为 ENU XYZ 数组，至少四个点。请求点必须有限、处于执行高度内、有已测净空、可达且彼此分离；至少有不同巡检高度。不给文件则从当前地图选点。请求会先制动，再保存地图、验证连接并巡飞；不支持跨重启地图重定位。拒绝原因写入 `events.jsonl`，不能把“请求已发出”当成巡检已完成。

## 状态与结果判读

巡检任务状态增加 `patrol_generation`、`patrol_auto_generated`、`patrol_replans`。自动巡检目标被后续已测障碍判定失效时，旧四点留档，整组重新生成，`patrol_index` 从零开始；独立评分仍只使用最新一组的真值样本。`auto_patrol_target_invalidated` 与 `auto_patrol_replanned` 事件记录失效点、旧进度、新版本和新四点。用户指定的目标不会被替换。

`/autonomy/status` 和 `/ganyu/status` 为 JSON 格式的 `std_msgs/msg/String`。常用诊断包括 `phase`、`reason`、`cloud_error`、`data_ready`、点数／频率／点云与位姿年龄、规划速度、占据／未知体素证据和缓存容量。

`data_ready=true` 仅表示输入有效及时，不能证明当前位置或整段轨迹安全。`current_safe`、扫掠机体及完整制动检查分别核验净空。`OBSERVE`／`TURN` 表示正在获取观测或等待真实转向；规划阶段保留悬停参考；`FAILED_HOVER` 是失败悬停；`COMPLETE_WITH_INCOMPLETE_MAP` 是已经降落但探索超时，仍判失败。

独立观察位移对同一个实际停止点做评分、去重和执行检查；不会给远端候选点打分后停在另一个已访问的中间格。候选仍来自当前位置附近的固定小范围，选择前检查从控制器悬停参考到该点的完整曲线及各时刻制动走廊，执行中再用最新地图检查。已观测停止点按体素及近邻距离去重。该逻辑不读取雷达品牌、线数或 FOV，也不会把未知体素改为空闲；找不到安全新观察点时仍保留停车／失败处理。

`result.json` 的 `success=true` 表示任务流程正常降落；最终通过还须看独立 `evaluation/evaluation.json` 的 `acceptance_pass`、当前版本核验以及 `bag/replay_verification.json`。不能用历史成功覆盖同配置的最新失败，也不能用软件接口测试代替完整飞行。

关闭录包的实际感知数据流回放使用 `scripts/preview_autonomy_recording.py`，详见 [回放入口](../uav_autonomy/README.md#已关闭录包的感知回放)。它使用隔离 ROS 域和录制的仿真 `/clock`，不启动控制器；存储接收时间不能代替消息采样时钟。


树林恢复流程：一次安全观察换位及随后的完整扫描完成后，撤销旧候选的失败位置缓存，再向原 B 点请求新的原生轨迹；地图中的未知和障碍均保留。`observation_replan_from_new_view` 事件给出旧失败点及其当时是否获得支撑。重试后的每条原生轨迹照常通过执行前缀、动态限制、完整制动走廊和时效检查；不是直接执行刚才失败的轨迹。新一次原生输出等待从重新进入任务执行阶段计时，已有扫描／转向时间不计入该次等待；任务总超时、目标进展保护和重试次数保持原规则。

2026-11-14 MID-360 森林条件 2 停车修复：滚动地图的原生完整制动管使用 `ObservedMap::stoppingSafe`，在原障碍安全半径外额外预留一个地图体素（当前 0.20 m）。新增状态字段 `native_stopping_obstacle_reserve_m`。未知体素仍按原机体扫掠椭球检查，不填充未知为空闲；非滚动的室内地图不增加这层余量。观察换位目标同样要求停车余量；恢复过程本身继续按原安全半径逐点检查完整曲线和完整制动管，允许从原标准安全、但新回波已消耗停车余量的位置移向有余量的目标。原标准不安全的起点仍禁止运动。任务时间、重试次数、飞行高度界限、GanYu 和原生优化器均未调整。

2026-11-14 后续高度恢复修复：`nativeStoppingSafe` 将地图停车余量与高度停车余量统一用于原生完整制动管，以及扫描后的恢复判断。森林原硬执行区间仍为 ENU Z 1.5–2.3 m，原生制动再预留上下各 0.10 m；EGO-v2 内部虚拟壁收至 1.6–2.2 m。任务显式 `execution_min_z/max_z` 优先于按原生虚拟壁推导的后备值，因此调规划参数不会静默移动硬安全界限。室内推导方式保留。观察请求事件增加 `unsupported_height_safe`／`unsupported_stopping_safe`，状态增加 `native_stopping_height_reserve_m`。保留原地图和 GanYu 控制律、未知空间检查、制动多项式和任务保护预算。

2026-11-15：森林 v2 到远 B 的单目标全局多项式只作为局部目标搜索的几何引导。其输入速度采用朝目标的非负切向分量，避免长引导把悬停横向残差放大。该条件化不用于实际执行参考；局部 MINCO 仍用原始测量 p/v/a 作为起始边界，接入层直接计算并执行原生输出，无重拟合、无参考裁剪。新原生回归对两组录包状态核对这一边界条件。
