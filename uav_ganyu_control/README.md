# GanYu 单机键盘控制

独立新增的 ROS 2 包，移植本机 `sz_indoor_transport` 的 `QuadrotorControllerGanYu`。
不替换原来的 `uav_keyboard_control`，不修改场景、机型、传感器、PX4 参数或原启动脚本。
场景入口与关键 ROS 节点的启动说明见[主 README](../README.md#ganyu-quickstart)。

## 编译与使用

在本工作空间根目录编译本包即可：

```bash
cd /home/ake/SJTU/适配终端/gazebo_sim
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --symlink-install --packages-select uav_ganyu_control --cmake-args -DCMAKE_BUILD_TYPE=Release
```

终端 1：按原入口启动已经加入的障碍场景，等待 READY。

```bash
cd /home/ake/SJTU/适配终端/gazebo_sim
./run_obstacle_scene.sh
```

终端 2：启动新控制器和键盘客户端。

```bash
cd /home/ake/SJTU/适配终端/gazebo_sim
./run_ganyu_keyboard.sh
```

启动后不自动解锁。状态显示“就绪”后按 `t`，进入 Offboard 并起飞到 1.5 m。
松开移动键后，飞机保持最后的位置目标。长按由终端的按键重复产生连续的目标增量。

- `w / s`：北 / 南；`a / d`：西 / 东，每次改变目标 0.25 m。
- `e / q`：上 / 下，每次改变目标 0.25 m。
- `空格`：将当前反馈位置设为悬停目标，并平滑制动。
- `l`：交给 PX4 原有 LAND 模式正常降落，等待自动上锁。
- `x` / `Ctrl+C`：先请求降落，确认未解锁后退出键盘和控制器。

方向使用世界坐标，不随机头转动。默认水平目标限制在起飞点南北/东西各 ±3 m，
高度限制为 0.5～3 m；这只是手动试飞范围限制，不是障碍物检测或避障。
终端必须保持键盘焦点，中文输入法请切换到英文。

客户端为本机 SITL 提供 MAVLink 地面站心跳，不改 PX4 参数。如果 QGroundControl
已占用 UDP 14550，会保留已有连接；也可显式使用 `./run_ganyu_keyboard.sh --external-gcs`。
不能与原键盘、RL、视觉控制或其他 Offboard 发布节点同时控制同一架飞机。

## 可选：相机 + MID-360 新模型

相机和雷达显示由独立的 [uav_sim_visualization](../uav_sim_visualization/README.md) 提供。
默认 `x500_depth` 的控制参数不变。需要新模型时，分别在三个终端运行：

```bash
./run_obstacle_scene.sh --mid360
```

```bash
./run_rviz.sh
```

```bash
./run_ganyu_keyboard.sh --ros-args -p mass:=2.390307692307692
```

新模型增加 265 g 雷达负载，最后一条使用已有质量参数与新模型匹配，未改 GanYu 控制律或增益。
首次需要安装 RGL 依赖并构建显示包，参见上面链接；本机已完成该准备。

## 接入方式

控制核来源为 `sz_indoor_transport` commit `214d185c2f5658dca4e630ff1fc9027f4daf8c9d`
中的 `src/sz_indoor_controller/include/sz_indoor_controller/custom/quadrotor_ganyu.h`。
保留位置/速度/积分、期望推力方向、方向变化前馈与耦合角速度控制公式。
控制核不依赖 ROS 1：时间步长由仿真时钟提供，原来固定的航向目标改为起飞时的航向，
偏航误差做角度回绕；不使用原机 MAVROS 发送层的 X/Y 交换和多项式标定。

- 状态：PX4 `VehicleOdometry` 的 NED 位置、NED 速度、FRD→NED 四元数。
- 参考：七次分段多项式，键盘重新设置目标时保持位置、速度、加速度、jerk 连续。
- 控制：GanYu 输出总推力 N 和 FRD 角速度 rad/s。
- 输出：仅发 `OffboardControlMode.body_rate=true` 和 `VehicleRatesSetpoint`，
  不向 `TrajectorySetpoint` 或 `VehicleAttitudeSetpoint` 写入参考；位置闭环由 GanYu 运行。
- LAND：主动请求降落后立即停止 GanYu 角速度输出，交给 PX4 降落，不重新争夺 Offboard。

当前机型质量由 SDF 各 link 求和：`x500_base 2.0643076923 kg + OakD-Lite 0.061 kg`。
位置/速度/积分默认增益分别为 9.75、3.6、0.5（三轴），kr=120、hr=30，沿用源控制律配置；
推力上限改为适合当前模型的 30 N，发送层限制横滚/俯仰角速度 ±1 rad/s、偏航 ±0.5 rad/s。

X500 当前单电机推力常数 `k=8.54858e-6 N/(rad/s)^2`，电机速度输出范围为 150～1000 rad/s。
均匀稳态时，先计算 `speed=sqrt(T/(4*k))`，再计算
`signal=(speed-150)/850` 并限幅。发送推力是
`THR_MDL_FAC*signal² + (1-THR_MDL_FAC)*signal`，当前参数默认 `THR_MDL_FAC=0`。
PX4 自身执行该模型的逆变换；这种总推力换算在等转速稳态时成立，姿态修正时近似。
如果外部修改过电机或推力模型参数，需要同步修改本包参数；不会自动改写飞控参数。

起飞前仍检查有效位置、仿真时钟、反馈新鲜度、键盘心跳和其他设定值发布者。
已删除节点附加的自动降落条件：35° 倾角、反馈/键盘超时、Offboard/解锁等待超时、
仿真时间跳变、EKF 位置/姿态跳变及控制计算异常，均不会主动请求 LAND。
EKF 重置时只清除积分并保留目标；控制计算异常时只限频报错，不发布无效控制量。
仿真时间暂停/倒退或计算异常时仍发送 Offboard 模式心跳，不主动切换飞行模式。
`l`、`x` 和客户端退出流程仍可请求降落；PX4 自身的失效保护参数没有修改。
人工切换飞行模式后停止输出，不会循环抢回控制权。

日志在 `log/ganyu_control/<时间>_<随机后缀>/controller.log`。
`/ganyu/status` 输出阶段和状态，`/ganyu/control_debug` 是检查用数组：
`[T, normalized_thrust, omega(3), ref_p(3), ref_v(3), ref_a(3), ref_jerk(3), integral(3), p(3), v(3)]`。

## 本机验证（2026-10-25）

### 原小步试飞记录（删除附加自动降落条件之前）

- 仅编译新增包成功；匹配参数和偏航条件后，500 组输入与本机原 GanYu 源码逐项对照，
  最大数值差约 `7.1e-15`；该对照不表示整个键盘/任务流程与原项目完全一致。
- 悬停平衡、当前电机模型逆换算和位置/速度/加速度/jerk 连续衔接检查通过。
- 在未改动的 Walls + x500_depth 场景中，通过真实终端键盘客户端完成 `t` 起飞至 1.5 m、
  `w/d` 各移动 0.25 m、`e` 升至 1.75 m、连续 `s/a/q` 返回、空格悬停、`l` 降落，
  并在降落过程中按 Ctrl+C，最终确认落地上锁并正常退出。
- 最后一轮记录 4403 条角速度设定值；所有 Offboard 模式消息均为 body_rate，
  未收到任何 `TrajectorySetpoint`，确认由 GanYu 运行位置闭环。
- 该轮基于位置估计与时间参考的全程跟踪 RMSE 约 `0.018 m`，最大误差约 `0.053 m`；
  这是当前理想仿真的结果，不是真值定位精度或实机指标。
- 结果与原始采样在 `log/ganyu_control/keyboard_validation_final/`；
  原源码对照记录在 `log/ganyu_control/parity/`。
- 上述试飞没有覆盖持续长按和连续快速反向，不能据此判断这些操作已经验证稳定。

### 删除附加自动降落条件后的回归

- 重新编译通过，14 项隔离 ROS 检查通过；测试使用独立通信域中的模拟 PX4 消息，
  没有向实际场景发送控制指令，不属于新一轮 Gazebo 动力学试飞。
- Offboard 和解锁阶段分别等待超过 15 秒，均不发 LAND。
- 50° 倾角、EKF 大幅位置/姿态跳变，均不发 LAND。
- 里程计、位置、仿真时钟、飞控状态和键盘心跳超时，以及无效位置标志，均不发 LAND。
- 仿真时间前跳、倒退及非有限控制计算，均不发 LAND，并保留模式心跳。
- 键盘主动降落及异常后的主动降落仍可完成状态确认并返回 `IDLE`。
- 这次变更保留了原有的参考轨迹、目标范围、输出限幅和 PX4 自身保护参数。
