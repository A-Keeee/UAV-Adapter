# 上游历史实验流程

本文保留当前工作空间此前记录的原位置键盘、双机、视觉及拦截实验流程，供查阅已有代码。
当前项目的开发目标、单机场景和 GanYu 使用步骤以[主 README](../README.md)为准，
当前单机任务无需运行本文中的视觉检测、目标机或制导节点。
逐项手动启动当前单机环境见[手动启动说明](../README.md#manual-start)。

<a id="keyboard-control"></a>

## 原有 uav_keyboard_control 位置键盘流程

本节是保留的旧节点，其按键方向、默认高度和退出行为与 GanYu 不同。
使用主 README 中的 GanYu 时，不要同时启动本节的控制节点。

该包的可执行文件是 `uav_keyboard_control`，ROS 节点名是
`/drone_keyboard_control`，实现见 [uav_keyboard_control/src/main.cpp](../uav_keyboard_control/src/main.cpp)。
它通过 PX4 **Offboard 位置控制**操作第一架飞机 `/px4_1`，默认不控制 `/px4_2`。

#### 启动顺序

1. **终端 1：只启动仿真底座。** `setup_workspace.sh` 已复制资源；若尚未复制，先执行
   下列两个 `cp`。修改本仓库的脚本或世界文件后，也要重新复制。

   ```bash
   cd /home/ake/SJTU/适配终端/gazebo_sim
   cp ./run_swarm.sh ~/PX4-Autopilot/run_swarm.sh
   cp ./assets/gazebo_world/grass_world.sdf ~/PX4-Autopilot/Tools/simulation/gz/worlds/
   cd ~/PX4-Autopilot
   bash ./run_swarm.sh
   ```

   等待脚本提示全部就绪，保持这个终端运行。脚本会生成两架飞机，第二架可以留在地面。
   只启动仿真底座，不同时启动目标运动、视觉或 RL 控制节点。
   当前工作空间没有提供原项目的 `start.sh` 一键拦截入口。

2. **终端 2：启动 QGroundControl。** 等待地面站显示两架飞机连接，并检查预飞提示。

   ```bash
   ~/Applications/QGroundControl-v5.0.8-x86_64.AppImage
   ```

3. **终端 3：启动键盘节点。** 保持该终端获得键盘焦点，使用小写字母，无需按回车。
   应在真实交互终端运行，不要把标准输入重定向到文件或后台运行。

   ```bash
   cd /home/ake/SJTU/适配终端/gazebo_sim
   source /opt/ros/humble/setup.bash
   source install/setup.bash
   ros2 run uav_keyboard_control uav_keyboard_control
   ```

   节点启动后立即持续请求 **进入 Offboard 和解锁**，同时发送位置目标
   `(x, y, z) = (0, 0, -5)`、偏航角 `yaw = -3.14 rad`。
   请求被 PX4 接受后，飞机会自动飞向本地原点上方约 5 m 的位置，无需另按起飞键。
   日志 `Arm command send` 只表示发送了请求，是否成功需查看地面站或状态话题。

#### 按键含义：以当前源码和 NED 坐标为准

位置指令采用 PX4 本地 **NED** 坐标：`x` 向北、`y` 向东、`z` 向下，单位为米。
这些方向相对于本地世界坐标，而不是机头或 Gazebo 观察相机的方向。
每读取一次按键，目标位置对应分量变化 **0.1 m**：

- `w`：`x += 0.1`，向北移动目标点；同时将目标偏航设为 `-3.14 rad`。
- `s`：`x -= 0.1`，向南移动目标点；同时将目标偏航设为 `-3.14 rad`。
- `a`：`y += 0.1`，向东移动目标点。
- `d`：`y -= 0.1`，向西移动目标点。
- `q`：`z += 0.1`，目标点**下降** 0.1 m。
- `e`：`z -= 0.1`，目标点**上升** 0.1 m。

**源码中 `q/e` 的 `Moving Up/Down` 日志与实际升降相反，以上按坐标值解释。**
默认 `yaw ≈ -π` 时机头朝南，因此日志中的 Forward/Left 也不能作为机体方向解释。
例如，从初始设定值连续按 10 次 `w`，目标变为 `(1, 0, -5)`；再按 5 次 `e`，
目标变为 `(1, 0, -5.5)`。长按依赖操作系统的按键重复；松开后继续保持最后目标点。
当前节点没有独立的偏航调节键、暂停键或自动降落键。

#### 退出与当前实现限制

- `x` 只退出键盘读取循环，**后台线程仍会发送 Offboard、解锁和位置指令**，不能用它结束控制。
  结束试验使用键盘终端的 `Ctrl+C`，确认进程退出；若终端回显异常，可执行 `stty sane`。
  节点没有完善的线程退出清理，退出时可能出现 `terminate` 报错。
- 停止节点不会发送降落指令。停止后 PX4 会按 Offboard 丢失保护参数处理；需要降落时，
  停止键盘节点后通过 QGroundControl 对应飞机选择降落。也可以在仿真终端按 `Ctrl+C`
  结束整个场景，下一次启动重新试验。持续运行键盘节点时，它会反复请求 Offboard 和解锁。
- 当前控制循环实际写的是 `sleep_for(microseconds(10))`，不是注释中的 100 ms，
  会高频发送控制和解锁请求；它是一个基础示例，尚未实现按状态确认后停止重发的状态机。
- 当前状态订阅仍使用旧话题 `/px4_1/fmu/out/vehicle_status` 和默认可靠 QoS，
  且构造函数被键盘循环阻塞，正常键盘操作期间尚未进入 `rclcpp::spin()`。
  因此不要依赖它的状态回调判断是否解锁；本机 PX4 v1.16 的实际状态话题是
  `/px4_1/fmu/out/vehicle_status_v1`，可用下面的命令独立确认。
- 同一架飞机一次只运行一个控制节点。不要与 `uav_vehicle_controller`、
  `uav_png_intercept`、`uav_vision_png`、`uav_rl_guidance` 或 `uav_ibvs_control` 同时控制 `/px4_1`。
  其中 `uav_vehicle_controller` 是另一个速度控制示例，与此包的位置控制按键含义不同。

```bash
# 另开终端，在本仓库根目录加载 ROS 2 和工作空间。
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic echo /px4_1/fmu/out/vehicle_status_v1 \
  px4_msgs/msg/VehicleStatus --qos-reliability best_effort --once
ros2 topic echo /px4_1/fmu/out/vehicle_local_position \
  px4_msgs/msg/VehicleLocalPosition --qos-reliability best_effort --once
```

解锁状态应为 `arming_state: 2`（`ARMING_STATE_ARMED`），Offboard 应为
`nav_state: 14`（`NAVIGATION_STATE_OFFBOARD`）。若未起飞，先检查 QGC 连接、预飞错误、
本地位置/速度的有效标志，以及 `~/PX4-Autopilot/drone1.log`，不要只看节点的发送日志。

## 原有双机自动拦截流程

以下流程依赖完整视觉/推理环境，独立于前面的 Walls + GanYu 单机入口。
每个直接执行 `ros2 run` 的终端都应先加载 `/opt/ros/humble/setup.bash` 和本仓库 `install/setup.bash`。

0. **启动仿真环境**:
   在本仓库根目录执行，启动前将原有双机仿真资源复制到 PX4-Autopilot 目录下：

   ```bash
   # 步骤1: 将本仓库的 run_swarm.sh 复制到 PX4-Autopilot 根目录（若尚未复制）
   cp ./run_swarm.sh ~/PX4-Autopilot/

   # 步骤2: 将自定义世界文件复制到 PX4 Gazebo 世界目录
   cp ./assets/gazebo_world/grass_world.sdf ~/PX4-Autopilot/Tools/simulation/gz/worlds/
   ```

   > 首次使用或修改脚本/地图后复制；资源未变化时，之后直接在 PX4-Autopilot 目录启动：
   ```bash
   cd ~/PX4-Autopilot
   ./run_swarm.sh
   ```

   该脚本会自动启动 Gazebo 仿真服务器、两架无人机（px4_1 拦截机 + px4_2 目标机）、MicroXRCEAgent 以及相机话题桥接。
   启动和退出时，会额外按完整命令行清理当前用户的 `gz sim` 服务端和图形界面，
   避免系统进程名为 `ruby` 而被原来的 `killall` 漏掉。启动流程保持原样，
   不检查 `px4_msgs`，无需为启动此脚本额外加载本工作空间。

   **启动目标节点前，打开 QGroundControl 并等待两架飞机通过 UDP 14550 自动连接。**
   本机安装位置为：

   ```bash
   ~/Applications/QGroundControl-v5.0.8-x86_64.AppImage
   ```

   Ubuntu 22.04 使用 [QGroundControl 5.0 系列](https://docs.qgroundcontrol.com/Stable_V5.0/en/qgc-user-guide/getting_started/download_and_install.html)。
   当前 PX4 配置要求地面站连接；否则 `drone2.log` 会报告
   `Preflight Fail: No connection to the ground control station` 并拒绝解锁。
   `Arm command sent` 仅表示目标节点发送了请求。目标节点仅请求一次解锁，
   若连接地面站前已经启动过该节点，连接成功后应在该节点终端按 Ctrl+C 并重新运行。


1. **启动目标机仿真**:
   ```bash
   # 默认圆周运动
   ros2 run uav_target_sim uav_target_sim

   # 或通过参数选择运动模式和活动范围
   ros2 run uav_target_sim uav_target_sim --ros-args -p motion_mode:=sinusoidal -p max_range:=10.0

   # 也可使用 launch 文件
   ros2 launch uav_target_sim target_sim.launch.py motion_mode:=random_walk
   ```

   **支持的目标运动模式**：

   - `circle`：匀速圆周，默认 R=5 m、w=0.5 rad/s。
   - `sinusoidal`：前向匀速与侧向正弦加速度组合。
   - `random_walk`：速度随机扰动，原流程限速 3 m/s。

   > `max_range` 参数（默认10m）控制目标活动范围，防止目标飞出拦截机视野。

2. **启动视觉制导拦截**:
   ```bash
   # 推荐直接运行（使用 .hpp 中的默认参数）
   ros2 run uav_vision_png uav_vision_png

   # 或通过 launch 文件加载 config/params.yaml 中的参数
   ros2 launch uav_vision_png vision_png.launch.py
   ```

   > **参数配置说明**：`uav_vision_png` 节点的参数优先级为：
   >   **launch 命令行 > params.yaml > .hpp 成员初始值**
   >
   > - 使用 `ros2 run` 直接启动：参数取 `.hpp` 中的默认值，**`params.yaml` 不会自动加载**，需手动传参 `--ros-args -p 参数名:=值`
   > - 使用 `ros2 launch` 启动：launch 文件自动加载 `config/params.yaml`，YAML 中的值会覆盖 `.hpp` 默认值
   > - 调试时可用 `ros2 param get /uav_vision_png <参数名>` 查看当前实际生效的值
   > - 可调参数详见 `uav_vision_png/config/params.yaml`，含 PNG 增益、视场补偿、起飞高度等

2-备. **启动 RL 学习制导拦截 (替代方案)** 🆕:
   `uav_rl_guidance` 是基于强化学习（BC + PPO）训练的 GRU 策略制导节点，用于取代传统 PNG 算法。
   它与 `uav_vision_png` 保持相同的话题接口、状态机和 CSV 统计格式，可无缝替换。
   使用 ONNX Runtime 推理，零 PyTorch 依赖，纯 C++ 实现。

   ```bash
   # 方式1：launch 启动（自动加载 config/params.yaml）
   ros2 launch uav_rl_guidance rl_guidance.launch.py

   # 方式2：直接运行（使用代码默认参数）
   ros2 run uav_rl_guidance uav_rl_guidance

   # A/B 基线模式：全程使用内置 PNG，不加载策略模型（与 uav_vision_png 等价）
   ros2 launch uav_rl_guidance rl_guidance.launch.py fallback_png:=true

   # 台架调试：跳过起飞，直接进入 SEARCHING
   ros2 launch uav_rl_guidance rl_guidance.launch.py bench_test:=true
   ```

   > **架构说明**：RL 策略在 INTERCEPT 阶段输出速度指令；若策略出现异常（watchdog），自动回退到内置 PNG 控制器。
   > 策略模型（ONNX `.onnx`）存放在 `uav_rl_guidance/models/` 下，
   > 配套训练项目 `End2End-FPVIntercept` 需要单独准备，当前目录未附带；
   > 其上游项目为 [AeroIntercept](https://github.com/Eaglewzw/AeroIntercept)。
   > 当前节点使用 15 维观测的 GRU 策略，更新兼容的 `uav_rl_guidance/models/policy.pt` 后，
   > 在本仓库根目录执行 `python3 uav_rl_guidance/src/export_onnx.py`；端到端图像策略需使用其对应的部署接口。

3. **启动视觉检测**:
   ```bash
   ros2 run uav_vision_dectect uav_vision_dectect
   ```

