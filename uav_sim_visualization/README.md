# 相机与 MID-360 的 RViz2 显示

本包仅发布可视化 TF 并启动 RViz2，不发布任何飞行控制指令。

```bash
cd /home/ake/SJTU/适配终端/gazebo_sim
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --symlink-install --packages-select uav_sim_visualization
./scripts/setup_mid360_sim.sh
```

不同终端分别运行：

```bash
./run_obstacle_scene.sh --mid360
```

```bash
./run_rviz.sh
```

新模型 `x500_mid360_1` 在原 x500 相机机型上增加 265 g 的 MID-360。
雷达使用 [RGLGazeboPlugin](https://github.com/RobotecAI/RGLGazeboPlugin) 的
`Livox Mid360` 预设及 40 段交替扫描模式，10 Hz 发布 `PointCloud2`。
这是仿真点云，不包含 Livox 驱动自定义消息、逐点时间戳或硬件网络数据包。
RGL 需要 NVIDIA RTX 显卡；依赖安装在用户目录，不修改系统驱动或 PX4 模型。

- 图像：`/camera/image`；相机内参：`/camera/camera_info`。
- 雷达：Gazebo `/mid360/points` → ROS `/lidar/points`。
- 点云坐标系：`mid360_link`，安装在 `base_link` 上方 0.12 m。
- RViz 固定坐标系：`odom`；显示节点将 PX4 NED/FRD 里程计转换为 ENU/FLU TF。
- 配置文件：`config/sensors.rviz`，同时启用 Image 与 PointCloud2，接收 QoS 为 Best Effort。

需要键盘飞行时，新机型总质量约为 2.390307692 kg，使用已有参数启动 GanYu：

```bash
./run_ganyu_keyboard.sh --ros-args -p mass:=2.390307692307692
```

原来的 `./run_obstacle_scene.sh` 继续使用未增加雷达的 `x500_depth`，原控制器参数不变。
直接手动启动可视化时，先加载 ROS/工作空间，然后执行：

```bash
ros2 launch uav_sim_visualization sensors.launch.py
```

仅发布 TF、不打开 RViz：

```bash
ros2 launch uav_sim_visualization sensors.launch.py rviz:=false
```

控制器、键盘与可视化节点的手动启动命令见
[主 README](../README.md#manual-start)。

本机于 2026-10-27 验证：1920×1080 RGB 图像、10 Hz 点云同时显示，
116 帧点云的消息时间均可查询到 TF。使用新质量参数完成六方向小步移动、
约 64 秒持续悬停与人工降落上锁；单次参考位置跟踪 RMSE 约 0.0144 m。
这不表示已验证快速操作或自动避障。
