# x500 相机 + MID-360 仿真模型

通过 SDF `include merge` 复用本机 PX4 的 `x500_depth`，保留原机架、电机和 OakD-Lite 相机。
独立增加 `mid360_link` 和固定关节，原 PX4 模型文件不变。

- MID-360 外壳使用简化几何：65×65×60 mm、265 g；惯性按均匀长方体近似。
- 安装位置相对 `base_link` 为 `(0, 0, 0.12)` m，雷达坐标轴与机体 FLU 一致。
- 新模型总质量为 `2.125307692307692 + 0.265 = 2.390307692307692` kg。
- [RGLGazeboPlugin](https://github.com/RobotecAI/RGLGazeboPlugin) 的 `Livox Mid360` 预设负责扫描，
  使用预录扫描方向的 40 段交替序列，10 Hz 更新，配置量程 0.1～40 m。
- Gazebo 输出 `/mid360/points`，ROS 桥接为 `/lidar/points`，帧名 `mid360_link`。
- 输出为标准 XYZ/强度点云，不含 Livox 自定义消息或逐点时间戳。

外壳尺寸和质量参考 [Livox 官方规格](https://www.livoxtech.com/mid-360/specs)。
这不是厂家提供的完整动力学或硬件误差模型。

先执行仓库根目录的 `./scripts/setup_mid360_sim.sh` 安装固定版本 RGL 运行库和扫描序列，
再用 `./run_obstacle_scene.sh --mid360` 启动。该入口在本次日志目录生成带 RGL 管理插件的
世界副本，不覆盖原 Walls 场景。`./run_rviz.sh` 可显示图像和点云。

使用已有 GanYu 键盘入口时传入新质量，其他参数沿用默认值：

```bash
./run_ganyu_keyboard.sh --ros-args -p mass:=2.390307692307692
```

完整启动、手动步骤及本机验证记录见[主 README](../../../README.md#rviz-sensors)。
