# 离线静态资源

- `warehouse` 来自 AWS RoboMaker Small Warehouse，提交 `ee0af733315e78432408c3cd98d378ecee5f767c`；原许可证见 `warehouse/LICENSE`。完整归档 SHA256：`c536d4a027f42bdac1a15d751ad8fb37296e14871e5af909f01207a854002211`。
- `forest` 的 Pine Tree 6、Oak tree 7：OpenRobotics，CC0-1.0；grasspatch 1：hexarotor，CC-BY-4.0。下载地址、版本、校验值和署名保存在 `forest/ORIGIN.json`。
- PX4 原始树林布局固定到 Gazebo models 提交 `e05f4312d3f28aa621157610584a4870406cb6d3`，原文件、来源与许可证分别见 `forest/forest.upstream.sdf`、`forest/forest.world.origin.json`、`forest/PX4_MODELS_LICENSE`。

仓库适配修正 `file://models/` 路径，移除静态物体中未使用且不满足 Harmonic 检查的惯量描述。网格、纹理及布局保留。世界系统、长树林和传感器模型由 `scripts/prepare_autonomy_assets.py` 生成。

静态模型还为碰撞表面增加完全透明的 `rgl_collision_surface_*` 可视节点。RGL 仅导入可视几何；这些节点让雷达射线与物理碰撞表面一致，避免箱堆的简化碰撞外壳成为传感器不可见的障碍。原有可视／碰撞网格与物理属性保留。模型校验值和 RGL 对应源码链接见 `RGL_COLLISION_SURFACES.json`。

导航节点不得读取本目录或 `assets/evaluation`。这些资源仅用于仿真、资源校验和独立评分。
