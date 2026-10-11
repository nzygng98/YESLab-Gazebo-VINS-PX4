# images/ —— 截图目录

本目录用于存放实验截图（Gazebo 场景、RViz 轨迹、终端输出等）。

## 当前状态

截图与演示视频已由作者**手动上传到 GitHub 网页**，位于仓库根目录 / Release：

- 视频：https://github.com/nzygng98/YESLab-Gazebo-VINS-PX4/blob/main/284ce96eb0f00d7f7c713d2fe0667554.mp4

建议后续把截图放到本目录，并在根目录 `README.md` 里用相对路径引用，例如：

```markdown
![Gazebo 中的 iris_0（亮蓝机身 + 亮橙旋翼）](../images/gazebo_iris.png)
![RViz 中的 VINS 轨迹](../images/rviz_traj.png)
```

## 建议采集的截图清单

| 文件名 | 内容 | 怎么采 |
|---|---|---|
| `gazebo_iris.png` | Gazebo 里能看清 iris_0（亮蓝机身/亮橙旋翼） | `1_sim.sh` 后对 Gazebo 窗口截图 |
| `gazebo_topics.png` | `rostopic list \| grep iris_0` 的输出 | 终端截图 |
| `mavros_state.png` | `/mavros/state` 显示 `connected: True` | `timeout -s KILL 15 rostopic echo -n1 /mavros/state` |
| `vins_init.png` | VINS 打印 `Initialization finish!` | 终端截图 |
| `vision_pose.png` | `/mavros/vision_pose/pose` 且 `header.stamp` 非 0 | 终端截图 |
| `offboard_armed.png` | `move_drone.py` 输出 `mode = OFFBOARD` + `arm success` | 终端截图 |
| `rviz_traj.png` | RViz 里的 VIO 轨迹与点云 | `5_rviz.sh` 后截图 |
| `before_after_segfault.png` | 修复前 `incorrect plugin type` + `Segmentation fault` 对比修复后无报错 | 按 `docs/ROOT_CAUSE_GPS_PLUGIN.md` 复现 |
