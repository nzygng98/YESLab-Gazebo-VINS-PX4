# images/ —— 实验截图

本目录存放实验证据截图（Gazebo 场景、mavros 状态、RViz 轨迹、里程计输出）。

## 已归档的 4 张关键截图

| 文件 | 拍的是什么 | 关键证据 |
|---|---|---|
| `01_gazebo_cafe_rtf0.98_fps62.jpg` | VMware 里的 Ubuntu + Gazebo `indoor1`（cafe）场景；另一终端在 echo `/mavros/local_position/pose` | 底部状态栏 `Real Time Factor: 0.98`、`Iterations: 841507`、`FPS: 62.1`；`header.stamp.secs = 2590`（**非 0**，仿真时间正常推进） |
| `02_mavros_state_offboard_armed.jpg` | `timeout … rostopic echo -n1 /mavros/state` | **`connected: True`、`armed: True`、`mode: "OFFBOARD"`**、`system_status: 4`；下方 `move_drone.py` 打印「到位（偏差 0.245 m < 0.25）」 |
| `03_rviz_vio_trajectory.jpg` | RViz（`default.rviz`）里叠加显示 VINS 输出 | 网格地面上的**绿色 VIO 轨迹**与密集特征点；`ROS Time: 3373.84`、`ROS Elapsed: 3024.58`；下方「✅ [平移] 到位（偏差 0.229 m < 0.25）」 |
| `04_local_position_1.49m.jpg` | `rostopic echo -n1 /mavros/local_position/pose`（完整消息） | `frame_id: "map"`；`position: x=0.2553, y=0.0289, z=1.4877`（**已飞到 1.49 m** 目标高度）；`orientation` 四元数完整，`w=-0.99998`（≈ 180° yaw） |

> 这 4 张原图也同时以原始文件名（`2dcebd9d…jpg` 等）保存在**仓库根目录**，
> 是作者通过 GitHub 网页直接上传的；本目录里的是**按内容重命名后的归档副本**，内容完全相同。
>
> 演示视频（`284ce96eb0f00d7f7c713d2fe0667554.mp4`，3.6 MB）同样在仓库根目录。

## 建议后续补充的截图

| 文件名 | 内容 | 怎么采 |
|---|---|---|
| `05_gazebo_iris_visible.jpg` | Gazebo 里能看清 `iris_0` 机体 | `1_sim.sh` 后对 Gazebo 窗口截图 |
| `06_px4_home_set.jpg` | PX4 终端出现 `INFO [tone_alarm] home set` | 终端截图 |
| `07_vins_init_finish.jpg` | VINS 打印 `Initialization finish!` | 终端截图 |
| `08_vision_pose_stamp.jpg` | `/mavros/vision_pose/pose/header/stamp` 的 `secs` 非 0 | 终端截图 |
| `09_rviz_pointcloud.jpg` | RViz 里的 `/vins_estimator/point_cloud` | `5_rviz.sh` 后截图 |
| `10_before_after_segfault.jpg` | 修复前 `incorrect plugin type` + `Segmentation fault` 对比修复后无报错 | 按 `docs/ROOT_CAUSE_GPS_PLUGIN.md` 复现 |

## 在 README 里引用

```markdown
![Gazebo indoor1 场景（RTF 0.98 / FPS 62）](images/01_gazebo_cafe_rtf0.98_fps62.jpg)
```

