# 段错误根因定案：XTDrone `iris_*` 模型的错位 `gps_plugin` 声明

> 日期：2026-10-10
> 定位手段：A/B 对照实验 + gdb 完整 backtrace（非推测）

---

## 1. 结论（一句话）

**段错误与相机、与硬件都无关。** 真因是 XTDrone 的 `iris_stereo_camera.sdf` 里
把 PX4 的 `libgazebo_gps_plugin.so` **写在了 `<model>` 直接子级**（ModelPlugin 席位），
而该 `.so` 实际注册为 **SensorPlugin**。Gazebo 判定「插件类型不符」后 `delete plugin`，
此时插件的 `Load()` 从未执行、成员 `world_` 是空指针，析构函数一访问就
**NULL 解引用 → SIGSEGV**。

---

## 2. 铁证：gdb backtrace

```
[Err] [Model.cc:1145] Model[iris_0] is attempting to load a plugin, but detected an
      incorrect plugin type. Plugin filename[libgazebo_gps_plugin.so] name[gps_plugin]
Thread 60 "gzserver" received signal SIGSEGV, Segmentation fault.

#0  gazebo::physics::World::IsPaused() const      at libgazebo_physics.so.11
#1  gazebo::physics::World::Reset()
#2  gazebo::GpsPlugin::~GpsPlugin()
#3  gazebo::GpsPlugin::~GpsPlugin()               at gazebo_gps_plugin.cpp:39
#4  (libgazebo_physics.so.11)                     ← delete plugin
#5  gazebo::physics::Model::LoadPlugin(...)
#6  gazebo::physics::Model::LoadPlugins(unsigned int)
#7  gazebo::physics::World::ProcessFactoryMsgs()
#8  gazebo::physics::World::ProcessMessages()
#9  gazebo::physics::World::Step()
#10 gazebo::physics::World::RunLoop()

故障指令：  <World::IsPaused() const+4>:  mov  0x10(%rdi),%rax
寄存器：    rip = ...IsPaused+4 , rbp = 0x0   → rdi(NULL) 解引用
```

### 类型不符的证据

| 位置 | 内容 |
|---|---|
| `Tools/sitl_gazebo/include/gazebo_gps_plugin.h:62` | `class GAZEBO_VISIBLE GpsPlugin : public SensorPlugin` |
| `Tools/sitl_gazebo/src/gazebo_gps_plugin.cpp:34` | `GZ_REGISTER_SENSOR_PLUGIN(GpsPlugin)` |
| `models/iris_stereo_camera/iris_stereo_camera.sdf:412` | `<plugin name='gps_plugin' filename='libgazebo_gps_plugin.so'>` ← 写在 `<model>` 下 |
| 该模型 | **没有任何 `type="gps"` 的 `<sensor>`** |

对照：PX4 官方 `models/iris/iris.sdf` **不含** `libgazebo_gps_plugin.so`。
⇒ 用官方 `iris` 的任何启动方式都不崩；用 XTDrone 的 `iris_*` 都崩。

XTDrone 的 `iris_realsense_camera` / `iris_downward_camera` / `iris_2d_lidar` 等
**同样各有 1 处**这个错位声明，是共性问题。

---

## 3. 为什么之前误判成「相机挂机体」

之前的对照实验**混杂了两个变量**：

| | 相机 | `GAZEBO_PLUGIN_PATH` 含 `build_gazebo`? | PX4 插件 | 结果 |
|---|---|---|---|---|
| 旧「能跑」 | 悬浮（不在机体） | ❌ 不含 | **全部加载失败** | 不崩 |
| 旧「必崩」 | 挂机体 | ✅ 含 | 全部加载成功 | 崩 |

插件路径缺失时，`libgazebo_gps_plugin.so` 根本没被加载（`Failed to load plugin`），
自然崩不了 —— 于是「相机挂机体」被误当成罪魁。

### 干净的 A/B 对照（本次，唯一变量 = 相机）

两次运行其余条件完全相同（Xvfb 提供真实 X、`build_gazebo` 在插件路径里）：

```
########## RUN A_NO_CAMERA  (相机挂机体=NO) ##########
  gzserver 存活            : 0   (0 = 已崩)
  Segmentation fault       : 1
  Rendering disabled       : 0   (渲染正常)
  CameraSensor 创建失败数   : 0
  PX4 插件加载失败数        : 0
  崩溃点: [Err] Model.cc:1145 ... libgazebo_gps_plugin.so ...
```

⇒ **相机不在机体上，照样崩在同一个位置。相机无罪。**


---

## 4. 不是硬件问题

1. 崩溃发生在**模型加载阶段，还没渲染第一帧** → 与 GPU 负载无关。
2. 渲染器是 `llvmpipe`（纯软件，无 GPU）也能稳定跑到模型加载 —— 硬件不是瓶颈。
3. 崩溃**确定性复现**（同组合必崩）。硬件故障是偶发/随负载波动的。
4. 已排除：坏插件、ABI 不匹配、内存/CPU 不足、世界太重、GL 驱动。

---

## 5. 「路径 A/B/C 互斥」之谜的真相

**不是** "`gazebo_ros` × PX4 `build_gazebo` 冲突"。
是 **XTDrone 的 `iris_*` 模型文件没跟上 PX4 v1.13 的 `gps_plugin` API 变更**。

---

## 6. 修复

删除 `iris_stereo_camera.sdf` 里那处错位声明（PX4/XTDrone 两份都要改，
因为 `GAZEBO_MODEL_PATH` 里 XTDrone 目录优先级更高）：

```xml
<!-- 删掉这 4 行 -->
<plugin name='gps_plugin' filename='libgazebo_gps_plugin.so'>
  <robotNamespace/>
  <gpsNoise>1</gpsNoise>
</plugin>
```

参考实现：`/tmp/cline/fix_apply.py`

### 修复后实测（目标配置：相机挂机体 + 标准 gazebo_ros 插件）

| 检查项 | 结果 |
|---|---|
| Segmentation fault | **0** ✅ |
| `incorrect plugin type` | **0** ✅ |
| 剩余 `[Err]` | **0** ✅ |
| gzserver | 存活 ✅ |
| `CameraSensor` 创建 | 4/4 成功 ✅ |
| PX4 | `Simulator connected on TCP port 4560` ✅ |
| `/iris_0/stereo_camera/left/image_raw` | 有发布者 + 真实帧（`seq` 增长、`stamp` 非 0、752×480）✅ |
| `/iris_0/imu_gazebo` | 真实数据 ✅ |
| 相机挂机体 | `<joint type='fixed'>` 刚性连接 `base_link` ✅ |
| 插件 | **标准** `libgazebo_ros_camera.so` / `libgazebo_ros_imu_sensor.so` ✅ |

---

## 7. 第二个独立坑：gzserver 打不开 X ⇒ 相机永远 0 帧

```
[Err] [RenderEngine.cc:749] Can't open display: :0
[Wrn] [RenderEngine.cc:89]  Unable to create X window. Rendering will be disabled
[Err] [CameraSensor.cc:125] Unable to create CameraSensor. Rendering is disabled.   ← ×4
```

**gzserver 即使不跑 gzclient 也必须有一个可用的 X display**，否则 OGRE 被整体禁用，
`CameraSensor` 根本创建不出来 —— 表现就是「话题在、publisher 在、就是不出图」。
这与之前那个「OGRE 材质脚本损坏渲染管线」是两个**独立**的成因。

判定：`xdpyinfo >/dev/null 2>&1 && echo X-OK || echo X-DEAD`

---

## 8. 必须配对的环境变量（`.bash_history` 里的可用组合）

```bash
export PX4=/home/nzyg/PX4-Autopilot
export ROS_PACKAGE_PATH="$PX4:$PX4/Tools/sitl_gazebo:$ROS_PACKAGE_PATH"
source "$PX4/Tools/setup_gazebo.bash" "$PX4" "$PX4/build/px4_sitl_default"
export GAZEBO_MODEL_PATH="$PX4/Tools/sitl_gazebo/models:/usr/share/gazebo-11/models"
export GAZEBO_PLUGIN_PATH="/opt/ros/noetic/lib:/usr/lib/x86_64-linux-gnu/gazebo-11/plugins"
```

- `ROS_PACKAGE_PATH` 必须含 **`$PX4` 本身** —— `$PX4/package.xml` 的 `<name>` 就是 `px4`；
  只加 `Tools/sitl_gazebo`（包名 `mavlink_sitl_gazebo`）会报
  `RLException: ... is neither a launch file in package [px4]`。
- `setup_gazebo.bash` **只设 GAZEBO_\*/LD_LIBRARY_PATH，不设 ROS_PACKAGE_PATH**。
- 它会把 `${BUILD_DIR}/build_gazebo` 追加进 `GAZEBO_PLUGIN_PATH` —— 少了它
  PX4 插件全部 `Failed to load plugin`（电机/mavlink/IMU 全废）。

---

## 9. 尚未处理的一项（与崩溃无关）

`mavros: connected: False` —— MAVLink UDP 端口错配：

| 端 | 端口 |
|---|---|
| PX4 监听 | `14580`（mavlink instance 0）、`18570`、`14280`、`13030` |
| mavros 监听 | `24540` |
| `indoor1.launch` 的 `fcu_url` | `udp://:24540@localhost:34580` |

**修复**：把 `~/PX4-Autopilot/launch/indoor1.launch` 里
`<arg name="fcu_url" default="udp://:24540@localhost:34580"/>`
改成 PX4 SITL 默认配对 `udp://:14540@localhost:14580`
（即 `~/scripts/mavros_iris0.launch` 之前验证成功过的那组端口）。

---

## 10. 备份文件索引

| 备份 | 含义 |
|---|---|
| `iris_stereo_camera.sdf.bak_before_attach` | 挂相机之前的版本（无 include） |
| `iris_stereo_camera.sdf.iso_bak` | 隔离实验前版本 |
| `stereo_camera/model.sdf.bak_ros` | 单目 camera 插件版 |
| `stereo_camera/model.sdf.multicamera_orig` | 最初 multicamera 版（Gazebo 11 不支持） |
| `stereo_camera/model.sdf.disabled_plugins` | 隔离测试时注释 gazebo_ros 插件的版本 |
| `worlds/indoor1.world.bak[2-4]` / `.bak_before_strip` / `.with_floating_cam` | world 各阶段 |
| `Tools/sitl_run.sh.bak` | sitl_run.sh 原始版（10-09 20:06） |

---

## 11. 本次会话引入的环境改动（请知悉）

| 改动 | 说明 / 回滚 |
|---|---|
| 安装 `xvfb` | `sudo apt-get install -y xvfb`；不需要可 `sudo apt-get remove --purge xvfb` |
| `/proc/sys/kernel/core_pattern` | 临时改过以抓 core；**已回滚**为 apport |
| 后台仍在运行 | `Xvfb :99`、`roslaunch px4 indoor1.launch`、`gzserver`、`mavros` |
