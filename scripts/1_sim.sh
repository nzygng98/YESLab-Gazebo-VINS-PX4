#!/bin/bash
# =============================================================================
#  1_sim.sh —— 终端 1 : 启动 Gazebo + PX4 SITL
# -----------------------------------------------------------------------------
#  用法:
#     ./1_sim.sh                 默认: 用 indoor1_no_mavros.launch (不含 mavros)
#     ./1_sim.sh --with-mavros   等价于: roslaunch px4 indoor1.launch gui:=true
#     ./1_sim.sh --no-gui        无 Gazebo 窗口 (gui:=false)
#
#  ⚠️⚠️ 为什么默认不是 indoor1.launch ?⚠️⚠️
#     ~/PX4-Autopilot/launch/indoor1.launch 末尾【内嵌】了一个 mavros:
#         <include file="$(find mavros)/launch/px4.launch">
#           <arg name="fcu_url" value="udp://:14540@localhost:14580"/>
#         </include>
#     它会以 ns=iris_0 起一个 mavros 并独占 UDP 14540。
#     而 2_mavros.sh 用的是 fcu_url="udp://:14540@127.0.0.1:14580" —— 同样是 14540。
#     => 两个 mavros 抢同一个端口, 第二个拿不到链路, 整个链路就废了。
#
#     所以本脚本默认走 indoor1_no_mavros.launch (功能完全一样, 只去掉内嵌 mavros),
#     让「1_sim.sh = Gazebo+PX4」「2_mavros.sh = mavros」真正解耦。
#     若你确实想要 indoor1.launch 的原样行为(自带 ns=iris_0 的 mavros),
#     用 ./1_sim.sh --with-mavros, 但【不要再运行 2_mavros.sh】,
#     且 4_bridge.sh / move_drone.py 需要用 ns=/iris_0。
# =============================================================================

# ⚠️ 注意: 在【脚本】里 `source ~/.bashrc` 是【无效】的 ——
#    .bashrc 顶部有 interactivity 守卫:
#        case $- in *i*) ;; *) return;; esac
#    非交互 shell (脚本) 会在第一行就 return, 里面的 export 一个都不会执行。
#    因此下面必须【显式】source ROS 并显式 export PX4/Gazebo 变量。
#    仍然保留 source ~/.bashrc 一行, 只为遵守约定 / 兼容你自己改过的 .bashrc。
set +u

source /opt/ros/noetic/setup.bash 2>/dev/null
source "$HOME/catkin_ws/devel/setup.bash" 2>/dev/null

export PX4="$HOME/PX4-Autopilot"
export ROS_PACKAGE_PATH="$PX4:$PX4/Tools/sitl_gazebo:$ROS_PACKAGE_PATH"
export GAZEBO_MODEL_PATH="$PX4/Tools/sitl_gazebo/models:/usr/share/gazebo-11/models"
export GAZEBO_PLUGIN_PATH="/opt/ros/noetic/lib:/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:$PX4/build/px4_sitl_default/build_gazebo"
export LD_LIBRARY_PATH="$PX4/build/px4_sitl_default/build_gazebo:$LD_LIBRARY_PATH"

source "$HOME/.bashrc" 2>/dev/null   # 非交互下为 no-op, 见上方说明

# ------------------------------- 参数解析 --------------------------------
LAUNCH="indoor1_no_mavros"
LAUNCH_FILE="$HOME/scripts/indoor1_no_mavros.launch"
GUI="true"
for a in "$@"; do
    case "$a" in
        --with-mavros) LAUNCH="indoor1"; LAUNCH_FILE="px4 indoor1.launch" ;;
        --no-gui)      GUI="false" ;;
        -h|--help)     sed -n '2,32p' "$0"; exit 0 ;;
        *) echo "未知参数: $a  (可用: --with-mavros / --no-gui)"; exit 1 ;;
    esac
done

# ------------------------------- 预检 ------------------------------------
echo "================================================================"
echo " 1_sim.sh —— Gazebo + PX4 SITL"
echo "================================================================"
echo "  PX4               = $PX4"
echo "  ROS_PACKAGE_PATH  = $ROS_PACKAGE_PATH"
echo "  launch            = $LAUNCH_FILE"
echo "  gui               = $GUI"
echo "  DISPLAY           = ${DISPLAY:-<unset!>}"
if [ -z "$DISPLAY" ]; then
    echo
    echo "  ❌ DISPLAY 未设置 —— 必须在 VMware 桌面原生终端里运行本脚本!"
    echo "     否则 gzserver 报 'Can't open display' -> 'Rendering is disabled'"
    echo "     -> 4 个 CameraSensor 全部创建失败 -> 相机永远 0 帧。"
    exit 1
fi
if ! xdpyinfo >/dev/null 2>&1; then
    echo
    echo "  ❌ DISPLAY=$DISPLAY 不可达 (xdpyinfo 失败)。请确认在桌面会话里运行。"
    exit 1
fi
echo "  X 显示           = ✅ 可达"
echo "  rospack find px4  = $(rospack find px4 2>/dev/null || echo '❌ 找不到, 检查 ROS_PACKAGE_PATH')"
echo "  rospack find mavlink_sitl_gazebo = $(rospack find mavlink_sitl_gazebo 2>/dev/null || echo '❌')"
echo "----------------------------------------------------------------"
echo "  启动后请另开终端观察:  /iris_0/stereo_camera/left/image_raw"
echo "  退出: Ctrl+C"
echo "================================================================"
echo

cd "$PX4" || exit 1
if [ "$LAUNCH" = "indoor1" ]; then
    exec roslaunch px4 indoor1.launch gui:="$GUI"
else
    exec roslaunch "$LAUNCH_FILE" gui:="$GUI"
fi
