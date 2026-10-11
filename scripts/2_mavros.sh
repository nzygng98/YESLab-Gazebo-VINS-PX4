#!/bin/bash
# =============================================================================
#  2_mavros.sh —— 终端 2 : 启动 MAVROS (root ns -> /mavros/...)
# -----------------------------------------------------------------------------
#  用法:  ./2_mavros.sh
#
#  端口配对 (与 PX4 SITL instance 0 严格对应):
#     PX4 监听 14580  ->  mavros 监听 14540 并发送到 14580
#     fcu_url = udp://:14540@127.0.0.1:14580
#        ↑ 冒号前的 14540 = mavros 本地绑定端口
#                        ↑ @ 后的 14580 = 发往 PX4 的端口
#
#  命名空间: 不加 ns => 话题为 /mavros/* , 服务为 /mavros/set_mode 等。
#  这与 4_bridge.sh (vins_to_px4.py 默认 /mavros/vision_pose/pose)
#  与 move_drone.py (默认 /mavros/setpoint_position/local) 一致。
#
#  ⚠️ 若 1_sim.sh 用的是 --with-mavros (即 indoor1.launch), 那它已自带
#     一个 ns=iris_0 的 mavros 且已占用 14540 —— 本脚本会检测到并拒绝启动。
# =============================================================================
set +u

source /opt/ros/noetic/setup.bash 2>/dev/null
source "$HOME/catkin_ws/devel/setup.bash" 2>/dev/null
export PX4="$HOME/PX4-Autopilot"
export ROS_PACKAGE_PATH="$PX4:$PX4/Tools/sitl_gazebo:$ROS_PACKAGE_PATH"
export ROS_HOSTNAME=localhost
export ROS_MASTER_URI=http://localhost:11311
source "$HOME/.bashrc" 2>/dev/null   # 非交互下为 no-op

NS="${MAVROS_NS:-}"        # 空 = root ns; 想用 /iris_0 就 MAVROS_NS=/iris_0 ./2_mavros.sh
FCU_URL="${FCU_URL:-udp://:14540@127.0.0.1:14580}"

echo "================================================================"
echo " 2_mavros.sh —— MAVROS"
echo "================================================================"
echo "  fcu_url = $FCU_URL"
echo "  ns      = ${NS:-<root>}  (话题: ${NS}/mavros/*)"
echo "----------------------------------------------------------------"

# ----------------------- 抢端口预检 -----------------------
if ! pgrep -f 'rosmaster --core' >/dev/null; then
    echo "  ❌ roscore 未运行。请先执行:  roscore   (或已跑 1_sim.sh)"
    exit 1
fi

# 已存在的 mavros?
EXISTING="$(pgrep -a -f 'mavros_node' | head -3)"
if [ -n "$EXISTING" ]; then
    echo "  ⚠️ 检测到已有 mavros 在运行:"
    echo "$EXISTING" | sed 's/^/      /'
    echo
    echo "  这通常是 1_sim.sh 用了 indoor1.launch (它内嵌了一个 ns=iris_0 的 mavros)。"
    echo "  两个 mavros 会抢同一个 UDP 14540, 第二个拿不到链路。"
    echo
    echo "  解决办法(二选一):"
    echo "    A) 推荐: 重启仿真用   ./1_sim.sh --no-mavros   然后再跑本脚本"
    echo "    B) 就用已存在的那个:  话题在 /iris_0/mavros/* ,"
    echo "       则 4_bridge.sh 要加 ns, move_drone.py 也要 --ns /iris_0"
    exit 1
fi

# 端口占用?
if ss -uln 2>/dev/null | grep -q ':14540 '; then
    echo "  ⚠️ UDP 14540 已被占用 (可能是上一次异常退出的 mavros 残留):"
    ss -ulnp 2>/dev/null | grep ':14540 ' | sed 's/^/      /'
    echo "  请先清理:  pkill -9 -f mavros_node   然后重跑本脚本"
    exit 1
fi

echo "  预检 OK ✅  正在启动 mavros..."
echo "  自检(另开终端):  rostopic echo -n1 ${NS}/mavros/state   # 期望 connected: True"
echo "  退出: Ctrl+C"
echo "================================================================"
echo

if [ -n "$NS" ]; then
    exec roslaunch mavros px4.launch fcu_url:="$FCU_URL" gcs_url:="" __ns:="$NS"
else
    exec roslaunch mavros px4.launch fcu_url:="$FCU_URL" gcs_url:=""
fi
