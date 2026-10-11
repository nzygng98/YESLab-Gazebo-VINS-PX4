#!/bin/bash
# =============================================================================
#  4_bridge.sh —— 终端 4 : 坐标桥接  VINS -> PX4 (mavros vision_pose)
# -----------------------------------------------------------------------------
#  用法:  ./4_bridge.sh
#
#  它运行 ~/scripts/vins_to_px4.py :
#      /vins_estimator/odometry            (ENU, 机体 FLU)
#        --[ENU->NED, FLU->FRD]-->
#      /mavros/vision_pose/pose            (NED, 机体 FRD)  <- PX4 EKF2 吃这个
#
#  PX4 侧必须同时设置 (本机 PX4 v1.13.3 的参数名, 老版本与新版本不同!):
#     EKF2_AID_MASK   = 24     (bit3 GPS? -> 0 表示不依赖 GPS; 24 = EV_POS|EV_YAW)
#     EKF2_HGT_MODE   = 3      (Vision)      <- v1.13 用这个
#     EKF2_GPS_CTRL   = 0      (关闭 GPS)
#  (若是 PX4 >= 1.14, 名字变成 EKF2_EV_CTRL / EKF2_HGT_REF)
#
#  ⚠️ mavros 若跑在 /iris_0, 要加 ns:  ./4_bridge.sh /iris_0
# =============================================================================
set +u

source /opt/ros/noetic/setup.bash 2>/dev/null
source "$HOME/catkin_ws/devel/setup.bash" 2>/dev/null
export PX4="$HOME/PX4-Autopilot"
export ROS_PACKAGE_PATH="$PX4:$PX4/Tools/sitl_gazebo:$ROS_PACKAGE_PATH"
export ROS_HOSTNAME=localhost
export ROS_MASTER_URI=http://localhost:11311
source "$HOME/.bashrc" 2>/dev/null   # 非交互下为 no-op

NS="${1:-}"                      # 例: ./4_bridge.sh /iris_0
ODOM="${ODOM_TOPIC:-/vins_estimator/odometry}"
POSE="${NS}/mavros/vision_pose/pose"
BRIDGE="$HOME/scripts/vins_to_px4.py"

echo "================================================================"
echo " 4_bridge.sh —— VINS -> PX4 坐标桥接"
echo "================================================================"
echo "  脚本      = $BRIDGE"
echo "  odom_topic= $ODOM"
echo "  pose_topic= $POSE"
echo "----------------------------------------------------------------"

[ -f "$BRIDGE" ] || { echo "  ❌ 找不到 $BRIDGE"; exit 1; }
if ! pgrep -f 'rosmaster --core' >/dev/null; then
    echo "  ❌ roscore 未运行"; exit 1
fi

echo "  等待 VINS 输出 $ODOM ..."
TOPICS="$(timeout -s KILL 10 rostopic list 2>/dev/null)"
if echo "$TOPICS" | grep -q 'vins_estimator'; then
    echo "  ✅ 检测到 /vins_estimator/*"
else
    echo "  ⚠️ 没看到 /vins_estimator/* —— 3_vins.sh 可能还没初始化完成"
fi

echo "  启动桥接 (Ctrl+C 退出)"
echo "  验证:  rostopic echo -n1 $POSE   # header.stamp 必须非 0!"
echo "================================================================"
echo

exec python3 "$BRIDGE" _odom_topic:="$ODOM" _pose_topic:="$POSE"
