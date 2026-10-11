#!/bin/bash
# =============================================================================
#  3_vins.sh —— 终端 3 : 启动 VINS-Fusion (双目 + IMU)
# -----------------------------------------------------------------------------
#  用法:  ./3_vins.sh
#
#  输入话题 (由 Gazebo 的 libgazebo_ros_camera / _imu_sensor 插件提供):
#     /iris_0/stereo_camera/left/image_raw
#     /iris_0/stereo_camera/right/image_raw
#     /iris_0/imu_gazebo
#
#  输出 (节点 /vins_estimator, NodeHandle("~") -> 私有命名空间):
#     /vins_estimator/odometry      ← 4_bridge.sh 消费这个
#     /vins_estimator/camera_pose
#     /vins_estimator/imu_propagate
#     /vins_estimator/path
#
#  ⚠️ 必须先有 roscore (1_sim.sh / 2_mavros.sh 已经起了)。
# =============================================================================
set +u

source /opt/ros/noetic/setup.bash 2>/dev/null
source "$HOME/catkin_ws/devel/setup.bash" 2>/dev/null
export PX4="$HOME/PX4-Autopilot"
export ROS_PACKAGE_PATH="$PX4:$PX4/Tools/sitl_gazebo:$ROS_PACKAGE_PATH"
export ROS_HOSTNAME=localhost
export ROS_MASTER_URI=http://localhost:11311
source "$HOME/.bashrc" 2>/dev/null   # 非交互下为 no-op

CFG="${VINS_CFG:-$HOME/catkin_ws/src/VINS-Fusion/config/xtdrone_sitl/px4_sitl_stereo_imu_config.yaml}"

echo "================================================================"
echo " 3_vins.sh —— VINS-Fusion"
echo "================================================================"
echo "  config = $CFG"
echo "----------------------------------------------------------------"

[ -f "$CFG" ] || { echo "  ❌ 配置文件不存在: $CFG"; exit 1; }
[ -x "$HOME/catkin_ws/devel/lib/vins/vins_node" ] || {
    echo "  ❌ vins_node 未构建。请先:  cd ~/catkin_ws && catkin_make -j2"; exit 1; }
if ! pgrep -f 'rosmaster --core' >/dev/null; then
    echo "  ❌ roscore 未运行。请先跑 1_sim.sh (它含 Gazebo+PX4+roscore)"
    exit 1
fi

echo "  等待相机/IMU 话题就绪 (最多 30s)..."
for i in $(seq 1 30); do
    if timeout -s KILL 5 rostopic list 2>/dev/null | grep -q '/iris_0/imu_gazebo'; then
        echo "  ✅ /iris_0/imu_gazebo 已存在"
        break
    fi
    [ "$i" = "30" ] && echo "  ⚠️ 30s 内没看到 /iris_0/imu_gazebo —— 检查 1_sim.sh 的 Gazebo 是否正常出帧"
    sleep 1
done

echo "  启动 VINS (初始化通常需要 10~20s, 会出现 'Initialization finish!')"
echo "  退出: Ctrl+C"
echo "================================================================"
echo

exec rosrun vins vins_node "$CFG"
