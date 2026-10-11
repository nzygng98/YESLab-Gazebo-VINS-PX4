#!/bin/bash
# =============================================================================
#  5_rviz.sh —— 终端 5 (可选) : 启动 RViz 可视化 VINS 输出
# -----------------------------------------------------------------------------
#  用法:  ./5_rviz.sh
#
#  显示内容 (由 vins_rviz.launch 提供):
#     /vins_estimator/odometry       VIO 轨迹
#     /vins_estimator/path           轨迹线
#     /vins_estimator/point_cloud    稀疏点云
#     /vins_estimator/image_track    特征跟踪图 (需 VINS 配置 show_track: 1)
#
#  注意: RViz 很吃资源, 内存紧张时可不开。
# =============================================================================
set +u

source /opt/ros/noetic/setup.bash 2>/dev/null
source "$HOME/catkin_ws/devel/setup.bash" 2>/dev/null
export ROS_HOSTNAME=localhost
export ROS_MASTER_URI=http://localhost:11311
source "$HOME/.bashrc" 2>/dev/null   # 非交互下为 no-op

echo "================================================================"
echo " 5_rviz.sh —— RViz 可视化"
echo "================================================================"

if [ -z "$DISPLAY" ]; then
    echo "  ❌ DISPLAY 未设置 —— 必须在 VMware 桌面终端里运行!"
    exit 1
fi
if ! pgrep -f 'rosmaster --core' >/dev/null; then
    echo "  ❌ roscore 未运行 (请先跑 1_sim.sh)"
    exit 1
fi

echo "  提示: 先在 RViz 里 Add -> By topic -> 选 /vins_estimator/path 等"
echo "  退出: Ctrl+C"
echo "================================================================"
echo

exec roslaunch vins vins_rviz.launch
