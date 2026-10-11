#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# =============================================================================
#  vins_to_px4.py  —  VINS-Fusion 位姿  ->  PX4 (mavros) 视觉位姿桥接
# -----------------------------------------------------------------------------
#  作用:
#    把 VINS-Fusion 的位姿从
#        世界系 ENU (East-North-Up)  +  机体系 FLU (Forward-Left-Up)
#    转换到 PX4 所需的
#        世界系 NED (North-East-Down) +  机体系 FRD (Forward-Right-Down)
#    再发布到 /mavros/vision_pose/pose
#
#  坐标转换数学:
#    位置:  p_ned = M * p_enu                     M = [[0,1,0],[1,0,0],[0,0,-1]]
#    姿态:  R_ned_frd = M * R_enu_flu * A         A = diag(1,-1,-1)  (绕 x 转180°)
#
#  参数 (rosparam):
#    ~odom_topic    (默认 /vins_estimator/odometry)
#    ~pose_topic    (默认 /mavros/vision_pose/pose)
#    ~zero_on_start (默认 true)  把第一帧位置平移到原点, 避免 PX4 EKF 原点跳变
#
#  PX4 侧需要配合的参数(在 QGC 里设):
#    EKF2_EV_CTRL = 15        (EnableEvPos + EnableEvVel + EnableEvYaw + EnableEvHgt)
#    EKF2_HGT_REF = 3         (Vision)
#    EKF2_EV_DELAY ≈ 0~20ms   (VINS 延迟)
# =============================================================================

import math
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

# ENU(世界) -> NED(世界)
M_ENU2NED = np.array([[0.0, 1.0, 0.0],
                      [1.0, 0.0, 0.0],
                      [0.0, 0.0, -1.0]])
# FLU(机体) -> FRD(机体)
A_FLU2FRD = np.diag([1.0, -1.0, -1.0])


def quat_to_R(q):
    """geometry_msgs/Quaternion -> 3x3 旋转矩阵"""
    x, y, z, w = q.x, q.y, q.z, q.w
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    xx, yy, zz = x * x * s, y * y * s, z * z * s
    xy, xz, yz = x * y * s, x * z * s, y * z * s
    wx, wy, wz = w * x * s, w * y * s, w * z * s
    return np.array([[1.0 - (yy + zz), xy - wz,          xz + wy],
                     [xy + wz,         1.0 - (xx + zz),  yz - wx],
                     [xz - wy,         yz + wx,          1.0 - (xx + yy)]])


def R_to_quat(R):
    """3x3 旋转矩阵 -> (x, y, z, w)"""
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0.0:
        S = math.sqrt(tr + 1.0) * 2.0
        w = 0.25 * S
        x = (R[2, 1] - R[1, 2]) / S
        y = (R[0, 2] - R[2, 0]) / S
        z = (R[1, 0] - R[0, 1]) / S
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        S = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / S
        x = 0.25 * S
        y = (R[0, 1] + R[1, 0]) / S
        z = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / S
        x = (R[0, 1] + R[1, 0]) / S
        y = 0.25 * S
        z = (R[1, 2] + R[2, 1]) / S
    else:
        S = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / S
        x = (R[0, 2] + R[2, 0]) / S
        y = (R[1, 2] + R[2, 1]) / S
        z = 0.25 * S
    return x, y, z, w


class VinsToPx4(object):
    def __init__(self):
        self.odom_topic = rospy.get_param('~odom_topic', '/vins_estimator/odometry')
        self.pose_topic = rospy.get_param('~pose_topic', '/mavros/vision_pose/pose')
        self.zero_on_start = rospy.get_param('~zero_on_start', True)

        self.origin = None          # 初始位置(NED), 用于归零
        self.pub = rospy.Publisher(self.pose_topic, PoseStamped, queue_size=10)
        rospy.Subscriber(self.odom_topic, Odometry, self.cb, queue_size=10)

        rospy.loginfo("[vins_to_px4] %s  ->  %s   (zero_on_start=%s)",
                      self.odom_topic, self.pose_topic, self.zero_on_start)

    def cb(self, msg):
        # ---------------- 1. 位置: ENU -> NED ----------------
        p_enu = np.array([msg.pose.pose.position.x,
                          msg.pose.pose.position.y,
                          msg.pose.pose.position.z])
        p_ned = M_ENU2NED.dot(p_enu)

        # 可选: 以第一帧为原点, 防止 PX4 原点跳变
        if self.origin is None:
            self.origin = p_ned.copy() if self.zero_on_start else np.zeros(3)
        p_ned = p_ned - self.origin

        # ---------------- 2. 姿态: (ENU,FLU) -> (NED,FRD) ----------------
        R_enu_flu = quat_to_R(msg.pose.pose.orientation)
        R_ned_frd = M_ENU2NED.dot(R_enu_flu).dot(A_FLU2FRD)
        qx, qy, qz, qw = R_to_quat(R_ned_frd)

        # ---------------- 3. 发布 ----------------
        out = PoseStamped()
        # ★ 时间戳: 优先用 VINS 消息自带的 stamp。
        #   原实现用 rospy.Time.now(): 一旦 /clock 还没起来(use_sim_time=true 但
        #   时钟发布滞后), 会发出 stamp=0 的消息, 而 PX4 EKF2 会【直接丢弃】时间戳
        #   为 0 的 vision_pose -> 表现为 vision 完全没被融合。
        st = msg.header.stamp
        if st.secs == 0 and st.nsecs == 0:
            st = rospy.Time.now()          # 兜底
        out.header.stamp = st
        out.header.frame_id = "world"          # mavros 本地世界系
        out.pose.position.x = float(p_ned[0])
        out.pose.position.y = float(p_ned[1])
        out.pose.position.z = float(p_ned[2])
        out.pose.orientation.x = qx
        out.pose.orientation.y = qy
        out.pose.orientation.z = qz
        out.pose.orientation.w = qw
        self.pub.publish(out)


if __name__ == '__main__':
    rospy.init_node('vins_to_px4', anonymous=False)
    VinsToPx4()
    rospy.spin()
