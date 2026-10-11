#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# =============================================================================
#  move_drone.py  ——  PX4 + MAVROS 位置控制 (自动 OFFBOARD -> arm -> 起飞 -> 平移 -> 保持)
# -----------------------------------------------------------------------------
#  用法:
#     python3 ~/scripts/move_drone.py 1.0 0.0 1.5
#         -> 起飞到 1.5 m, 再沿 x(东) 正方向平移 1 m, 然后保持悬停
#
#     python3 ~/scripts/move_drone.py 0 -2 1.5      # y(北) 负方向 2 m
#     python3 ~/scripts/move_drone.py 0 0 2.0       # 只升到 2 m
#     python3 ~/scripts/move_drone.py 1 1 1.5 --ns /iris_0
#     python3 ~/scripts/move_drone.py 1 0 1.5 --takeoff-alt 1.0 --land
#
#  坐标系: mavros 的 local_position / setpoint_position 都是 【ENU】
#          x = East(东), y = North(北), z = Up(上)   —— 原点 = EKF 原点(解锁点附近)
#
#  动作顺序 (PX4 的硬性要求):
#     1) 先以 20 Hz 持续发 setpoint (OFFBOARD 必须先有 setpoint 流, 否则会被拒)
#     2) 切 OFFBOARD (调 /mavros/set_mode)
#     3) 解锁        (调 /mavros/cmd/arming)
#     4) 目标 (cur_x, cur_y, takeoff_alt)  -> 爬到 1.5 m
#     5) 目标 (x, y, z)                    -> 平移过去
#     6) 原地保持 (继续 20 Hz 发流, 否则 PX4 会触发 OFFBOARD 失控保护)
#
#  ⚠️ 前置条件:
#     - 2_mavros.sh 已跑, /mavros/state 的 connected=True
#     - PX4 EKF 有有效本地位置 (可用  rostopic echo /mavros/local_position/pose 检查)
#     - SITL 下若解锁被拒, 设 PX4 参数 COM_RC_IN_MODE=2 (UDP Joystick)
# =============================================================================

import argparse
import math
import sys
import threading
import time

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest


# ============================== 参数解析 =====================================
def parse_args():
    ap = argparse.ArgumentParser(
        description="PX4+MAVROS 位置控制: 自动 OFFBOARD/arm/起飞/平移/保持",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="示例: python3 move_drone.py 1.0 0.0 1.5\n")
    ap.add_argument("x", type=float, help="目标 x (East, 东 为正) [m]")
    ap.add_argument("y", type=float, help="目标 y (North, 北 为正) [m]")
    ap.add_argument("z", type=float, help="目标 z (Up, 上 为正) [m]")
    ap.add_argument("--ns", default="", help="mavros 命名空间, 如 /iris_0 (默认 空=root)")
    ap.add_argument("--takeoff-alt", type=float, default=1.5,
                    help="第 4 步的起飞高度 [m], 默认 1.5")
    ap.add_argument("--rate", type=float, default=20.0, help="setpoint 发布频率 [Hz], 默认 20")
    ap.add_argument("--tol", type=float, default=0.25, help="到位判定容差 [m], 默认 0.25")
    ap.add_argument("--timeout", type=float, default=90.0,
                    help="每个阶段等待超时 [s], 默认 90")
    ap.add_argument("--land", action="store_true",
                    help="保持 10s 后切 AUTO.LAND 降落 (默认不降落, 一直保持)")
    ap.add_argument("--arm-retry", type=int, default=10, help="解锁重试次数, 默认 10")
    return ap.parse_args()


# ============================== 主体 =========================================
class DroneMover(object):
    def __init__(self, args):
        self.a = args
        self.ns = (args.ns or "").rstrip("/")     # "" 或 "/iris_0"

        self.state = None
        self.local = None                          # 当前本地位置 (ENU)
        self._pos_count = 0                        # local_position 收到的消息计数
        self._lk = threading.Lock()
        self._target = [0.0, 0.0, 0.0]             # 当前目标 (ENU)
        self._stop = threading.Event()
        self._sent = 0                             # 已发出的 setpoint 数

        base = self.ns + "/mavros"

        rospy.Subscriber(base + "/state", State, self._cb_state, queue_size=10)
        rospy.Subscriber(base + "/local_position/pose", PoseStamped,
                         self._cb_local, queue_size=10)

        self.pub = rospy.Publisher(base + "/setpoint_position/local",
                                   PoseStamped, queue_size=10)

        self.srv_mode = base + "/set_mode"
        self.srv_arm = base + "/cmd/arming"
        rospy.loginfo("等待服务 %s / %s ...", self.srv_mode, self.srv_arm)
        rospy.wait_for_service(self.srv_mode, timeout=30.0)
        rospy.wait_for_service(self.srv_arm, timeout=30.0)
        self.cli_mode = rospy.ServiceProxy(self.srv_mode, SetMode)
        self.cli_arm = rospy.ServiceProxy(self.srv_arm, CommandBool)

        # 先把目标设成【起点附近】再开始发流, 避免解锁瞬间跳变
        self._target = [0.0, 0.0, max(0.0, self.a.takeoff_alt)]

        self._th = threading.Thread(target=self._stream_loop)
        self._th.daemon = True
        self._th.start()

    # ---------------------------- 回调 ----------------------------
    def _cb_state(self, msg):
        self.state = msg

    def _cb_local(self, msg):
        self.local = msg
        self._pos_count += 1

    # ------------------------ 20Hz setpoint 线程 ------------------------
    def _stream_loop(self):
        """独立线程以固定频率发布 setpoint —— 必须在切换 OFFBOARD 之前就已在跑"""
        rate = rospy.Rate(self.a.rate)
        while not rospy.is_shutdown() and not self._stop.is_set():
            with self._lk:
                tgt = list(self._target)
            sp = PoseStamped()
            sp.header.stamp = rospy.Time.now()
            sp.header.frame_id = "map"          # setpoint_position/local 的 frame 被忽略
            sp.pose.position.x = tgt[0]
            sp.pose.position.y = tgt[1]
            sp.pose.position.z = tgt[2]
            sp.pose.orientation.w = 1.0         # yaw = 0
            self.pub.publish(sp)
            self._sent += 1
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                break            # 关闭时不要在后台线程抛异常刷屏

    def set_target(self, x, y, z):
        with self._lk:
            self._target = [float(x), float(y), float(z)]
        rospy.loginfo(">>> 新目标 (ENU): x=%.2f  y=%.2f  z=%.2f", x, y, z)

    def cur_target(self):
        with self._lk:
            return list(self._target)

    # ---------------------------- 等待/辅助 ----------------------------
    def wait_position_ready(self, need_ok=3, timeout=30.0,
                            min_hz=5.0, max_down_drift=0.05):
        """
        在切换 OFFBOARD / arm 之前, 必须先确认位置数据真的可用:
          - 每 1 秒检查一次 /mavros/local_position/pose 的消息计数
          - 必须【连续 need_ok 秒】都有有效数据 (速率 >= min_hz)
          - 且 z 轴【不向下剧烈漂移】: 1 秒内下降不得超过 max_down_drift 米
          - 总等待超过 timeout 秒 -> 打印 "等待位置超时，退出" 并返回 False

        背景: PX4 若处于保护模式(如 AUTO.RTL), set_mode OFFBOARD 会被反复拒绝;
              盲目重试只会刷屏。先确认位置健康再动手。
        """
        rospy.loginfo("=== 等待位置数据就绪 ===")
        rospy.loginfo("  要求: 连续 %d 秒有数据 (>=%.0f Hz) 且 z 不向下漂 >%.2f m/s; 超时 %.0fs",
                      need_ok, min_hz, max_down_drift, timeout)
        t0 = time.time()
        ok_sec = 0
        prev_cnt = self._pos_count
        prev_z = None
        while not rospy.is_shutdown():
            el = time.time() - t0
            if el > timeout:
                rospy.logerr("等待位置超时，退出")
                return False
            time.sleep(1.0)
            el = time.time() - t0
            cnt = self._pos_count
            hz = cnt - prev_cnt
            prev_cnt = cnt

            if self.local is None or hz < min_hz:
                ok_sec = 0
                prev_z = None
                rospy.logwarn("  [%4.1fs] 位置数据不可用 (1s 内 %d 条, 需 >=%.0f) -> 重新计时",
                              el, hz, min_hz)
                continue

            z = self.local.pose.position.z
            dz = 0.0 if prev_z is None else (z - prev_z)
            prev_z = z

            if dz < -max_down_drift:
                ok_sec = 0
                rospy.logwarn("  [%4.1fs] z 在向下漂 (%.3f m, z=%.3f) -> 重新计时",
                              el, dz, z)
                continue

            ok_sec += 1
            p = self.local.pose.position
            rospy.loginfo("  [%4.1fs] pos=(%.2f, %.2f, %.2f)  %d Hz  dz=%+.3f  "
                          "连续正常 %d/%d s", el, p.x, p.y, z, hz, dz, ok_sec, need_ok)
            if ok_sec >= need_ok:
                rospy.loginfo("✅ 位置数据就绪 (连续 %d 秒稳定)", ok_sec)
                return True
        return False

    def wait_connected(self, timeout=30.0):
        rospy.loginfo("等待 MAVROS 连接上飞控 ...")
        t0 = time.time()
        while not rospy.is_shutdown() and time.time() - t0 < timeout:
            if self.state is not None and self.state.connected:
                rospy.loginfo("✅ MAVROS connected  (mode=%s armed=%s)",
                              self.state.mode, self.state.armed)
                return True
            time.sleep(0.2)
        rospy.logerr("❌ %.0fs 内 MAVROS 仍未 connected —— 检查 2_mavros.sh 的 fcu_url",
                     timeout)
        return False

    def wait_stream(self, min_sent=40):
        """等 setpoint 流真正被 mavros 订阅, 并发出足够多帧"""
        rospy.loginfo("等待 setpoint 订阅者 ...")
        t0 = time.time()
        while not rospy.is_shutdown() and time.time() - t0 < 20.0:
            if self.pub.get_num_connections() > 0:
                break
            time.sleep(0.2)
        rospy.loginfo("等待发出 >= %d 帧 setpoint ...", min_sent)
        t0 = time.time()
        while not rospy.is_shutdown() and time.time() - t0 < 20.0:
            if self._sent >= min_sent:
                rospy.loginfo("✅ 已发出 %d 帧", self._sent)
                return True
            time.sleep(0.1)
        rospy.logwarn("⚠️ setpoint 帧数不足 (%d), 继续尝试", self._sent)
        return True

    def set_mode_offboard(self, retries=15):
        rospy.loginfo("请求切换 OFFBOARD ...")
        for i in range(retries):
            if rospy.is_shutdown():
                return False
            req = SetModeRequest()
            req.custom_mode = "OFFBOARD"
            try:
                r = self.cli_mode(req)
            except rospy.ServiceException as e:
                rospy.logwarn("set_mode 调用异常: %s", e)
                r = None
            if r is not None and r.mode_sent:
                rospy.loginfo("✅ OFFBOARD 请求已发送 (第 %d 次)", i + 1)
                for _ in range(10):          # 确认真的切过去了
                    time.sleep(0.2)
                    if self.state and self.state.mode == "OFFBOARD":
                        rospy.loginfo("✅ 当前 mode = OFFBOARD")
                        return True
                rospy.logwarn("⚠️ 已发送但 mode 仍为 %s, 重试...",
                              self.state.mode if self.state else "?")
            else:
                rospy.logwarn("OFFBOARD 被拒 (第 %d 次): mode_sent=%s", i + 1,
                              getattr(r, "mode_sent", None))
            time.sleep(0.5)
        rospy.logerr("❌ 无法切到 OFFBOARD。常见原因: setpoint 流未建立 / EKF 无本地位置")
        return False

    def arm(self, retries=None):
        retries = retries or self.a.arm_retry
        if self.state and self.state.armed:
            rospy.loginfo("✅ 已处于解锁状态")
            return True
        rospy.loginfo("请求解锁 (arm) ...")
        for i in range(retries):
            if rospy.is_shutdown():
                return False
            req = CommandBoolRequest()
            req.value = True
            try:
                r = self.cli_arm(req)
            except rospy.ServiceException as e:
                rospy.logwarn("arming 调用异常: %s", e)
                r = None
            if r is not None and r.success:
                rospy.loginfo("✅ arm success (第 %d 次)", i + 1)
                return True
            rospy.logwarn("解锁被拒 (第 %d 次): success=%s result=%s", i + 1,
                          getattr(r, "success", None), getattr(r, "result", None))
            time.sleep(1.0)
        rospy.logerr("❌ 解锁失败。SITL 常见原因: 需要 COM_RC_IN_MODE=2 (UDP Joystick);"
                     " 或 EKF/预检未通过")
        return False

    def dist_to(self, tgt):
        if self.local is None:
            return float("inf")
        p = self.local.pose.position
        return math.sqrt((p.x - tgt[0]) ** 2 + (p.y - tgt[1]) ** 2 + (p.z - tgt[2]) ** 2)

    def goto_and_wait(self, x, y, z, phase="", timeout=None, print_hz=1.0):
        """设定目标并等待到位 (超时则告警但继续保持)"""
        timeout = timeout or self.a.timeout
        self.set_target(x, y, z)
        t0 = time.time()
        t_print = 0.0
        while not rospy.is_shutdown() and time.time() - t0 < timeout:
            if self.state and self.state.mode != "OFFBOARD":
                rospy.logwarn("⚠️ mode 掉出 OFFBOARD -> %s", self.state.mode)
            d = self.dist_to([x, y, z])
            if self.local is not None and time.time() - t_print > 1.0 / print_hz:
                t_print = time.time()
                p = self.local.pose.position
                rospy.loginfo("[%s] 当前(%.2f,%.2f,%.2f) 目标(%.2f,%.2f,%.2f) 偏差 %.3f m",
                              phase, p.x, p.y, p.z, x, y, z, d)
            if d < self.a.tol:
                rospy.loginfo("✅ [%s] 到位 (偏差 %.3f m < %.2f), 用时 %.1fs",
                              phase, d, self.a.tol, time.time() - t0)
                return True
            time.sleep(0.1)
        rospy.logwarn("⚠️ [%s] %.0fs 未完全到位 (偏差 %.3f m), 继续保持当前目标",
                      phase, timeout, self.dist_to([x, y, z]))
        return False

    def escape_failsafe_mode(self):
        """PX4 处于 AUTO.RTL / AUTO.LAND 等保护模式时, set_mode OFFBOARD 会被反复拒绝。
        先显式切到 POSCTL 退出保护模式, 并打印根因提示。"""
        protected = ("RTL", "LAND", "RETURN", "TERMINATION")
        m = (self.state.mode if self.state else "") or ""
        if not any(k in m.upper() for k in protected):
            return True
        rospy.logwarn("⚠️ PX4 当前处于保护模式: %s", m)
        rospy.logwarn("   这通常由【RC-loss / 数据链失控保护】触发, 而不是位置无效:")
        rospy.logwarn("     本机实测 COM_RC_IN_MODE=1 且无摇杆源, NAV_RCL_ACT=2 -> RTL")
        rospy.logwarn("   先尝试切到 POSCTL 退出保护模式 ...")
        for i in range(10):
            if rospy.is_shutdown():
                return False
            req = SetModeRequest()
            req.custom_mode = "POSCTL"
            try:
                r = self.cli_mode(req)
            except rospy.ServiceException as e:
                rospy.logwarn("   set_mode 异常: %s", e)
                r = None
            time.sleep(0.4)
            if self.state and self.state.mode == "POSCTL":
                rospy.loginfo("✅ 已切到 POSCTL, 保护模式解除")
                return True
            rospy.loginfo("   第 %d 次 POSCTL 请求: mode_sent=%s  当前 mode=%s",
                          i + 1, getattr(r, "mode_sent", None),
                          self.state.mode if self.state else "?")
        rospy.logerr("❌ 无法退出保护模式 %s。请在 PX4 侧根治 (mavros 或 PX4 终端):", m)
        rospy.logerr("     param set NAV_RCL_ACT 0      # RC loss -> 不做任何动作 (根治)")
        rospy.logerr("     param set COM_RCL_EXCEPT 5   # GCS(1) + OFFBOARD(4) 例外")
        rospy.logerr("   然后:  rosservice call /mavros/set_mode \"{custom_mode: 'POSCTL'}\"")
        return False

    # ------------------------------ 主流程 ------------------------------
    def run(self):
        a = self.a
        if abs(a.x) > 20 or abs(a.y) > 20:
            rospy.logwarn("⚠️ 目标 |x| 或 |y| > 20 m, 请确认场地空旷")

        # 1) 等连接
        if not self.wait_connected():
            return 2
        # 2) 【新增】等位置数据真的就绪 (连续 3s 有效 + z 不向下剧烈漂移), 超时 30s
        if not self.wait_position_ready(need_ok=3, timeout=30.0):
            return 2
        # 3) 【新增】若处于 AUTO.RTL 等保护模式, 先切 POSCTL 退出 (否则 OFFBOARD 必被拒)
        if not self.escape_failsafe_mode():
            return 3
        # 4) 先让 setpoint 流跑够 2 秒 (OFFBOARD 的硬性前提)
        self.wait_stream(min_sent=int(a.rate * 2))

        # 5) OFFBOARD
        if not self.set_mode_offboard():
            return 3
        # 6) 解锁
        if not self.arm():
            return 4

        # 5) 起飞到 takeoff_alt (x,y 保持当前位置, 避免解锁瞬间水平跳变)
        cx = self.local.pose.position.x if self.local else 0.0
        cy = self.local.pose.position.y if self.local else 0.0
        rospy.loginfo("=== 阶段: 起飞到 %.2f m (保持 x=%.2f y=%.2f) ===",
                      a.takeoff_alt, cx, cy)
        self.goto_and_wait(cx, cy, a.takeoff_alt, phase="起飞")

        # 6) 平移到目标 (x, y, z)
        rospy.loginfo("=== 阶段: 移动到 (%.2f, %.2f, %.2f) ===", a.x, a.y, a.z)
        self.goto_and_wait(a.x, a.y, a.z, phase="平移")

        # 7) 保持
        rospy.loginfo("=== 已到位, 开始保持当前位置 (Ctrl+C 结束) ===")
        if a.land:
            rospy.loginfo("保持 10 s 后切 AUTO.LAND 降落 ...")
            for _ in range(100):
                if rospy.is_shutdown():
                    return 0
                time.sleep(0.1)
            req = SetModeRequest()
            req.custom_mode = "AUTO.LAND"
            try:
                r = self.cli_mode(req)
                rospy.loginfo("AUTO.LAND 请求: mode_sent=%s", r.mode_sent)
            except rospy.ServiceException as e:
                rospy.logerr("AUTO.LAND 失败: %s", e)
        while not rospy.is_shutdown():
            time.sleep(0.2)
        return 0


def make_signal_handler(mover):
    """
    自接管 SIGINT/SIGTERM (覆盖 rospy 默认 handler)。

    为什么必须自接管: 如果等 rospy 自己 shutdown 再在 finally 里调 set_mode,
    那时服务连接已经断了 -> 报 "service [/mavros/set_mode] returned no response"。
    必须在 rospy 还活着的时候先把 AUTO.LAND 发出去。
    """
    def handler(signum, frame):
        rospy.loginfo("收到信号 %d, 安全收尾: 尝试切 AUTO.LAND ...", signum)
        try:
            if mover.state and mover.state.armed:
                req = SetModeRequest()
                req.custom_mode = "AUTO.LAND"
                r = mover.cli_mode(req)          # 此刻 rospy 仍可用
                rospy.loginfo("  ✅ 已请求 AUTO.LAND (mode_sent=%s)", r.mode_sent)
            else:
                rospy.loginfo("  当前 armed=%s, 无需降落",
                              mover.state.armed if mover.state else "?")
        except Exception as e:
            rospy.logwarn("  AUTO.LAND 失败: %s", e)
        mover._stop.set()
        rospy.signal_shutdown("user interrupt")
    return handler


def main():
    args = parse_args()
    rospy.init_node("move_drone", anonymous=False, disable_signals=False)
    if args.z <= 0.0:
        rospy.logwarn("⚠️ 目标 z=%.2f 非正数, 会贴地/撞地", args.z)
    mover = DroneMover(args)

    # 自接管信号 (必须在 init_node 之后, 才能覆盖 rospy 的 handler)
    import signal
    sig = make_signal_handler(mover)
    try:
        signal.signal(signal.SIGINT, sig)
        signal.signal(signal.SIGTERM, sig)
    except ValueError:
        rospy.logwarn("非主线程, 无法接管信号")

    try:
        code = mover.run()
    except KeyboardInterrupt:
        code = 0
    finally:
        mover._stop.set()     # 停掉 20Hz 线程
        # 注意: 此处 rospy 可能已 shutdown, 再调服务会失败;
        #       真正的安全降落已在上面的信号 handler 里完成。
    rospy.loginfo("退出。")
    sys.exit(code)


if __name__ == "__main__":
    main()
