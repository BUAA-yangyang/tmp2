#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
cmd_vel_guard —— /cmd_vel 的唯一出口。

上游是 twist_mux（做多源固定优先级仲裁），本节点做剩下四件事：
  1. 限速     —— 不超过 A1 的 RL 策略训练范围，超了步态会发散
  2. 限加速度 —— 平滑指令跳变
  3. 超时归零 —— 上游断流后主动持续发零。这是最关键的一条：
                 A1 的 State_RL 对 /cmd_vel 既不限幅也不超时，
                 停止发布后它会保持最后一个速度一直走下去。
  4. 就绪门控 —— 控制器没进 RL /cmd_vel 模式时不放行

本节点以固定频率发布，不依赖输入频率，这样"没有输入"和"输入为零"都能稳定输出零。
"""

import math
import threading
import time

import rospy
from a1_navigation_interfaces.msg import CmdMuxStatus
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


def clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


def finite_or_zero(v):
    """拒绝 NaN/Inf，避免无效浮点数进入四足底层控制器。"""
    return v if math.isfinite(v) else 0.0


def rate_limit(target, current, max_delta):
    """把 target 相对 current 的变化限制在 ±max_delta 内。"""
    d = target - current
    if d > max_delta:
        return current + max_delta
    if d < -max_delta:
        return current - max_delta
    return target


class CmdVelGuard(object):
    def __init__(self):
        g = rospy.get_param

        self.max_vx = g("~max_vel_x", 0.50)
        self.max_vy = g("~max_vel_y", 0.30)
        self.max_wz = g("~max_vel_theta", 0.80)

        self.acc_x = g("~max_acc_x", 3.0)
        self.acc_y = g("~max_acc_y", 3.0)
        self.acc_th = g("~max_acc_theta", 4.0)

        self.rate_hz = g("~publish_rate", 50.0)
        self.input_timeout = g("~input_timeout", 0.7)
        self.decel_to_stop = g("~decelerate_to_stop", True)
        self.max_timer_dt = g("~max_timer_dt", 0.1)

        self.require_ready = g("~require_ready", False)
        self.ready_timeout = g("~ready_timeout", 1.0)
        self.emergency_timeout = g("~emergency_timeout", 0.5)
        self.source_timeout = g("~source_timeout", 0.5)

        in_topic = g("~input_topic", "/cmd_vel_muxed")
        out_topic = g("~output_topic", "/cmd_vel")
        navigation_topic = g("~navigation_topic", "/cmd_vel_nav")
        behavior_topic = g("~behavior_topic", "/cmd_vel_behavior")
        teleop_topic = g("~teleop_topic", "/cmd_vel_teleop")
        ready_topic = g("~ready_topic", "/a1/controller_ready")
        emergency_topic = g("~emergency_topic", "/cmd_vel_emergency")
        safety_lock_topic = g("~safety_lock_topic", "/a1_cmd_mux/safety_lock")
        status_topic = g("~status_topic", "/a1/cmd_mux/status")
        self.status_rate = g("~status_rate", 2.0)

        if self.rate_hz <= 0.0:
            raise ValueError("~publish_rate 必须大于 0")
        if min(self.max_vx, self.max_vy, self.max_wz,
               self.acc_x, self.acc_y, self.acc_th,
               self.input_timeout, self.ready_timeout,
               self.emergency_timeout, self.source_timeout,
               self.max_timer_dt) < 0.0:
            raise ValueError("速度、加速度和超时参数不得为负数")

        self.lock = threading.Lock()
        self.target = Twist()
        self.last_in = None            # 上游最后一次消息的时间
        self.out = Twist()             # 当前实际输出（限加速度后的状态）
        self.ready = False
        self.last_ready = None
        self.last_emergency = None
        self.source_stamps = {
            CmdMuxStatus.SOURCE_NAVIGATION: None,
            CmdMuxStatus.SOURCE_BEHAVIOR: None,
            CmdMuxStatus.SOURCE_TELEOP: None,
            CmdMuxStatus.SOURCE_ESTOP: None,
        }
        self.safety_locked = False
        self.last_tick = None
        self.state = "init"

        self.pub = rospy.Publisher(out_topic, Twist, queue_size=1)
        self.status_pub = rospy.Publisher(
            status_topic, CmdMuxStatus, queue_size=1, latch=True)
        rospy.Subscriber(in_topic, Twist, self.on_cmd, queue_size=1)
        # 这些订阅只用于生成团队共享的 CmdMuxStatus；真正仲裁仍只由
        # twist_mux 完成，guard 不会自行选择或转发这些原始速度。
        rospy.Subscriber(
            navigation_topic, Twist, self.on_source,
            callback_args=CmdMuxStatus.SOURCE_NAVIGATION, queue_size=1)
        rospy.Subscriber(
            behavior_topic, Twist, self.on_source,
            callback_args=CmdMuxStatus.SOURCE_BEHAVIOR, queue_size=1)
        rospy.Subscriber(
            teleop_topic, Twist, self.on_source,
            callback_args=CmdMuxStatus.SOURCE_TELEOP, queue_size=1)
        # 急停同时进入 twist_mux 和 guard：mux 保证优先级，guard 保证不受减速度
        # 限制影响，在下一个发布周期立即归零。消息内容会被忽略，任何新消息都表示急停。
        rospy.Subscriber(emergency_topic, Twist, self.on_emergency, queue_size=1)
        # safety_lock 是 std_msgs/Bool。True 后保持锁定，直到明确收到 False；
        # 发布者崩溃不会意外解锁。
        rospy.Subscriber(safety_lock_topic, Bool, self.on_safety_lock, queue_size=1)
        if self.require_ready:
            rospy.Subscriber(ready_topic, Bool, self.on_ready, queue_size=1)

        rospy.loginfo("cmd_vel_guard: %s -> %s @%.0fHz", in_topic, out_topic, self.rate_hz)
        rospy.loginfo("  限速 vx<=%.2f vy<=%.2f wz<=%.2f", self.max_vx, self.max_vy, self.max_wz)
        rospy.loginfo("  限加速 ax<=%.1f ay<=%.1f ath<=%.1f", self.acc_x, self.acc_y, self.acc_th)
        rospy.loginfo("  输入超时 %.2fs -> 归零 (平滑减速=%s)", self.input_timeout, self.decel_to_stop)
        rospy.loginfo("  急停 %s (新鲜窗口 %.2fs，立即归零)", emergency_topic, self.emergency_timeout)
        rospy.loginfo("  安全锁 %s (True 立即归零，需 False 解锁)", safety_lock_topic)
        rospy.loginfo("  就绪门控 %s", "开" if self.require_ready else "关")

        rospy.on_shutdown(self.stop_on_shutdown)
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.rate_hz), self.tick)
        rospy.Timer(rospy.Duration(1.0 / max(self.status_rate, 0.1)), self.publish_status)

    def on_cmd(self, msg):
        clean = Twist()
        clean.linear.x = finite_or_zero(msg.linear.x)
        clean.linear.y = finite_or_zero(msg.linear.y)
        clean.angular.z = finite_or_zero(msg.angular.z)
        if (clean.linear.x != msg.linear.x
                or clean.linear.y != msg.linear.y
                or clean.angular.z != msg.angular.z):
            rospy.logwarn_throttle(1.0, "cmd_vel_guard: 收到 NaN/Inf，已替换为零")
        with self.lock:
            self.target = clean
            self.last_in = rospy.Time.now()

    def on_ready(self, msg):
        with self.lock:
            self.ready = bool(msg.data)
            self.last_ready = rospy.Time.now()

    def on_emergency(self, _msg):
        with self.lock:
            now = rospy.Time.now()
            self.last_emergency = now
            self.source_stamps[CmdMuxStatus.SOURCE_ESTOP] = now

    def on_source(self, _msg, source_id):
        with self.lock:
            self.source_stamps[source_id] = rospy.Time.now()

    def on_safety_lock(self, msg):
        with self.lock:
            self.safety_locked = bool(msg.data)

    @staticmethod
    def _fresh(stamp, now, timeout):
        if stamp is None:
            return False
        age = (now - stamp).to_sec()
        # ROS /clock 回拨时 age 为负。把旧消息判为失效，而不是继续放行。
        return 0.0 <= age <= timeout

    def _input_fresh(self, now):
        return self._fresh(self.last_in, now, self.input_timeout)

    def _ready_ok(self, now):
        if not self.require_ready:
            return True
        if not self._fresh(self.last_ready, now, self.ready_timeout):
            return False
        return self.ready

    def tick(self, _):
        now = rospy.Time.now()

        with self.lock:
            nominal_dt = 1.0 / self.rate_hz
            if self.last_tick is None:
                raw_dt = nominal_dt
            else:
                raw_dt = (now - self.last_tick).to_sec()
            self.last_tick = now

            if raw_dt < 0.0:
                # 仿真 reset 会让 /clock 回拨。旧时间戳此时全部失去意义，
                # 清空状态并立即归零，等待各上游重新发布。
                self.target = Twist()
                self.last_in = None
                self.last_ready = None
                self.last_emergency = None
                for source_id in self.source_stamps:
                    self.source_stamps[source_id] = None
                self.out = Twist()
                self.state = "time_jump_zero"
            else:
                dt = min(raw_dt, max(self.max_timer_dt, nominal_dt))
                fresh = self._input_fresh(now)
                ready = self._ready_ok(now)
                emergency = self._fresh(
                    self.last_emergency, now, self.emergency_timeout)
                tgt = self.target

                if self.safety_locked:
                    # mission_manager 等故障保护：立即零，明确 False 后才解锁。
                    want = Twist()
                    self.state = "safety_locked"
                    immediate = True
                elif emergency:
                    # 急停不走加速度限制。即使发布者错误地发了非零 Twist，
                    # guard 也只把它解释为“急停仍然有效”。
                    want = Twist()
                    self.state = "emergency_stop"
                    immediate = True
                elif not ready:
                    # 控制器没就绪：立即零，不做平滑（不该有任何运动）
                    want = Twist()
                    self.state = "not_ready"
                    immediate = True
                elif not fresh:
                    # 上游断流：归零。平滑与否看配置
                    want = Twist()
                    self.state = "timeout_zero"
                    immediate = not self.decel_to_stop
                else:
                    want = Twist()
                    want.linear.x = clamp(tgt.linear.x, -self.max_vx, self.max_vx)
                    want.linear.y = clamp(tgt.linear.y, -self.max_vy, self.max_vy)
                    want.angular.z = clamp(tgt.angular.z, -self.max_wz, self.max_wz)
                    self.state = "active"
                    immediate = False

                if immediate:
                    self.out = Twist()
                else:
                    o = Twist()
                    o.linear.x = rate_limit(
                        want.linear.x, self.out.linear.x, self.acc_x * dt)
                    o.linear.y = rate_limit(
                        want.linear.y, self.out.linear.y, self.acc_y * dt)
                    o.angular.z = rate_limit(
                        want.angular.z, self.out.angular.z, self.acc_th * dt)
                    self.out = o

            out = self.out

        self.pub.publish(out)

    def _active_source(self, now):
        candidates = (
            (CmdMuxStatus.SOURCE_ESTOP, self.emergency_timeout),
            (CmdMuxStatus.SOURCE_TELEOP, self.source_timeout),
            (CmdMuxStatus.SOURCE_BEHAVIOR, self.source_timeout),
            (CmdMuxStatus.SOURCE_NAVIGATION, self.source_timeout),
        )
        for source_id, timeout in candidates:
            stamp = self.source_stamps[source_id]
            if self._fresh(stamp, now, timeout):
                return source_id, (now - stamp).to_sec()
        return CmdMuxStatus.SOURCE_NONE, -1.0

    def publish_status(self, _):
        now = rospy.Time.now()
        with self.lock:
            tracked_source, source_age = self._active_source(now)
            if self.state == "emergency_stop":
                active_source = CmdMuxStatus.SOURCE_ESTOP
            elif self.state == "active":
                active_source = tracked_source
            else:
                active_source = CmdMuxStatus.SOURCE_NONE
                source_age = -1.0

            msg = CmdMuxStatus()
            msg.header.stamp = now
            msg.active_source = active_source
            msg.emergency_stop = self._fresh(
                self.last_emergency, now, self.emergency_timeout)
            msg.output_enabled = self.state == "active"
            msg.last_cmd = self.out
            msg.active_source_age_s = source_age
            msg.status = self.state
        self.status_pub.publish(msg)

    def stop_on_shutdown(self):
        """正常 Ctrl-C/rosnode kill 时，在连接断开前尽力把最后指令改成零。"""
        zero = Twist()
        with self.lock:
            self.target = zero
            self.out = zero
            self.state = "shutdown_zero"
        # 用墙钟而不是 ROS 时间：仿真可能已经暂停或 /clock 已经停止。
        for _ in range(5):
            self.pub.publish(zero)
            time.sleep(0.02)


if __name__ == "__main__":
    rospy.init_node("cmd_vel_guard")
    CmdVelGuard()
    rospy.spin()
