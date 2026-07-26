#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
a1_cmd_mux 可重复验收。

覆盖 MODULES.md 的五项标准，并补测：
  - ready 心跳门控
  - 急停与 safety lock 的立即归零延迟
  - 高优先级撤走时不因 guard 抢先超时而产生停顿
  - NaN / Inf 不进入底层

本脚本会主动发布速度。应在独立 ROS master 中运行，或确保机器人不在
RL /cmd_vel 模式。推荐直接使用 cmd_mux_acceptance.launch。
"""

import math
import subprocess
import sys
import threading

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


SRC = {
    "nav": "/cmd_vel_nav",
    "beh": "/cmd_vel_behavior",
    "tel": "/cmd_vel_teleop",
    "est": "/cmd_vel_emergency",
}

ZERO_EPS = 1e-3
results = []


def check(name, ok, detail=""):
    results.append((name, ok, detail))
    print("[%s] %s%s" % (
        "通过" if ok else "失败", name, ("  " + detail) if detail else ""))


def to_twist(value):
    if isinstance(value, (tuple, list)):
        vx, vy, wz = value
    else:
        vx, vy, wz = value, 0.0, 0.0
    msg = Twist()
    msg.linear.x = vx
    msg.linear.y = vy
    msg.angular.z = wz
    return msg


class Harness(object):
    def __init__(self):
        self.output_topic = rospy.get_param("~output_topic", "/cmd_vel")
        self.ready_topic = rospy.get_param(
            "~ready_topic", "/a1/controller_ready")
        self.safety_topic = rospy.get_param(
            "~safety_lock_topic", "/a1_cmd_mux/safety_lock")

        self.pubs = {
            key: rospy.Publisher(topic, Twist, queue_size=1)
            for key, topic in SRC.items()
        }
        self.ready_pub = rospy.Publisher(
            self.ready_topic, Bool, queue_size=1)
        self.safety_pub = rospy.Publisher(
            self.safety_topic, Bool, queue_size=1, latch=True)

        self.active = {}
        self.ready_value = None
        self.safety_value = False
        self.data_lock = threading.Lock()
        self.samples = []

        rospy.Subscriber(
            self.output_topic, Twist, self.on_out, queue_size=500)
        rospy.Timer(rospy.Duration(0.05), self.pump)

    def on_out(self, msg):
        with self.data_lock:
            self.samples.append((
                rospy.Time.now().to_sec(),
                msg.linear.x,
                msg.linear.y,
                msg.angular.z,
            ))

    def pump(self, _event):
        with self.data_lock:
            active = dict(self.active)
            ready_value = self.ready_value
            safety_value = self.safety_value
        for key, value in active.items():
            self.pubs[key].publish(to_twist(value))
        if ready_value is not None:
            self.ready_pub.publish(Bool(data=ready_value))
        self.safety_pub.publish(Bool(data=safety_value))

    def set_sources(self, **sources):
        with self.data_lock:
            self.active = dict(sources)

    def set_ready(self, value):
        with self.data_lock:
            self.ready_value = value

    def set_safety(self, value):
        with self.data_lock:
            self.safety_value = bool(value)

    def clear(self):
        with self.data_lock:
            self.samples = []

    def rows(self):
        with self.data_lock:
            return list(self.samples)

    @staticmethod
    def now():
        return rospy.Time.now().to_sec()


def is_zero(row):
    return all(abs(value) < ZERO_EPS for value in row[1:])


def first_zero_delay(rows, start):
    for row in rows:
        if row[0] >= start and is_zero(row):
            return row[0] - start
    return None


def mean_axis(rows, index):
    return sum(row[index] for row in rows) / len(rows) if rows else None


def collect(harness, duration, settle=0.0):
    if settle > 0.0:
        rospy.sleep(settle)
    harness.clear()
    rospy.sleep(duration)
    return harness.rows()


def check_unique_output(output_topic):
    try:
        output = subprocess.check_output(
            ["rostopic", "info", output_topic],
            stderr=subprocess.STDOUT,
        ).decode()
        publishers = []
        in_publishers = False
        for line in output.splitlines():
            if line.startswith("Publishers:"):
                in_publishers = True
                continue
            if line.startswith("Subscribers:"):
                in_publishers = False
            if in_publishers and line.strip().startswith("*"):
                publishers.append(line.strip().lstrip("* ").split(" ")[0])
        ok = len(publishers) == 1 and "cmd_vel_guard" in publishers[0]
        check("5. /cmd_vel 唯一出口", ok, "发布者=%s" % publishers)
    except Exception as exc:
        check("5. /cmd_vel 唯一出口", False, "检查失败: %s" % exc)


def main():
    rospy.init_node(
        "cmd_mux_acceptance", anonymous=True, disable_signals=True)
    harness = Harness()
    rospy.sleep(2.0)

    check_unique_output(harness.output_topic)

    # ---------- ready 门控 ----------
    harness.set_sources(nav=0.30)
    harness.set_ready(None)
    rows = collect(harness, 0.5, settle=1.1)
    check(
        "6a. ready 未建立时锁零",
        bool(rows) and all(is_zero(row) for row in rows),
        "无 ready 心跳时样本数=%d" % len(rows),
    )

    harness.set_ready(True)
    rows = collect(harness, 0.5, settle=0.5)
    ready_vx = mean_axis(rows, 1)
    check(
        "6b. ready=True 放行",
        ready_vx is not None and abs(ready_vx - 0.30) < 0.04,
        "期望 +0.30，实测 %s" % (
            "无输出" if ready_vx is None else "%+.3f" % ready_vx),
    )

    harness.clear()
    ready_false_at = harness.now()
    harness.set_ready(False)
    rospy.sleep(0.35)
    delay = first_zero_delay(harness.rows(), ready_false_at)
    check(
        "6c. ready=False 立即锁零",
        delay is not None and delay <= 0.15,
        "归零延迟=%s" % (
            "未归零" if delay is None else "%.3fs" % delay),
    )
    harness.set_ready(True)
    rospy.sleep(0.4)

    # ---------- 速度限制 ----------
    harness.set_sources(nav=(5.0, -5.0, 5.0))
    rows = collect(harness, 0.8, settle=1.0)
    if rows:
        maxima = tuple(max(abs(row[index]) for row in rows)
                       for index in (1, 2, 3))
        ok = (maxima[0] <= 0.505
              and maxima[1] <= 0.305
              and maxima[2] <= 0.805)
        check(
            "3a. 三轴限速",
            ok,
            "最大 |vx/vy/wz|=%.3f/%.3f/%.3f" % maxima,
        )
    else:
        check("3a. 三轴限速", False, "没有收到输出")

    # ---------- 加速度限制 ----------
    harness.set_sources(nav=(0.5, 0.3, 0.8))
    rospy.sleep(1.0)
    harness.clear()
    harness.set_sources(nav=(-0.5, -0.3, -0.8))
    rospy.sleep(1.2)
    rows = harness.rows()
    worst = [0.0, 0.0, 0.0]
    for before, after in zip(rows, rows[1:]):
        dt = after[0] - before[0]
        if dt < 1e-4:
            continue
        for out_index, row_index in enumerate((1, 2, 3)):
            worst[out_index] = max(
                worst[out_index],
                abs(after[row_index] - before[row_index]) / dt,
            )
    accel_ok = (len(rows) > 10
                and worst[0] <= 3.6
                and worst[1] <= 3.6
                and worst[2] <= 4.8)
    check(
        "3b. 三轴限加速度",
        accel_ok,
        "最大 |ax/ay/ath|=%.2f/%.2f/%.2f" % tuple(worst),
    )

    # ---------- 普通速度源优先级 ----------
    cases = [
        ("behavior 覆盖 navigation",
         dict(nav=0.40, beh=-0.30), -0.30),
        ("teleop 覆盖 behavior",
         dict(nav=0.40, beh=-0.30, tel=0.25), 0.25),
    ]
    for name, sources, expected in cases:
        harness.set_sources(**sources)
        rows = collect(harness, 0.5, settle=1.0)
        actual = mean_axis(rows, 1)
        check(
            "2. " + name,
            actual is not None and abs(actual - expected) < 0.04,
            "期望 %+.2f，实测 %s" % (
                expected,
                "无输出" if actual is None else "%+.3f" % actual),
        )

    # ---------- 急停：消息内容忽略并立即归零 ----------
    harness.set_sources(nav=0.45)
    rospy.sleep(1.0)
    harness.clear()
    emergency_at = harness.now()
    harness.set_sources(nav=0.45, est=1.0)
    rospy.sleep(0.4)
    rows = harness.rows()
    delay = first_zero_delay(rows, emergency_at)
    tail_zero = bool(rows) and all(is_zero(row) for row in rows[-10:])
    check(
        "2c. emergency 覆盖所有源并立即归零",
        delay is not None and delay <= 0.15 and tail_zero,
        "故意输入非零急停，归零延迟=%s，保持零=%s" % (
            "未归零" if delay is None else "%.3fs" % delay, tail_zero),
    )
    harness.set_sources(nav=0.35)
    rows = collect(harness, 0.4, settle=0.9)
    recovered = mean_axis(rows, 1)
    check(
        "2d. emergency 停发后超时释放",
        recovered is not None and abs(recovered - 0.35) < 0.04,
        "期望恢复 nav +0.35，实测 %s" % (
            "无输出" if recovered is None else "%+.3f" % recovered),
    )

    # ---------- safety lock：立即锁零且显式解锁 ----------
    harness.set_sources(nav=0.40)
    rospy.sleep(0.8)
    harness.clear()
    lock_at = harness.now()
    harness.set_safety(True)
    rospy.sleep(0.35)
    delay = first_zero_delay(harness.rows(), lock_at)
    check(
        "7a. safety_lock=True 立即锁零",
        delay is not None and delay <= 0.15,
        "归零延迟=%s" % (
            "未归零" if delay is None else "%.3fs" % delay),
    )
    harness.set_safety(False)
    rows = collect(harness, 0.4, settle=0.5)
    unlocked = mean_axis(rows, 1)
    check(
        "7b. safety_lock=False 显式解锁",
        unlocked is not None and abs(unlocked - 0.40) < 0.04,
        "期望恢复 nav +0.40，实测 %s" % (
            "无输出" if unlocked is None else "%+.3f" % unlocked),
    )

    # ---------- 切换瞬态 ----------
    # 两个源取同方向速度。如果 guard 0.3s 抢先超时，输出会明显跌到接近零；
    # 正确的 0.7s 超时应先等 mux 在 0.5s 后回落到 nav。
    harness.set_sources(nav=0.25, beh=0.45)
    rospy.sleep(1.0)
    harness.clear()
    harness.set_sources(nav=0.25)
    rospy.sleep(0.9)
    rows = harness.rows()
    transient = rows[:max(1, int(len(rows) * 0.85))]
    min_vx = min((row[1] for row in transient), default=-999.0)
    final_vx = mean_axis(rows[-10:], 1) if rows else None
    switch_ok = (bool(rows)
                 and min_vx >= 0.18
                 and final_vx is not None
                 and abs(final_vx - 0.25) < 0.04)
    check(
        "4. 高优先级撤走无停顿、无残留",
        switch_ok,
        "切换过程最小 vx=%.3f，最终=%s" % (
            min_vx,
            "无输出" if final_vx is None else "%+.3f" % final_vx),
    )

    # ---------- 全部输入断开后归零 ----------
    harness.set_sources(nav=0.40)
    rospy.sleep(1.0)
    harness.clear()
    stop_at = harness.now()
    harness.set_sources()
    rospy.sleep(2.0)
    rows = harness.rows()
    delay = first_zero_delay(rows, stop_at)
    tail_zero = bool(rows) and all(is_zero(row) for row in rows[-20:])
    check(
        "1a. 输入超时后归零",
        delay is not None and delay <= 1.10 and tail_zero,
        "归零延迟=%s，之后持续为零=%s" % (
            "未归零" if delay is None else "%.2fs" % delay, tail_zero),
    )
    if len(rows) > 1:
        span = rows[-1][0] - rows[0][0]
        rate = (len(rows) - 1) / span if span > 0.0 else 0.0
    else:
        rate = 0.0
    check(
        "1b. 断流后仍持续发零",
        rate > 30.0,
        "实测发布频率 %.1fHz" % rate,
    )

    # ---------- guard 有序退出与自动重启 ----------
    harness.set_sources(nav=0.40)
    rospy.sleep(0.8)
    harness.clear()
    kill_at = harness.now()
    try:
        subprocess.check_call(
            ["rosnode", "kill", "/cmd_vel_guard"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.STDOUT,
        )
        rospy.sleep(1.2)
        rows = harness.rows()
        delay = first_zero_delay(rows, kill_at)
        restarted_vx = mean_axis(rows[-10:], 1) if rows else None
        check(
            "9a. guard 有序退出先归零",
            delay is not None and delay <= 0.20,
            "归零延迟=%s" % (
                "未观察到零" if delay is None else "%.3fs" % delay),
        )
        check(
            "9b. guard 异常后自动重启",
            restarted_vx is not None and abs(restarted_vx - 0.40) < 0.04,
            "重启后期望恢复 nav +0.40，实测 %s" % (
                "无输出" if restarted_vx is None
                else "%+.3f" % restarted_vx),
        )
    except Exception as exc:
        check("9a. guard 有序退出先归零", False, "rosnode kill 失败: %s" % exc)
        check("9b. guard 异常后自动重启", False, "未能执行重启检查")

    # ---------- 无效浮点保护 ----------
    harness.set_sources(nav=(float("nan"), float("inf"), -float("inf")))
    rows = collect(harness, 0.5, settle=0.8)
    finite_zero = bool(rows) and all(
        all(math.isfinite(value) and abs(value) < ZERO_EPS
            for value in row[1:])
        for row in rows
    )
    check(
        "8. NaN/Inf 拒绝",
        finite_zero,
        "无效输入全部转换为有限零输出=%s" % finite_zero,
    )

    harness.set_sources()
    harness.set_safety(False)
    harness.set_ready(False)
    rospy.sleep(0.2)

    print("\n================ 验收汇总 ================")
    passed = sum(1 for _, ok, _ in results if ok)
    for name, ok, _detail in results:
        print("  %s  %s" % ("✓" if ok else "✗", name))
    print("  %d/%d 通过" % (passed, len(results)))
    return 0 if passed == len(results) else 1


if __name__ == "__main__":
    sys.exit(main())
