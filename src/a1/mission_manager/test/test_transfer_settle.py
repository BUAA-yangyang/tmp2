#!/usr/bin/env python3
"""换层前「机身真的停了没有」这条判据。

为什么需要它：跨 generation 的 map->world 变换靠「换层前后同一个物理位姿」
把两个 localization generation 联系起来。mf61 的裁判真值显示，转身发完零速
之后机身还会继续回弹 **+13.44 / -3.19 / -3.12 度**（方向恒与转身相反），而
新一代估计器要到 ELEVATOR_CALL 之后 10.9 / 10.7 / 4.1 仿真秒才重锚——那时
回弹早已结束。于是旧代码在 ELEVATOR_CALL 采的那个位姿和新一代重锚的位姿
不是同一个航向，mf61 实测差 13.47 度。

13.47 度会让距机器人 5 m 处的点偏 1.16 m，超过官方 evaluation.md 的 1.0 m
匹配阈值；3.16 度只偏 0.28 m 能过。所以这条判据的精度直接决定上层危险源
的 22 分拿不拿得到，阈值不能随手改。
"""
import math
from pathlib import Path
import sys
import unittest

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from transfer_settle import YawQuiescence

CONFIG = Path(__file__).resolve().parents[1] / "config" / "multifloor.yaml"

# docs/evaluation.md：三维欧氏距离，默认阈值 1.0 m。
MATCH_THRESHOLD_M = 1.0
# 房间尺度下检测点到机器人的典型距离；mf61 的房间 23.5-46.1 m²。
TYPICAL_SOURCE_RANGE_M = 5.0
# mf61 实测：机身确实静止时，估计器在 0.5 s 窗口上的 |dyaw| 最大值
# （三次换层分别 0.1525 / 0.0379 / 0.0161 度）。判据必须在噪声之上。
MEASURED_ESTIMATOR_NOISE_DEG = 0.1525
# mf61 实测回弹总量的最大值。预算必须够等完它。
MEASURED_WORST_REBOUND_DEG = 13.44


def settle_config():
    document = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
    return document["elevator"]["transfer_settle"]


class QuiescenceRuleTest(unittest.TestCase):
    """判据本身：不能提前说停了，也不能永远不说。"""

    def monitor(self):
        return YawQuiescence(reference_max_age_s=0.5,
                             quiescent_yaw=math.radians(0.25),
                             minimum_hold_s=2.0)

    def test_a_still_body_is_declared_settled_after_the_minimum_hold(self):
        monitor = self.monitor()
        settled_at = None
        for step in range(0, 61):
            now = step * 0.1
            if monitor.observe(now, 1.234) and settled_at is None:
                settled_at = now
        self.assertIsNotNone(settled_at, "完全静止的机身必须能被判定为停了")
        # 两个条件并行计时而不是相加：0.5 s 后才有够老的参考帧可比，静止确认
        # 从那时起算，到 t+1.0 已满一个窗口；最小保持 2.0 s 是更晚的那个下界。
        self.assertAlmostEqual(settled_at, 2.0, places=6)

    def test_the_minimum_hold_survives_a_single_lucky_sample(self):
        """mf61 transfer 1->2：t+0.5 的 0.5 s 窗口只有 0.16 度，
        而后面还有 3.5 度没回弹完。只看一帧就会提前放行。"""
        monitor = self.monitor()
        # 0.0-0.6 s 几乎不动（那个假象），之后继续以 2 度/秒回弹 2 秒。
        for step in range(0, 7):
            self.assertFalse(monitor.observe(step * 0.1, 0.0))
        yaw = 0.0
        for step in range(7, 27):
            yaw += math.radians(0.2)
            self.assertFalse(
                monitor.observe(step * 0.1, yaw),
                "回弹还在进行时不得判定为已停止 (t=%.1f)" % (step * 0.1))

    def test_a_replayed_sample_cannot_age_into_a_reference(self):
        """位姿话题停更时，缓存里的旧位姿会被反复读到。
        没有新测量就不是「没有运动」。"""
        monitor = self.monitor()
        monitor.observe(0.0, 0.0)
        for _ in range(100):
            self.assertFalse(monitor.observe(0.0, 0.0))

    def test_a_gap_in_the_pose_stream_does_not_prove_stillness(self):
        monitor = self.monitor()
        for step in range(0, 30):
            monitor.observe(step * 0.1, 0.0)
        self.assertTrue(monitor.settled(2.9))
        # 中断 5 秒后第一帧回来，机身已经转了 30 度：不能还认为是停的。
        self.assertFalse(monitor.observe(8.0, math.radians(30.0)))

    def test_the_measured_mf61_rebound_is_not_called_settled_early(self):
        """用 mf61 transfer 0->1 估计器实测的回弹曲线直接回放。

        (t 相对 ELEVATOR_CALL, 估计器 yaw 相对 CALL 时刻, 单位度)
        """
        curve = [(0.0, 0.000), (0.5, 2.794), (1.0, 5.807), (1.5, 8.702),
                 (2.0, 10.530), (2.5, 12.069), (3.0, 12.616), (3.5, 12.996),
                 (4.0, 13.193), (5.0, 13.379), (6.0, 13.443)]
        monitor = self.monitor()
        settled_at = None
        for now, degrees in curve:
            if monitor.observe(now, math.radians(degrees)) and settled_at is None:
                settled_at = now
        self.assertIsNotNone(settled_at, "回弹结束后必须判停，否则每次都要烧满预算")
        self.assertGreaterEqual(
            settled_at, 3.0,
            "回弹在 t+3 前还有 0.8 度以上没走完，不能在此之前判停")
        remaining = math.radians(13.443) - math.radians(
            dict(curve)[settled_at])
        self.assertLess(
            math.degrees(remaining), 1.0,
            "判停时剩余回弹 %.2f 度过大" % math.degrees(remaining))


class SettleBudgetTest(unittest.TestCase):
    """配置：阈值和预算都必须能被 mf61 的实测数字反推出来。"""

    @classmethod
    def setUpClass(cls):
        cls.settle = settle_config()

    def test_the_threshold_sits_above_the_measured_estimator_noise(self):
        threshold = float(self.settle["quiescent_yaw_deg"])
        self.assertGreater(
            threshold, MEASURED_ESTIMATOR_NOISE_DEG,
            "判据 %.3f 度低于实测估计器噪声 %.4f 度，机身停了也判不出来"
            % (threshold, MEASURED_ESTIMATOR_NOISE_DEG))

    def test_the_threshold_leaves_the_match_threshold_intact(self):
        """剩余航向误差换算成位置误差，必须远小于 1.0 m 匹配阈值。"""
        threshold = math.radians(float(self.settle["quiescent_yaw_deg"]))
        # 判定窗口内还能有 quiescent_yaw 的漂移，之后按同速率再走一个窗口是
        # 最坏情况的粗上界。
        worst_residual = 2.0 * threshold
        displacement = TYPICAL_SOURCE_RANGE_M * math.sin(worst_residual)
        self.assertLess(
            displacement, 0.1 * MATCH_THRESHOLD_M,
            "残余航向 %.3f 度在 %.1f m 处产生 %.3f m 位置误差"
            % (math.degrees(worst_residual), TYPICAL_SOURCE_RANGE_M,
               displacement))

    def test_the_budget_can_actually_outlast_the_measured_rebound(self):
        """预算不够等完回弹的话，这个等待就只是浪费时间。"""
        timeout = float(self.settle["timeout_sim"])
        # mf61 最大回弹 13.44 度在 t+3.7 左右进入阈值；留一倍余量。
        self.assertGreaterEqual(
            timeout, 2.0 * 3.7,
            "预算 %.1f 仿真秒不足以等完实测 %.2f 度的回弹"
            % (timeout, MEASURED_WORST_REBOUND_DEG))
        minimum = float(self.settle["minimum_hold_sim"])
        self.assertLessEqual(minimum, timeout)
        self.assertGreaterEqual(
            minimum, 2.0,
            "最小保持时间短于 2.0 仿真秒会放过 mf61 transfer 1->2 的假象")

    def test_the_settle_never_becomes_a_wall_clock_judgement(self):
        """和 test_wall_clock_backstops 同一条纪律，在这里再钉一次：
        两个预算都必须以仿真秒为主判据、墙钟只做 10 倍兜底。"""
        for base in ("minimum_hold", "timeout"):
            sim = float(self.settle["%s_sim" % base])
            wall = float(self.settle["%s_wall" % base])
            self.assertGreaterEqual(
                wall / sim, 10.0,
                "%s 的墙钟兜底只有 %.1f 倍" % (base, wall / sim))

    def test_the_pre_transfer_turn_tolerance_was_not_repurposed(self):
        """回弹会把航向误差带出 transfer_turn_tolerance 的 0.20 rad 带
        （mf61 收在 0.168 rad，之后又走了 0.235 rad）。如果有人把等停
        实现成「延长在容差内的保持时间」，转身控制器就会去追回弹。"""
        elevator = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))["elevator"]
        self.assertLessEqual(
            float(elevator["transfer_turn_settle_wall"]), 1.0,
            "transfer_turn_settle 是「有没有扫过目标」的确认，不是等停")
        self.assertGreater(
            math.radians(MEASURED_WORST_REBOUND_DEG),
            float(elevator["transfer_turn_tolerance"]) -
            math.radians(MEASURED_WORST_REBOUND_DEG),
            "实测回弹足以把航向误差推出容差带，这就是它不能靠容差保持来等的原因")


if __name__ == "__main__":
    unittest.main()
