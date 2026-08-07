#!/usr/bin/env python3
"""电梯侧扫的收敛判据必须带滞环，否则机身在容差边界横跳永不收敛。

mf77 实测：航向误差在 **0.23..0.27** 之间反复跨越 0.25 容差线，15 仿真秒
一次都没攒满 settle 就超时，整轮报废。机制是「误差一进容差就立刻发零并开始
计时」，机身惯性回弹后误差又出容差、计时清零，如此往复——那一段
`/cmd_vel_behavior` 42% 时间在发非零转向、58% 在发零，误差却始终停在 0.25
附近不下降。指令链路无辜：bag 里 behavior→muxed→/cmd_vel 三级同值送达。

与 A13（换层等停）同族：判据在误差刚进容差时撒手，而机身还会回弹。
"""
from pathlib import Path
import re
import unittest

NODE = (Path(__file__).resolve().parents[1] / "scripts"
        / "frontier_explorer_node.py")

# mf77 实测的误差序列（rad，取自 ELEVATOR_SCAN_HEADING 日志）
MF77_ERRORS = (-0.227, -0.231, -0.235, -0.238, -0.260, -0.256, -0.260,
               -0.256, -0.261, -0.269, -0.263, -0.259, -0.266, -0.243,
               -0.239, -0.249, -0.246, -0.252, -0.251, -0.259)
VIEW = 0.25


# mf78 实测：滞环把误差从 0.25 推进到 0.133 后机身**完全静止**
# （current yaw 恒为 −1.418），差 enter gate 0.125 只有 0.008 rad 转不过去。
MF78_ERRORS = tuple([-0.133, -0.132, -0.133, -0.133, -0.133, -0.132] * 6)


def run_loop(errors, enter, view, stall_frames=None, eps=0.005):
    """返回 (最长连续保持的帧数, 是否曾判定收敛)。enter==view 即旧逻辑。

    stall_frames 为 None 表示不启用「进展停滞则接受」的兜底。
    """
    stable = None
    best_err = None
    last_improve = 0
    best = 0
    for i, err in enumerate(errors):
        if best_err is None or abs(err) < best_err - eps:
            best_err = abs(err)
            last_improve = i
        stalled = (stall_frames is not None
                   and (i - last_improve) >= stall_frames)
        if stable is None:
            conv = abs(err) <= enter or (stalled and abs(err) <= view)
        else:
            conv = abs(err) <= view
        if conv:
            if stable is None:
                stable = i
            best = max(best, i - stable + 1)
        else:
            stable = None
    return best, stable is not None


class ScanHysteresisTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = NODE.read_text(encoding="utf-8")
        start = cls.source.index("def refine_scan_heading")
        cls.body = cls.source[start:start + 6000]

    def test_the_old_single_threshold_never_holds_on_mf77_data(self):
        """复现故障：单一容差下，实测误差序列攒不出连续保持。"""
        best, _ = run_loop(MF77_ERRORS, VIEW, VIEW)
        self.assertLessEqual(
            best, 4,
            "单一容差在 mf77 序列上竟能连续保持 %d 帧；该序列的复现前提已变，"
            "请重新采样" % best)

    def test_hysteresis_stops_the_boundary_oscillation(self):
        """滞环下，误差 0.23..0.27 全部落在进入门限之外——机器人会继续转，
        而不是在 0.25 边界上撒手横跳。"""
        enter = VIEW * 0.5
        best, _ = run_loop(MF77_ERRORS, enter, VIEW)
        self.assertEqual(
            best, 0,
            "进入门限 %.3f 本应把全部 |err|>=0.227 的样本挡在外面，"
            "使控制器继续转向" % enter)

    def test_once_inside_the_wider_gate_keeps_counting(self):
        """一旦进到严门限内，回弹到宽容差以内仍应继续累计，不清零。"""
        errors = (0.10, 0.20, 0.24, 0.20, 0.10)   # 先进入，再回弹但不超 view
        best, holding = run_loop(errors, VIEW * 0.5, VIEW)
        self.assertEqual(best, len(errors),
                         "回弹到 %.2f（仍在 view %.2f 内）不该清零计时"
                         % (0.24, VIEW))
        self.assertTrue(holding)

    def test_source_uses_two_gates_and_validates_them(self):
        self.assertIn("enter_tolerance", self.body,
                      "refine_scan_heading 没有滞环的进入门限")
        self.assertRegex(
            self.body,
            r"if stable_since is None:\s*\n\s*converged = abs\(error\) <= "
            r"enter_tolerance",
            "尚未进入时必须用**严**门限判定，否则 mf77 的边界横跳会复现")
        self.assertIn("must be", self.source[
            self.source.index("enter_yaw_tolerance"):][:900],
            "enter_yaw_tolerance 缺少「必须严于 view」的启动校验")

    def test_the_timeout_message_names_both_clocks(self):
        """旧文案只报 wall，害我把一次仿真钟到期误读成墙钟问题。"""
        idx = self.source.index("did not converge to")
        msg = self.source[idx:idx + 260]
        self.assertIn("sim", msg)
        self.assertIn("wall", msg)


class ScanStallFallbackTest(unittest.TestCase):
    """机器人转不到任意精度：误差不再改善时必须接受它的能力极限。

    只调阈值是把卡点挪个位置——mf77 卡在 0.25（横跳）、mf78 卡在 0.133
    （静止）。stall 兜底对两者同时成立，所以它比滞环更根本。
    """

    ENTER = VIEW * 0.5
    STALL = 3          # 帧；源码里是 stall_sim 仿真秒

    def test_mf78_stalls_just_outside_the_enter_gate(self):
        """复现 mf78：误差 0.133 恒定，仅比进入门限 0.125 大 0.008。"""
        self.assertGreater(abs(MF78_ERRORS[0]), self.ENTER)
        self.assertLess(abs(MF78_ERRORS[0]), VIEW)
        best, _ = run_loop(MF78_ERRORS, self.ENTER, VIEW, stall_frames=None)
        self.assertEqual(
            best, 0, "没有 stall 兜底时，mf78 的恒定误差本应永不收敛")

    def test_stall_fallback_accepts_mf78(self):
        best, holding = run_loop(
            MF78_ERRORS, self.ENTER, VIEW, stall_frames=self.STALL)
        self.assertGreater(
            best, 0,
            "误差 %.3f 已在 view %.2f 之内且长期不改善，应被接受为能力极限"
            % (abs(MF78_ERRORS[0]), VIEW))
        self.assertTrue(holding)

    def test_stall_fallback_also_rescues_mf77(self):
        """同一道兜底也覆盖 mf77 的边界横跳——那里误差同样长期不改善。"""
        best, _ = run_loop(
            MF77_ERRORS, self.ENTER, VIEW, stall_frames=self.STALL)
        self.assertGreater(best, 0, "stall 兜底应同时解决 mf77 的横跳")

    def test_stall_never_accepts_outside_the_view_tolerance(self):
        """兜底只放宽「多久」，绝不放宽「多准」：超出 view 的一律不接受。"""
        errors = tuple([-0.40] * 30)      # 恒定但远超 view=0.25
        best, holding = run_loop(
            errors, self.ENTER, VIEW, stall_frames=self.STALL)
        self.assertEqual(best, 0)
        self.assertFalse(
            holding, "stall 不得成为放宽 view_yaw_tolerance 的后门")

    def test_source_has_the_stall_fallback(self):
        source = NODE.read_text(encoding="utf-8")
        body = source[source.index("def refine_scan_heading"):][:7000]
        self.assertIn("stall_sim", body)
        self.assertIn("stalled and abs(error) <= view_tolerance", body,
                      "stall 兜底必须仍受 view_tolerance 约束")


if __name__ == "__main__":
    unittest.main()
