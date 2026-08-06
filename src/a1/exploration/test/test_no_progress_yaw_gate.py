"""原地旋转不再无限重置无进展看门狗。

mf44 实测:机器人在 world (6.70, 18.59) 冻结 53 s,期间 /cmd_vel 峰值 vx 1.283、
速度链无截断,而 bounded_backout 一次都没被调用。原因是 DWA 振荡恢复原地转圈,
turned 反复越过 0.35 rad 阈值,把看门狗 anchor 一次次重置,20 s 超时永远到不了。

单独放一个文件,不动 test_frontier_core.py —— 那个文件正在被 room_source_keepout
的工作改动,合进去只会制造冲突。
"""

from __future__ import annotations

import unittest

from a1_exploration.frontier import NoProgressWatchdog


def spinning_watchdog():
    # 与 exploration.yaml 中生效值一致
    return NoProgressWatchdog(timeout_s=20.0, distance_m=0.20, yaw_rad=0.35)


class YawResetsWatchdogTest(unittest.TestCase):
    def test_old_behaviour_is_the_default(self):
        """不传参数时逐位保持旧语义:旋转仍然算进展。"""
        dog = spinning_watchdog()
        dog.observe(0.0, 0.0, 0.0, 0.0)
        seen = dog.observe(1.0, 0.0, 0.0, 0.5)
        self.assertTrue(seen.progressed)
        self.assertFalse(seen.stalled)

    def test_spinning_in_place_used_to_postpone_the_stall_forever(self):
        """复现 mf44:每秒转 0.5 rad,零位移,旧语义下永远不超时。"""
        dog = spinning_watchdog()
        dog.observe(0.0, 6.70, 18.59, 0.0)
        stalled = False
        for step in range(1, 121):          # 120 s,是那次冻结时长的两倍
            seen = dog.observe(float(step), 6.70, 18.59, 0.5 * step)
            stalled = stalled or seen.stalled
        self.assertFalse(stalled, "旧语义本就不会超时,这正是被修的问题")

    def test_spinning_far_from_the_goal_now_reaches_the_timeout(self):
        dog = spinning_watchdog()
        dog.observe(0.0, 6.70, 18.59, 0.0, yaw_counts_as_progress=False)
        stalled_at = None
        for step in range(1, 61):
            seen = dog.observe(float(step), 6.70, 18.59, 0.5 * step,
                               yaw_counts_as_progress=False)
            if seen.stalled and stalled_at is None:
                stalled_at = step
        self.assertIsNotNone(stalled_at, "零位移必须在超时后判定为 stalled")
        self.assertEqual(stalled_at, 20, "应当正好在 20 s 超时处开火")

    def test_translation_still_clears_the_watchdog_when_yaw_is_gated(self):
        """关掉旋转计分后,正常行进不能被误判为卡住。"""
        dog = spinning_watchdog()
        dog.observe(0.0, 0.0, 0.0, 0.0, yaw_counts_as_progress=False)
        for step in range(1, 61):
            seen = dog.observe(float(step), 0.25 * step, 0.0, 0.0,
                               yaw_counts_as_progress=False)
            self.assertFalse(seen.stalled,
                             "0.25 m/s 稳定前进不得被判为卡住(t=%d)" % step)

    def test_final_yaw_alignment_near_the_goal_keeps_the_old_allowance(self):
        """贴近目标时的最后对准仍受保护 —— 这是原判据存在的理由。"""
        dog = spinning_watchdog()
        dog.observe(0.0, 1.0, 1.0, 0.0, yaw_counts_as_progress=True)
        for step in range(1, 61):
            seen = dog.observe(float(step), 1.0, 1.0, 0.4 * step,
                               yaw_counts_as_progress=True)
            self.assertFalse(seen.stalled,
                             "接近目标的原地对准不得被判为卡住(t=%d)" % step)

    def test_gate_cannot_make_the_watchdog_less_sensitive(self):
        """任何一组输入下,关掉旋转计分只可能更早 stalled,不可能更晚。"""
        cases = [
            (0.0, 0.0, 0.0), (0.0, 0.0, 1.2), (0.30, 0.0, 0.0),
            (0.05, 0.05, 0.9), (0.19, 0.0, 0.34),
        ]
        for dx, dy, dyaw in cases:
            with_yaw = spinning_watchdog()
            without_yaw = spinning_watchdog()
            with_yaw.observe(0.0, 0.0, 0.0, 0.0)
            without_yaw.observe(0.0, 0.0, 0.0, 0.0, yaw_counts_as_progress=False)
            a = b = None
            for step in range(1, 61):
                t = float(step)
                x, y, yaw = dx * step, dy * step, dyaw * step
                if a is None and with_yaw.observe(t, x, y, yaw).stalled:
                    a = step
                if b is None and without_yaw.observe(
                        t, x, y, yaw, yaw_counts_as_progress=False).stalled:
                    b = step
            if a is not None:
                self.assertIsNotNone(
                    b, "关掉旋转计分后反而不再 stalled: %s" % ((dx, dy, dyaw),))
                self.assertLessEqual(
                    b, a, "关掉旋转计分后 stalled 变晚了: %s" % ((dx, dy, dyaw),))


if __name__ == "__main__":
    unittest.main(verbosity=2)
