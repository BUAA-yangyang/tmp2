#!/usr/bin/env python3
"""入口横向重定心的配置不变量。

mf64 教训：tolerance 与 probe_step 取了同一个数(0.05)，于是搜索给出的最小偏移
就等于到位容差，|remaining| <= tolerance 在第一次循环即成立，日志写着
\"entry recentre reached +0.000 m of +0.050 m\" —— 一步没走就宣布完成，
重定心退化成空操作。两次尝试全部作废，那一轮入口能过是靠等地图更新。

这类「两个常量取了同一个数」与 A7–A11 同族：数值本身都合理，错的是它们之间
的关系没有被任何东西约束。
"""
import unittest
from pathlib import Path

import yaml

CONFIG = Path(__file__).resolve().parents[1] / 'config' / 'exploration.yaml'
# navigation 的 footprint 半宽；重定心不得把机身推出走廊。
NEAR_FIELD_HALF_WIDTH = 0.30
# cmd_mux/config/guard.yaml 既有的横向速度上限。
GUARD_MAX_VEL_Y = 0.30


class EntryRecenterConfigTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        document = yaml.safe_load(CONFIG.read_text(encoding='utf-8'))
        cls.entry = document['entry']
        cls.recenter = cls.entry['recenter']

    def test_tolerance_is_finer_than_the_search_step(self):
        tolerance = float(self.recenter['tolerance'])
        step = float(self.recenter['probe_step'])
        self.assertLess(
            tolerance, step,
            '到位容差 %.3f >= 搜索步长 %.3f：最小偏移会在第一次循环就判定到达，'
            '重定心变成空操作(mf64 实测)' % (tolerance, step))
        self.assertLessEqual(
            tolerance, 0.5 * step,
            '容差应至少比步长细一倍，否则最小偏移只剩不到 2 倍分辨力')

    def test_the_sidestep_stays_inside_the_existing_speed_envelope(self):
        speed = float(self.recenter['speed'])
        self.assertGreater(speed, 0.0)
        self.assertLessEqual(
            speed, GUARD_MAX_VEL_Y,
            '横向速度 %.2f 超过 cmd_vel_guard 既有的 max_vel_y %.2f；'
            '重定心不得要求放宽任何包线' % (speed, GUARD_MAX_VEL_Y))

    def test_the_search_bound_cannot_push_the_body_out_of_a_corridor(self):
        max_offset = float(self.recenter['max_offset'])
        self.assertGreater(max_offset, float(self.recenter['probe_step']))
        self.assertLess(
            max_offset, 2.0 * NEAR_FIELD_HALF_WIDTH + 0.30,
            '搜索上限 %.2f m 过大：闸门只保证探测框自由，'
            '不保证这么远之外仍在走廊内' % max_offset)

    def test_attempts_are_bounded(self):
        attempts = int(self.recenter['max_attempts'])
        self.assertGreaterEqual(attempts, 1)
        self.assertLessEqual(attempts, 3, '重定心次数必须有界，否则会在门口来回蹭')

    def test_the_hold_budget_still_bounds_the_whole_thing(self):
        """重定心只能在 obstacle_hold_timeout 的预算内发生，不得延长它。"""
        hold = float(self.entry['obstacle_hold_timeout'])
        per_attempt = float(self.recenter['timeout_sim'])
        attempts = int(self.recenter['max_attempts'])
        self.assertLessEqual(
            per_attempt * attempts, hold,
            '%d 次 x %.1f s 的重定心超出 %.1f s 的等待预算，'
            '会把失败判据推后' % (attempts, per_attempt, hold))


if __name__ == '__main__':
    unittest.main()

class EscapeBandCentreTest(unittest.TestCase):
    """挪到可行区间的中点，而不是第一个可行的偏移。

    mf67 实测：可行横向偏移区间 −0.29..−0.01 m（宽 0.28 m，中点 −0.150），
    而「第一个可行」返回 −0.05 —— 离区间边界只有 0.04 m。机器人挪过去后
    1 秒内就被重新判堵，此时机身已抵住门框，第二次横移只走了 0.003 m，
    整轮死于 `entry remained explicitly occupied for 30.0 sim s`。
    区间中点则两侧各留 0.14 m。

    这与「先前那次 tolerance 等于 probe_step」是**两个不同的缺陷**：
    那次是一步没走，这次是走到了边缘。都要各自钉住。
    """

    STEP = 0.05

    @classmethod
    def band_centre(cls, clearing):
        """复刻节点里的取中点逻辑，便于直接用实测数据验算。"""
        if not clearing:
            return None
        nearest = min(clearing, key=abs)
        band = [nearest]
        for direction in (1, -1):
            probe = nearest
            while True:
                probe = probe + direction * cls.STEP
                if not any(abs(v - probe) < 1e-9 for v in clearing):
                    break
                band.append(probe)
        return sum(band) / len(band)

    def test_mf67_band_would_now_send_the_body_to_the_middle(self):
        clearing = [-0.05, -0.10, -0.15, -0.20, -0.25]
        centre = self.band_centre(clearing)
        self.assertAlmostEqual(centre, -0.15, places=6)
        self.assertGreater(
            abs(centre - max(clearing)), 0.08,
            "中点距区间边缘不足 0.08 m，跟踪误差会立刻把闸门重新堵上")

    def test_a_single_clearing_offset_is_returned_unchanged(self):
        self.assertAlmostEqual(self.band_centre([-0.05]), -0.05, places=9)

    def test_no_clearing_offset_yields_none(self):
        self.assertIsNone(self.band_centre([]))

    def test_a_disconnected_far_band_is_not_averaged_in(self):
        """远处另有一段可行区间时不得平均进来——那会指向一个中间堵着的方向。"""
        clearing = [-0.05, -0.10, 0.30, 0.35]
        self.assertAlmostEqual(self.band_centre(clearing), -0.075, places=6)

    def test_the_centre_sits_further_from_the_band_edge_than_the_first(self):
        """余量要相对**中点所在的那一段**算，不是相对整个可行集合。

        可行偏移可以有不连续的多段（中间被门框隔开），把远处那段的端点
        也当边界是错的——我第一版断言就是这么写的，被自己这条测试抓到。
        """
        for clearing in ([-0.05, -0.10, -0.15, -0.20, -0.25],
                         [0.05, 0.10, 0.15],
                         [-0.10, -0.05, 0.05, 0.10]):
            centre = self.band_centre(clearing)
            first = min(clearing, key=abs)
            band = [first]
            for direction in (1, -1):
                probe = first
                while True:
                    probe = probe + direction * self.STEP
                    if not any(abs(v - probe) < 1e-9 for v in clearing):
                        break
                    band.append(probe)
            lo, hi = min(band), max(band)
            margin_centre = min(abs(centre - lo), abs(centre - hi))
            margin_first = min(abs(first - lo), abs(first - hi))
            self.assertGreaterEqual(
                margin_centre, margin_first,
                '区间 %s：中点余量 %.3f 应不小于首个可行的 %.3f'
                % (band, margin_centre, margin_first))
            # 中点的定义决定了它到两端等距
            self.assertAlmostEqual(abs(centre - lo), abs(centre - hi), places=9)

class FloorCompletionWindowTest(unittest.TestCase):
    """判定「这层探完了」的确认窗口必须长于候选的失败冷却期。

    mf70 三楼：走廊 frontier 进入 4.0 s 冷却，2.0 s 后楼层就被判定探完，
    而候选再过 2 s 就会恢复、却已经没机会了。三楼因此只探到 world y=19.4，
    三个危险源距机器人 6.60 / 12.97 / 15.06 m，全部在相机 5.5 m 上限之外，
    识别 1/4、prob 0.25、识别分 0。

    与本文件 tolerance/probe_step 那条同族：两个常量各自合理，
    错在它们之间的关系无人约束。
    """

    @classmethod
    def setUpClass(cls):
        document = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
        cls.frontier = document["frontier"]

    def test_the_completion_window_outlasts_the_failure_cooldown(self):
        window = float(self.frontier["stable_no_frontier_duration"])
        cooldown = float(self.frontier["failure_cooldown"])
        self.assertGreater(
            window, cooldown,
            "探完确认窗口 %.1f s 不长于失败冷却期 %.1f s：冷却中的候选会被"
            "当成不存在，楼层在它恢复前就收工" % (window, cooldown))
        self.assertGreaterEqual(
            window, 2.0 * cooldown,
            "窗口 %.1f s 相对冷却期 %.1f s 余量不足一倍" % (window, cooldown))

    def test_the_window_admits_enough_distinct_maps(self):
        """确认还要求若干帧互不相同的地图；floor_mapping 约 0.5 Hz。"""
        window = float(self.frontier["stable_no_frontier_duration"])
        needed = int(self.frontier["empty_confirmations"])
        self.assertGreaterEqual(
            window, needed * 2.0,
            "窗口 %.1f s 装不下 %d 帧地图(每帧约 2 仿真秒)" % (window, needed))

