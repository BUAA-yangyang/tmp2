#!/usr/bin/env python3
"""房间候选按「规划器给出的路径」计代价，而不是按直线。

mf64 的现场：station 10 left 的第三个候选只有 0.68 m 长，直线距离 3.66 m，
算出 score −0.24，压着 −0.5 的准入线过关；而规划器给的路径是 5.03 m。
追它的代价是——机器人走出房间、沿走廊倒退 6.8 m、停在 (10.99, 1.16) 十秒、
再折返，共 22.9 仿真秒，最后还失败。收益是 0.68 m 的 frontier。

操作员当场目视报告「探索完第一个房间出来之后去了走廊入口处，呆了一会，
才回去探索第二个房间，是反常的」，位姿逐帧对账完全吻合。

score 的定义本来就是「信息增益 − 移动代价」，所以问题不在阈值宽严，
而在代价项用了直线——机器人付的是路径的钱。
"""
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))

from a1_exploration.frontier import path_cost_adjusted_score

INFORMATION_GAIN_WEIGHT = 1.0
DISTANCE_WEIGHT = 0.25
MINIMUM_SCORE = -0.5

# mf64 实测：(名称, frontier 长度, 直线距离, 规划路径长度, 日志里的 score)
MEASURED = [
    ('goal1 (22.34, 6.21)', 12.23, 6.05, 8.55, 10.72),
    ('goal2 (13.49, 5.24)', 3.53, 8.60, 9.30, 1.38),
    ('goal3 (10.56, 7.26)', 0.68, 3.66, 5.03, -0.23),
    ('goal4 (23.09,-4.29)', 5.25, 5.89, 7.95, 3.78),
]
FRAGMENT = 'goal3 (10.56, 7.26)'


class PathCostScoreTest(unittest.TestCase):
    def test_the_formula_is_gain_minus_planned_cost(self):
        """代入后必须等于 增益 − 权重×路径长，没有隐藏项。

        容差 0.01 而非更严：表里的 score 抄自日志，是两位小数。例如 goal3
        日志写 −0.23，精确值是 0.68 − 0.25×3.66 = −0.235，这 0.005 的舍入
        会一路传到结果里。用实测数据做回放就得承认实测数据的精度。
        """
        for name, length, ray, path, score in MEASURED:
            adjusted = path_cost_adjusted_score(
                score, ray, path, DISTANCE_WEIGHT)
            expected = INFORMATION_GAIN_WEIGHT * length - DISTANCE_WEIGHT * path
            self.assertAlmostEqual(adjusted, expected, delta=0.01, msg=name)

    def test_a_straight_line_path_changes_nothing(self):
        """路径长度等于直线时必须完全不改变分数——没有绕路就没有惩罚。"""
        for name, _length, ray, _path, score in MEASURED:
            self.assertAlmostEqual(
                path_cost_adjusted_score(score, ray, ray, DISTANCE_WEIGHT),
                score, places=9, msg=name)

    def test_only_the_fragment_is_rejected(self):
        """这是核心：新判据必须精确命中那个碎片，且不误伤其余三个。"""
        rejected, kept = [], []
        for name, _length, ray, path, score in MEASURED:
            adjusted = path_cost_adjusted_score(
                score, ray, path, DISTANCE_WEIGHT)
            (rejected if adjusted < MINIMUM_SCORE else kept).append(name)
        self.assertEqual(rejected, [FRAGMENT],
                         '应当且只应当拒绝那个 0.68 m 碎片')
        self.assertEqual(len(kept), 3, '其余三个候选必须全部保留')

    def test_the_fragment_passed_on_the_ray_and_fails_on_the_path(self):
        """钉住「为什么旧判据放过了它」——否则这个修复的理由会被遗忘。"""
        _n, _l, ray, path, score = [m for m in MEASURED if m[0] == FRAGMENT][0]
        self.assertGreaterEqual(score, MINIMUM_SCORE, '旧判据确实放过了它')
        self.assertLess(
            path_cost_adjusted_score(score, ray, path, DISTANCE_WEIGHT),
            MINIMUM_SCORE, '新判据必须拒绝它')

    def test_the_margin_is_recorded_because_it_is_thin(self):
        """边际只有 0.08，写进测试而不是留在某人的记忆里。

        若将来它变得更薄甚至翻转，这条会先失败，提醒重新看数据，
        而不是让一个碎片重新溜进房间事务。
        """
        _n, _l, ray, path, score = [m for m in MEASURED if m[0] == FRAGMENT][0]
        margin = MINIMUM_SCORE - path_cost_adjusted_score(
            score, ray, path, DISTANCE_WEIGHT)
        self.assertGreater(margin, 0.0)
        self.assertLess(margin, 0.2, '边际薄是已知事实(实测 0.08)，须持续复核')

    def test_a_longer_detour_is_penalised_monotonically(self):
        base = path_cost_adjusted_score(1.0, 5.0, 5.0, DISTANCE_WEIGHT)
        previous = base
        for path in (6.0, 8.0, 12.0, 20.0):
            value = path_cost_adjusted_score(1.0, 5.0, path, DISTANCE_WEIGHT)
            self.assertLess(value, previous)
            previous = value


if __name__ == '__main__':
    unittest.main()
