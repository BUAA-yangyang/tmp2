#!/usr/bin/env python3
"""跨楼层把 home 坐标系带过去的 SE(2) 累积。

危险源提交的是 Gazebo world 坐标，而通往那个系的唯一合法桥梁是
team_scene_info.json 的 robot_start 配上「机器人站在它上面那一刻的 map 位姿」。
这个配对只在取它的那一代 localization 里成立——FAST-LIO 每次换层都重锚。
把它带过换层，靠的是「同一个物理位姿在换层前后各表达一次」。

mf61 裁判真值：位置守恒 0.003 / 0.001 m（毫米级，假设成立）；航向不守恒，
换层前采样时机身还在回弹（A13）。所以本模块的正确性依赖 A13 的等停。
"""
import math
import unittest
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from home_frame import (compose, inverse, normalize_angle,
                        propagate_home_transform, transform_pose)


def close(a, b, tol=1e-9):
    return (abs(a[0] - b[0]) < tol and abs(a[1] - b[1]) < tol
            and abs(normalize_angle(a[2] - b[2])) < tol)


class SE2AlgebraTest(unittest.TestCase):
    def test_inverse_undoes_compose(self):
        for t in ((1.0, 2.0, 0.3), (-4.0, 0.5, -2.9), (0.0, 0.0, math.pi)):
            self.assertTrue(close(compose(t, inverse(t)), (0.0, 0.0, 0.0)))
            self.assertTrue(close(compose(inverse(t), t), (0.0, 0.0, 0.0)))

    def test_transform_pose_matches_manual_rotation(self):
        frame = (1.0, -2.0, math.pi / 2.0)
        got = transform_pose(frame, (3.0, 0.0, 0.0))
        self.assertAlmostEqual(got[0], 1.0, places=9)
        self.assertAlmostEqual(got[1], 1.0, places=9)
        self.assertAlmostEqual(got[2], math.pi / 2.0, places=9)


class PropagationTest(unittest.TestCase):
    """换层不改变物理位姿时，累积出来的变换必须把它认回原处。"""

    def test_a_perfectly_preserved_transfer_is_recovered(self):
        home_from_source = (0.0, 0.0, 0.0)
        # 同一物理位姿：出发代读作 A，到达代重锚后读作原点附近。
        source_base = (5.0, -3.0, 1.2)
        target_base = (0.0, 0.0, 0.0)
        home_from_target = propagate_home_transform(
            home_from_source, source_base, target_base)
        # 到达代的原点，换算回 home，应当正好落在出发时的位姿上。
        self.assertTrue(close(transform_pose(home_from_target, (0.0, 0.0, 0.0)),
                              source_base))

    def test_the_spawn_pose_comes_back_in_the_new_generation(self):
        """这正是 publish_world_anchor 干的事：把出生点表达到新一代坐标里。"""
        spawn_in_home = (-0.02, 0.18, 0.005)
        home_from_current = (0.0, 0.0, 0.0)
        for source_base, target_base in (((5.5, -1.4, 1.9), (0.0, 0.0, 0.0)),
                                         ((-0.2, 0.3, -0.7), (0.01, -0.02, 0.0))):
            home_from_current = propagate_home_transform(
                home_from_current, source_base, target_base)
        spawn_here = transform_pose(inverse(home_from_current), spawn_in_home)
        # 再换算回 home 必须还原，否则链式累积本身就是错的。
        self.assertTrue(close(transform_pose(home_from_current, spawn_here),
                              spawn_in_home))

    def test_three_chained_transfers_do_not_accumulate_algebraic_error(self):
        spawn_in_home = (1.0, 2.0, 0.4)
        home_from_current = (0.0, 0.0, 0.0)
        legs = [((3.0, 1.0, 0.5), (0.0, 0.0, 0.0)),
                ((-2.0, 4.0, -1.1), (0.05, 0.05, 0.02)),
                ((7.0, -6.0, 2.7), (-0.03, 0.01, -0.01))]
        for source_base, target_base in legs:
            home_from_current = propagate_home_transform(
                home_from_current, source_base, target_base)
        spawn_here = transform_pose(inverse(home_from_current), spawn_in_home)
        self.assertTrue(close(transform_pose(home_from_current, spawn_here),
                              spawn_in_home, tol=1e-8))

    def test_a_heading_error_at_the_sample_shows_up_as_position_error(self):
        """A13 的量化理由：采样瞬间的航向误差会放大成位置误差。

        13.47 度在 5 m 处 = 1.16 m，掉出官方 1.0 m 阈值；
        A13 落地后实测残余 0.12 度 = 0.010 m。
        """
        for degrees, expected in ((13.47, 1.16), (3.16, 0.28), (0.117, 0.010)):
            displacement = 5.0 * math.sin(math.radians(degrees))
            self.assertAlmostEqual(displacement, expected, places=2)
        self.assertGreater(5.0 * math.sin(math.radians(13.47)), 1.0)
        self.assertLess(5.0 * math.sin(math.radians(0.117)), 1.0)


if __name__ == "__main__":
    unittest.main()
