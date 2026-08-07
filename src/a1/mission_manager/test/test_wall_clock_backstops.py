#!/usr/bin/env python3
"""墙钟兜底不得成为主判据。

这条规则被违反过五次，而且每次都是"沿用旁边的数值，没有验算 RTF"：

    factor 3.0  需 RTF > 0.333  elevator/return_alignment/turn_timeout
    factor 3.0  需 RTF > 0.333  upper_floor/opening_alignment/turn_timeout
    factor 4.0  需 RTF > 0.250  mission/move_base_handover_timeout
    factor 4.5  需 RTF > 0.222  elevator/transfer_turn_timeout
    factor 4.5  需 RTF > 0.222  upper_floor/opening_alignment/active_scan_timeout

而这套仿真实测的 RTF 是 0.165–0.238（mf46 0.238 / mf49 0.209 / mf51 0.186 /
mf53 0.165），一个都不满足。于是这五个墙钟值全都变成了主判据，各自成为一颗
定时炸弹：mf54 死在 active_scan——90 墙钟秒在 RTF 0.165 下只买到 14.85 仿真秒，
360 度扫描转到 339.9 度就被掐断，仿真钟那 20 秒预算根本没用上。

墙钟兜底的唯一职责是"/clock 停摆时别永远等下去"。只要 /clock 还在走，
仿真预算就必须先到期。所以每个 X_timeout_wall 相对 X_timeout_sim 的倍数
必须至少是 MINIMUM_FACTOR，也就是容忍 RTF 低到 1/MINIMUM_FACTOR。
"""
from pathlib import Path
import unittest

import yaml


CONFIG = Path(__file__).resolve().parents[1] / "config" / "multifloor.yaml"

# navigation_wall_factor 和 exploration 的 timeouts/wall_factor 都用 10.0，
# 即容忍 RTF 低到 0.10。所有成对的墙钟兜底对齐到同一个数。
MINIMUM_FACTOR = 10.0
WORST_MEASURED_RTF = 0.165  # mf53
# navigation/config/dwa_local_planner_params.yaml；latch_xy_goal_tolerance: true
DWA_XY_GOAL_TOLERANCE = 0.45


def _flatten(node, path=""):
    if isinstance(node, dict):
        for key, value in node.items():
            child = "%s/%s" % (path, key) if path else key
            for item in _flatten(value, child):
                yield item
    else:
        yield path, node


class WallClockBackstopTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        flat = dict(_flatten(yaml.safe_load(CONFIG.read_text(encoding="utf-8"))))
        cls.sims = {k[:-len("_sim")]: float(v)
                    for k, v in flat.items() if k.endswith("_sim")}
        cls.walls = {k[:-len("_wall")]: float(v)
                     for k, v in flat.items() if k.endswith("_wall")}
        cls.pairs = sorted(set(cls.sims) & set(cls.walls))
        # 第二种写法：X_sim 搭配一个**倍数** X_wall_factor，而不是绝对秒数。
        # 这个测试原来只认第一种，于是 mission/action_timeout 用倍数写法加进来
        # 时它一声不吭 —— 而那正是 mf56 死掉的那个预算。两种写法都要查。
        cls.factors = {k[:-len("_wall_factor")]: float(v)
                       for k, v in flat.items() if k.endswith("_wall_factor")}
        cls.flat = flat

    def test_the_pairs_are_actually_discovered(self):
        """守卫这个测试本身：改名/挪层级不能让它静默失效。"""
        self.assertGreaterEqual(len(self.pairs), 5)
        for expected in ("upper_floor/opening_alignment/active_scan_timeout",
                         "elevator/return_alignment/turn_timeout",
                         "mission/move_base_handover_timeout"):
            self.assertIn(expected, self.pairs)

    def test_every_sim_budget_has_a_backstop_in_one_of_the_two_forms(self):
        """每个 *_sim 预算都必须配一个兜底，绝对秒数或倍数都行，但不能没有。

        mission/action_timeout 曾经是纯墙钟 1800 s 没有仿真预算，mf56 的三楼
        探索因此只买到 209 仿真秒（那段 RTF 约 0.12），而二楼几分钟前用掉了
        226.8 秒。漏掉一个就够死一轮。
        """
        # *_max_age_sim 是「这份感知数据还算不算新鲜」的门限，不是等待预算，
        # 没有对应的墙钟概念，也不该有。
        orphans = [base for base in self.sims
                   if not base.endswith("max_age")
                   and base not in self.walls and base not in self.factors]
        self.assertEqual(
            orphans, [],
            "这些仿真预算没有任何墙钟兜底：%s" % orphans)

    def test_factor_style_backstops_meet_the_same_bar(self):
        offenders = ["%s: factor %.1f 需 RTF > %.3f" % (base, f, 1.0 / f)
                     for base, f in sorted(self.factors.items())
                     if f < MINIMUM_FACTOR]
        self.assertEqual(
            offenders, [],
            "倍数写法的墙钟兜底同样必须 >= %.0f：\n  %s"
            % (MINIMUM_FACTOR, "\n  ".join(offenders)))
        # 倍数必须真的挂在某个仿真预算上，否则它谁也没保护。
        for base in self.factors:
            self.assertIn(
                base, self.sims,
                "%s_wall_factor 没有对应的 %s_sim" % (base, base))

    def test_the_a_plane_check_admits_what_movebase_may_legally_produce(self):
        """后置条件不能比导航系统的保证还严，否则健康的进车也会失败。

        DWA 在距 A_safe 的 xy_goal_tolerance 内 latch，而 A_safe 只比 A 深
        upper_transfer_safe_inset。两者之差就是一次合法停车可能出现的最坏
        短缺量，容差必须覆盖它。mf59 三层全探索跑完后差 0.02 m 报废。
        """
        elevator = self.flat
        inset = float(elevator["elevator/upper_transfer_safe_inset"])
        tolerance = float(elevator["elevator/upper_transfer_plane_tolerance"])
        worst_legal_shortfall = DWA_XY_GOAL_TOLERANCE - inset
        self.assertGreater(
            tolerance, worst_legal_shortfall,
            "A 平面容差 %.2f m 覆盖不了最坏合法短缺 %.2f m "
            "(xy_goal_tolerance %.2f - inset %.2f)"
            % (tolerance, worst_legal_shortfall,
               DWA_XY_GOAL_TOLERANCE, inset))
        # 但也不能大到失去意义：mf37 那次短了 0.361 m，机身只有约 0.26 m
        # 在开口内，电梯一动就横滚 96 度。
        self.assertLess(tolerance, 0.20, "容差过大，会放过 mf37 那类真实危险")

    def test_no_bare_wall_only_task_budget_remains(self):
        """任务预算不许只有墙钟。服务等待可以，物理任务不行。"""
        self.assertNotIn("mission/action_timeout_wall", self.flat,
                         "action_timeout 必须是仿真预算 + 倍数兜底")

    def test_every_wall_backstop_tolerates_the_measured_rtf(self):
        offenders = []
        for base in self.pairs:
            sim, wall = self.sims[base], self.walls[base]
            self.assertGreater(sim, 0.0, base)
            factor = wall / sim
            if factor < MINIMUM_FACTOR:
                offenders.append(
                    "%s: sim %.1f / wall %.1f -> factor %.1f 需 RTF > %.3f"
                    % (base, sim, wall, factor, 1.0 / factor))
        self.assertEqual(
            offenders, [],
            "墙钟兜底必须 >= %.0f 倍仿真预算，否则在实测 RTF %.3f 下会变成主判据：\n  %s"
            % (MINIMUM_FACTOR, WORST_MEASURED_RTF, "\n  ".join(offenders)))

    def test_the_worst_measured_rtf_still_leaves_the_sim_budget_in_charge(self):
        """用实测最差 RTF 直接算一遍，而不是只信 factor。"""
        for base in self.pairs:
            sim, wall = self.sims[base], self.walls[base]
            wall_buys_sim_seconds = wall * WORST_MEASURED_RTF
            self.assertGreaterEqual(
                wall_buys_sim_seconds, sim,
                "%s: 在 RTF %.3f 下 %.1f 墙钟秒只买到 %.1f 仿真秒，"
                "少于 %.1f 秒的仿真预算 —— 墙钟会先掐断"
                % (base, WORST_MEASURED_RTF, wall, wall_buys_sim_seconds, sim))



# 允许保持**纯墙钟**的预算：它们等的是墙钟事件，不是物理过程。
# 台账 §4.1 的例外条款：「等 ROS 服务响应本来就该用墙钟，因为服务响应是墙钟
# 事件不是物理事件」。节点重启、控制器状态切换同理——进程什么时候答应你，
# 与仿真里的机器人走了多远无关。
LEGITIMATE_WALL_ONLY = {
    # 等 ROS 服务 / 节点进程
    "mission/service_timeout",
    "mission/localization_recovery_timeout",
    "mission/mapping_recovery_timeout",
    "startup/localization_timeout",
    "startup/mapping_timeout",
    "startup/stand_attempt_timeout",
    "elevator/detection_timeout",
    # 观测新鲜度门限，不是等待预算
    "elevator/template_min_age",
    # settle：只确认「不是扫过目标」，很短，后面都另有复检兜底。
    # 台账 E7 记录它们仍是墙钟；A13 已在换层那处加了独立的等停机制，
    # 所以这里保留的是确认窗口，不是等物理过程结束。
    "elevator/transfer_turn_settle",
    "elevator/return_alignment/turn_settle",
    "upper_floor/opening_alignment/turn_settle",
    "upper_floor/opening_alignment/active_scan_settle",
}


class OrphanWallClockTest(unittest.TestCase):
    """每个纯墙钟预算都必须被显式认领。

    这个检查是本文件原先的**盲区**：它只查「每个 _sim 有没有兜底」，
    从不查「每个 _wall 有没有 _sim 主判据」。于是
    upper_floor/opening_alignment/wait_wall: 20.0 这种纯墙钟预算一直没人看见，
    直到 mf68 死在它手里——20 墙钟秒在 RTF 0.151 下只买到 3 仿真秒，而稳定
    开口检测要 3 帧互不相同的地图、地图 0.5 Hz，至少要 6 仿真秒，
    **先天不可能满足**。

    本族到 mf68 为止已经致命 7 次（A7 A8 A9 A10 B6 + 开门对准）。每一次的
    数值单看都合理，错的是判据用了错误的钟。
    """

    @classmethod
    def setUpClass(cls):
        flat = dict(_flatten(yaml.safe_load(CONFIG.read_text(encoding="utf-8"))))
        cls.sims = {k[:-len("_sim")] for k in flat if k.endswith("_sim")}
        cls.walls = {k[:-len("_wall")] for k in flat if k.endswith("_wall")}
        cls.factors = {k[:-len("_wall_factor")] for k in flat
                       if k.endswith("_wall_factor")}

    def test_every_wall_only_budget_is_explicitly_claimed(self):
        orphans = self.walls - self.sims - self.factors
        unclaimed = sorted(orphans - LEGITIMATE_WALL_ONLY)
        self.assertEqual(
            unclaimed, [],
            "这些预算只有墙钟、没有仿真钟主判据。若它等的是物理过程，"
            "请补 X_sim 并把墙钟放到 10 倍；若它等的确实是 ROS 服务或节点"
            "进程，请加进 LEGITIMATE_WALL_ONLY 并写明理由：\n  %s"
            % "\n  ".join(unclaimed))

    def test_the_whitelist_does_not_rot(self):
        """白名单里的条目必须真的还存在，否则它会掩盖后来同名的新参数。"""
        stale = sorted(LEGITIMATE_WALL_ONLY - self.walls)
        self.assertEqual(
            stale, [],
            "白名单里这些键已不在配置中，请删除：%s" % stale)

    def test_the_opening_alignment_budget_can_collect_three_maps(self):
        """mf68 的具体教训：预算必须够采到 stable_samples 帧互不相同的地图。

        floor_mapping 的 OccupancyGrid 实测约 0.5 Hz（每 2 仿真秒一帧）。
        """
        flat = dict(_flatten(yaml.safe_load(CONFIG.read_text(encoding="utf-8"))))
        budget = float(flat["upper_floor/opening_alignment/wait_sim"])
        samples = int(flat["upper_floor/opening_alignment/stable_samples"])
        needed = samples * 2.0
        self.assertGreaterEqual(
            budget, 2.0 * needed,
            "预算 %.1f 仿真秒采 %d 帧地图（每帧约 2 仿真秒）需要 %.1f 秒，"
            "余量不足一倍" % (budget, samples, needed))

if __name__ == "__main__":
    unittest.main()
