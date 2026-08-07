#!/usr/bin/env python3
"""「建图多久没有健康证据」必须按仿真时间计，不能按墙钟。

mf66 实测：探索器在 sim 71.99 抛出 `floor mapping health lost for 3.55 s`
并终止整轮，而同一时段 bag 里
    /a1/floor_mapping/status   60.08→71.98 全程 MAPPING / healthy
    /a1/localization/status    60.08→71.98 全程 TRACKING / HEALTHY
**建图和定位一次都没降级**。真实原因是探索器进程在 RTF 0.151 下被 CPU 抢占
（该轮同时开着 RViz + 前视相机 + 另一个仿真容器），3.55 墙钟秒没跑到回调。

3.55 墙钟秒在 RTF 0.151 下只等于 **0.54 仿真秒**——机器人的物理处境毫无变化。
按墙钟计的判据会随评委机器的负载而变，这是 A7–A11 那一族的第六例。
"""
import math
from pathlib import Path
import unittest

import yaml

CONFIG = Path(__file__).resolve().parents[1] / "config" / "exploration.yaml"

# 台账 §4.1：墙钟兜底必须 >= 10 倍仿真预算，容忍 RTF 低到 0.10
MINIMUM_FACTOR = 10.0
WORST_MEASURED_RTF = 0.151          # mf66
# floor_mapping 的 status 发布间隔（实测 bag：60.088/60.288/60.488...）
STATUS_PERIOD_SIM_S = 0.2
# mf66 那次误判的实际数值
MF66_STALE_WALL_S = 3.55


class MappingHealthGraceTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.timeouts = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))["timeouts"]

    def test_the_pure_wall_clock_parameter_is_gone(self):
        """旧的纯墙钟参数必须消失，否则会有人以为改它还有用。"""
        self.assertNotIn(
            "mapping_health_grace", self.timeouts,
            "纯墙钟的 mapping_health_grace 必须替换成 _sim/_wall 配对")
        self.assertIn("mapping_health_grace_sim", self.timeouts)
        self.assertIn("mapping_health_grace_wall", self.timeouts)

    def test_the_wall_backstop_never_fires_before_the_sim_budget(self):
        sim = float(self.timeouts["mapping_health_grace_sim"])
        wall = float(self.timeouts["mapping_health_grace_wall"])
        self.assertGreaterEqual(
            wall / sim, MINIMUM_FACTOR,
            "墙钟兜底只有 %.1f 倍，RTF 低于 %.3f 时它会变成主判据"
            % (wall / sim, sim / wall))
        self.assertGreaterEqual(
            wall * WORST_MEASURED_RTF, sim,
            "在实测最差 RTF %.3f 下，%.1f 墙钟秒只买到 %.1f 仿真秒，少于 %.1f 秒预算"
            % (WORST_MEASURED_RTF, wall, wall * WORST_MEASURED_RTF, sim))

    def test_the_mf66_false_positive_would_no_longer_fire(self):
        """把 mf66 那次的实际数值代回去，必须不再触发。"""
        sim = float(self.timeouts["mapping_health_grace_sim"])
        stale_sim = MF66_STALE_WALL_S * WORST_MEASURED_RTF
        self.assertLess(
            stale_sim, sim,
            "mf66 的 %.2f 墙钟秒 = %.2f 仿真秒，仍会超过 %.1f 仿真秒的预算，"
            "误杀不会消失" % (MF66_STALE_WALL_S, stale_sim, sim))
        # 留出至少一倍余量，否则下次负载再高一点又会踩到
        self.assertGreater(
            sim, 2.0 * stale_sim,
            "预算 %.1f 仿真秒相对 mf66 的 %.2f 仿真秒余量不足两倍" % (sim, stale_sim))

    def test_a_genuine_mapping_stall_still_fails_the_action(self):
        """放宽之后仍必须能抓住真正的停更，否则这条判据就没用了。"""
        sim = float(self.timeouts["mapping_health_grace_sim"])
        lost_frames = sim / STATUS_PERIOD_SIM_S
        self.assertGreaterEqual(
            lost_frames, 5.0,
            "预算只够 %.1f 帧，太短，正常抖动就会触发" % lost_frames)
        self.assertLessEqual(
            lost_frames, 40.0,
            "预算长达 %.1f 帧，建图真的停了也太晚才发现" % lost_frames)

    def test_the_budget_is_not_secretly_shorter_than_floor_mapping_s_own(self):
        """floor_mapping 自己有约 3 s 的 input-lost 门限；

        我们的判据必须比它宽，否则它还没来得及把 DEGRADED 报出来，
        我们已经先把整轮判死了。
        """
        self.assertGreaterEqual(
            float(self.timeouts["mapping_health_grace_sim"]), 3.0,
            "必须不短于 floor_mapping 自身的 input-lost 门限")


if __name__ == "__main__":
    unittest.main()
