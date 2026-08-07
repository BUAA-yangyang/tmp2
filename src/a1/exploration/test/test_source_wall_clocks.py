#!/usr/bin/env python3
"""扫**源码**里的墙钟参数，而不只是配置文件。

mf69 死于 `entry/elevator_scan/yaw_timeout_wall`，它的键**从来没有出现在
exploration.yaml 里**，只有代码里 `self.param(..., 30.0)` 的默认值。于是：

  * 对配置文件的墙钟扫描看不见它
  * test_wall_clock_backstops 也看不见它（那个测试读 yaml）

它就这样活到了第八次致命。本族此前七次：A7 A8 A9 A10 B6，以及 mf68 的
opening_alignment/wait_wall。

规则：源码里每出现一个 `X_wall` 参数，就必须同时存在 `X_sim`（或
`X_wall_factor`），否则必须在白名单里写明它等的是墙钟事件。
"""
import re
from pathlib import Path
import unittest

SOURCES = [
    Path(__file__).resolve().parents[1] / "scripts" / "frontier_explorer_node.py",
    Path(__file__).resolve().parents[2] / "mission_manager" / "scripts"
    / "multifloor_mission_node.py",
]

PARAM_RE = re.compile(r'"~?([A-Za-z0-9_/]+)_wall"')
SIM_RE = re.compile(r'"~?([A-Za-z0-9_/]+)_sim"')
FACTOR_RE = re.compile(r'"~?([A-Za-z0-9_/]+)_wall_factor"')

# 等的是墙钟事件，不是物理过程：ROS 服务响应、节点进程重启、控制器状态切换。
# 台账 §4.1 的例外条款。settle 类另见说明。
LEGITIMATE_WALL_ONLY = {
    "elevator/detection_timeout",
    "elevator/template_min_age",
    "mission/localization_recovery_timeout",
    "mission/mapping_recovery_timeout",
    "startup/localization_timeout",
    "startup/mapping_timeout",
    "startup/stand_attempt_timeout",
    "planning/make_plan_retry_delay",
    "planning/make_plan_unavailable_timeout",
    "entry/speed_limit/service_wait",
    "entry/door_centerline/timeout",      # 该功能 enabled:false，且只等 Doorway 消息
    "entry/axis_alignment/timeout",       # 同上，仅在上层楼且默认关闭
    "timeouts/entry_door",                # 等 /set_door_state 服务返回
    "elevator/recenter_step",             # 步进节流，非预算
    "elevator/recenter_timeout",
    # settle：只确认「不是扫过目标」，很短，后面都另有复检或独立等停兜底
    "elevator/transfer_turn_settle",
    "elevator/return_alignment/turn_settle",
    "upper_floor/opening_alignment/turn_settle",
    "upper_floor/opening_alignment/active_scan_settle",
}


class SourceWallClockTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.walls, cls.sims, cls.factors = set(), set(), set()
        cls.missing_files = []
        for path in SOURCES:
            if not path.exists():
                cls.missing_files.append(str(path))
                continue
            text = path.read_text(encoding="utf-8")
            cls.walls |= set(PARAM_RE.findall(text))
            cls.sims |= set(SIM_RE.findall(text))
            cls.factors |= set(FACTOR_RE.findall(text))

    def test_the_sources_were_actually_read(self):
        self.assertEqual(self.missing_files, [], "源文件路径失效，扫描形同虚设")
        self.assertGreater(len(self.walls), 20, "解析到的墙钟参数过少，正则可能失效")

    def test_every_wall_parameter_in_source_has_a_sim_partner(self):
        orphans = self.walls - self.sims - self.factors
        unclaimed = sorted(orphans - LEGITIMATE_WALL_ONLY)
        self.assertEqual(
            unclaimed, [],
            "这些墙钟预算在源码里没有仿真钟主判据。若它等的是物理过程，"
            "请补 X_sim 并让墙钟 >= 10 倍；若它等的确实是 ROS 服务或节点进程，"
            "请加进 LEGITIMATE_WALL_ONLY 并写明理由。\n  "
            "⚠️ 注意：键只写在代码默认值里、不写进 yaml 也会被这条抓到——"
            "mf69 就是这样漏掉的。\n  %s" % "\n  ".join(unclaimed))

    def test_the_whitelist_does_not_rot(self):
        stale = sorted(LEGITIMATE_WALL_ONLY - self.walls)
        self.assertEqual(
            stale, [], "白名单里这些键已不在源码中，请删除：%s" % stale)


if __name__ == "__main__":
    unittest.main()
