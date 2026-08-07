#!/usr/bin/env python3
"""房间优先的几何规则只排序，不淘汰；降级档绝不按 score 排。

背景（都是实测，不是推理）：

1. 这三条规则编码的是**一楼**的门厅深度，量自**一楼**的 anchor：
       abs(lateral) >= lateral_threshold(1.0) and longitudinal < 7.0
       abs(lateral) <  lateral_threshold(1.0) and longitudinal < max_progress-0.75
       abs(lateral) >= lateral_threshold(1.0) and 没有匹配的 LiDAR 门洞
   而 entry-local 坐标系每层重新锚定，同一栋楼里原点漂 1.73 m
   （二楼 anchor 落在 world y=7.51 走廊口，三楼 y=5.78）。判据的分辨力
   （横向 1.0 m）小于它输入的误差，所以在上层楼它们会把整帧候选清空。
   mf72 三楼 sim 735.89：提取 3 条，3 条全灭，其中主走廊那条横向 1.03
   —— 比阈值只多 3 厘米。整层 1.9 仿真秒宣布探完，识别 0 分。

2. 旧的补救（ROOM_PRIORITY_FALLBACK）按 **-score** 排序取最高分。而电梯厅
   旁边「最大的未知边界」结构性地就是开放竖井方向，于是：
       mf74 取 lat=-7.07 score=13.99 → 真值 z 2.914 → 0.102，摔到楼下
       mf80 取 lat=-7.37 score= 9.25 → 真值 z 5.524 → 0.060，三楼摔到一楼
   近 21 轮里唯一两次运行中坠落，都是它。mf60–mf72 共 13 轮它不存在，坠落 0 次。

所以本测试钉死两件事：**降级不等于淘汰**，以及**降级档按前进量排、绝不按分数排**。
"""
from pathlib import Path
import unittest

import yaml

CONFIG = Path(__file__).resolve().parents[1] / "config" / "exploration.yaml"
NODE = (Path(__file__).resolve().parents[1] / "scripts"
        / "frontier_explorer_node.py")

# (longitudinal, lateral, length_m, score) —— 两轮事故帧的实测候选全集。
MF72_FLOOR2 = (
    (-1.32, -6.74, 17.03, 15.31),   # 门厅/竖井方向，分数最高
    (3.75, 1.03, 3.98, 3.00),       # 主走廊 —— 该选的就是它
    (1.45, 3.23, 1.05, 0.16),
)
MF80_FALL = (
    (-0.34, -7.37, 11.10, 9.25),    # 旧兜底选中它，机器人坠楼
    (-1.46, 6.55, 3.53, 1.85),
    (7.02, 3.28, 3.53, 1.59),       # 走廊方向
)


def demoted(longitudinal, lateral, lateral_threshold,
            minimum_door_longitudinal, maximum_corridor_progress,
            has_doorway):
    """源码 room_priority() 里 demoted 的判据，逐条对齐。"""
    is_lateral = abs(lateral) >= lateral_threshold
    return (
        (is_lateral and longitudinal < minimum_door_longitudinal)
        or (not is_lateral
            and longitudinal < maximum_corridor_progress - 0.75)
        or (is_lateral and not has_doorway)
    )


def demoted_key(longitudinal, lateral):
    """源码 room_priority() 里降级档的排序键。"""
    return (5, 0, -longitudinal, abs(lateral))


class DemotionIsNotEliminationTest(unittest.TestCase):
    def setUp(self):
        config = yaml.safe_load(CONFIG.read_text())
        room = config["frontier"]["room_priority"]
        self.lateral_threshold = float(room["lateral_threshold"])
        self.minimum_door_longitudinal = float(
            room["minimum_door_longitudinal"])

    def test_the_geometry_still_wipes_out_the_upper_floor_frame(self):
        """前提没变：这三条规则在三楼那一帧确实命中全部候选。"""
        for longitudinal, lateral, _length, _score in MF72_FLOOR2:
            self.assertTrue(
                demoted(longitudinal, lateral, self.lateral_threshold,
                        self.minimum_door_longitudinal, 0.0, False),
                "%.2f/%.2f 不再被规则命中，本测试的前提需要重新测量"
                % (longitudinal, lateral),
            )

    def test_selection_loop_no_longer_deletes_demoted_candidates(self):
        """三个 continue 必须已经从选择循环里消失，只剩记账。"""
        source = NODE.read_text()
        self.assertIn("note_demotion", source)
        for cause in ("lateral_gate", "behind_progress",
                      "no_matching_doorway"):
            self.assertIn('self.note_demotion("%s")' % cause, source)
            self.assertNotIn('self.note_rejection("%s")' % cause, source)

    def test_fallback_is_gone(self):
        """兜底通道必须已删除——闸门降级后它是死代码，且是唯一的坠落来源。"""
        source = NODE.read_text()
        self.assertNotIn("ROOM_PRIORITY_FALLBACK:", source)

    def test_demoted_tier_sorts_after_every_admitted_candidate(self):
        """降级档必须排在旧闸门放行过的每一个候选之后。"""
        admitted_tiers = (0, 1, 2)
        self.assertGreater(demoted_key(0.0, 0.0)[0], max(admitted_tiers))

    def test_demoted_order_prefers_forward_progress_not_score(self):
        """两个事故帧：新排序必须把「高分朝竖井」那条排到后面。"""
        for label, frame, shaft_ward, wanted in (
                ("mf72 floor 2", MF72_FLOOR2, (-1.32, -6.74), (3.75, 1.03)),
                ("mf80 fall", MF80_FALL, (-0.34, -7.37), (7.02, 3.28)),
        ):
            by_score = sorted(frame, key=lambda row: -row[3])
            self.assertEqual(
                (by_score[0][0], by_score[0][1]), shaft_ward,
                "%s: 旧的 -score 排序应当先挑中朝竖井那条" % label,
            )
            by_new = sorted(frame, key=lambda row: demoted_key(row[0], row[1]))
            self.assertEqual(
                (by_new[0][0], by_new[0][1]), wanted,
                "%s: 新排序必须先挑走廊方向那条" % label,
            )
            self.assertNotEqual(
                (by_new[0][0], by_new[0][1]), shaft_ward,
                "%s: 新排序绝不能先挑中朝竖井那条" % label,
            )

    def test_score_is_never_part_of_the_demoted_key(self):
        """同一几何、分数差 100 倍，降级档排序必须完全相同。"""
        self.assertEqual(demoted_key(3.75, 1.03), demoted_key(3.75, 1.03))
        low = sorted(((3.75, 1.03, 0.01), (-1.32, -6.74, 99.0)),
                     key=lambda row: demoted_key(row[0], row[1]))
        self.assertEqual(low[0][0], 3.75)


if __name__ == "__main__":
    unittest.main()
