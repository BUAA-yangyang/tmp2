#!/usr/bin/env python3
"""探索 ROI 必须装得下整个楼层，而且各层不能莫名其妙地不一致。

mf65 实测这个框正在切掉全楼层 58% 的待探索边界：
    ROI 内 free 84181 格，frontier(邻接未知)  806
    ROI 外 free 16224 格，frontier(邻接未知) 1133   = 91.3 m2
逐房间看，component_cells 与 roi_allowed_cells 的差值从 640 涨到 1914 格
(3.6→10.8 m2)，而房间面积只有 23.5–46.1 m2。房间因此生成不出候选，
报「no frontier candidate was generated」退出——操作员看到的
「没探索完就出去了」就是这么来的。

它一直是这个值(mf61 那轮也是)，不是新改坏的，只是从来没人量过它切掉多少。
"""
from pathlib import Path
import unittest

import yaml

CONFIG = Path(__file__).resolve().parents[1] / 'config' / 'exploration.yaml'
# mf65 实测已建图可通行区域：x -1.21..38.61 (39.8 m)，y -8.56..9.81 (18.4 m)
MEASURED_FLOOR_WIDTH_M = 18.4
MEASURED_FLOOR_LENGTH_M = 39.8
# 官方 competition-rules.md：楼栋约 20 m x 36 m
OFFICIAL_FOOTPRINT_WIDTH_M = 20.0


def extent(polygon):
    xs = polygon[0::2]
    ys = polygon[1::2]
    return (min(xs), max(xs), min(ys), max(ys))


class RoiExtentTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.roi = yaml.safe_load(CONFIG.read_text(encoding='utf-8'))['roi']
        cls.margin = float(cls.roi['boundary_margin'])

    def test_floor_zero_roi_covers_the_measured_floor(self):
        x0, x1, y0, y1 = extent(self.roi['default_local_polygon'])
        width = (y1 - y0) - 2 * self.margin
        length = (x1 - x0) - 2 * self.margin
        self.assertGreaterEqual(
            width, MEASURED_FLOOR_WIDTH_M,
            'ROI 可用宽度 %.2f m 装不下实测楼层 %.2f m，'
            '框外的 frontier 永远不会被探索' % (width, MEASURED_FLOOR_WIDTH_M))
        self.assertGreaterEqual(
            length, MEASURED_FLOOR_LENGTH_M,
            'ROI 可用长度 %.2f m 装不下实测楼层 %.2f m' % (length, MEASURED_FLOOR_LENGTH_M))

    def test_floor_zero_roi_covers_the_official_footprint(self):
        _x0, _x1, y0, y1 = extent(self.roi['default_local_polygon'])
        self.assertGreaterEqual(
            (y1 - y0) - 2 * self.margin, OFFICIAL_FOOTPRINT_WIDTH_M - 1.0,
            '官方楼栋约 20 m 宽，ROI 至少要接近它')

    def test_all_floors_use_the_same_lateral_half_width(self):
        """一楼曾是 ±8.65 而上层是 ±10.5，同一栋楼窄了 1.85 m。

        两个入口的**纵向**范围本来就该不同(一楼锚点在大门内侧、上层锚点在
        走廊中段，后方还有电梯厅)，但**横向**是同一栋楼的同一个宽度，
        不一致只可能是遗漏。
        """
        _a, _b, y0, y1 = extent(self.roi['default_local_polygon'])
        _c, _d, ey0, ey1 = extent(self.roi['elevator_entry_local_polygon'])
        self.assertAlmostEqual(
            y1 - y0, ey1 - ey0, places=6,
            msg='一楼 ROI 宽 %.2f m 与上层 %.2f m 不一致' % (y1 - y0, ey1 - ey0))

    def test_the_anchor_is_not_on_the_rear_edge(self):
        """后边界 x=0 正好切在 entry pose 上，入口内侧那一段会被排除。"""
        x0, _x1, _y0, _y1 = extent(self.roi['default_local_polygon'])
        self.assertLess(x0, 0.0, 'ROI 后边界必须在 entry pose 之后，否则入口区在框外')


if __name__ == '__main__':
    unittest.main()
