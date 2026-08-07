#!/usr/bin/env python3
"""launch 的 <param> 会覆盖 <rosparam> 从 yaml 载入的同名值。

mf64 实测：我把 config/danger_perception.yaml 的 target_frame 从 map 改成
odom，跑了一整轮，节点日志第一行仍然是 `target_frame=map` —— 因为 launch 里
`<arg name="target_frame" default="map"/>` 配合 `<param .../>` 把它盖掉了。
那一轮所有检测继续在 TF 那一步被丢弃，改动等于没做，而日志里没有任何
一行会告诉你「你改的那个值没被采用」。

凡是被 <param> 覆写的键，两边必须一致；否则改配置的人无从判断改动是否生效。
这与 A7–A11 同族：单看任一处都合理，错的是两处之间的关系无人约束。
"""
import re
from pathlib import Path
import unittest

import yaml

PKG = Path(__file__).resolve().parents[1]
LAUNCH = PKG / "launch" / "danger_perception.launch"
CONFIG = PKG / "config" / "danger_perception.yaml"

ARG_RE = re.compile(r'<arg\s+name="([^"]+)"\s+default="([^"]*)"')
OVERRIDE_RE = re.compile(r'<param\s+name="([^"]+)"\s+value="\$\(arg\s+([^)]+)\)"')


class LaunchParamConsistencyTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        launch_text = LAUNCH.read_text(encoding="utf-8")
        cls.config = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
        cls.args = dict(ARG_RE.findall(launch_text))
        cls.overrides = OVERRIDE_RE.findall(launch_text)

    def test_the_parser_actually_found_the_overrides(self):
        """守卫这个测试本身：改了 launch 的写法不能让它静默失效。"""
        self.assertTrue(self.args, "未解析到任何 <arg default=...>")
        self.assertTrue(self.overrides, "未解析到任何 <param value=$(arg ...)>")
        self.assertIn("target_frame", dict(self.overrides))

    def test_every_overridden_param_agrees_with_the_yaml(self):
        mismatched = []
        for param, arg in self.overrides:
            if param not in self.config or arg not in self.args:
                continue
            if str(self.args[arg]).strip().lower() != \
                    str(self.config[param]).strip().lower():
                mismatched.append("%s: launch 默认 %r vs yaml %r"
                                  % (param, self.args[arg], self.config[param]))
        self.assertEqual(
            mismatched, [],
            "这些键改 yaml 不会生效，会被 launch 的 <param> 覆盖：\n  "
            + "\n  ".join(mismatched))

    def test_the_output_frame_is_one_this_workspace_publishes(self):
        """`map` 在本工作区无任何发布者；`world` 是每代重锚的坏帧(台账 D1)。

        mf63 运行中实测 TF：odom 可解、world 可解但二楼 z 报 0.604、
        map 直接 `does not exist`。检测点转 world 是 result_manager 的职责，
        由 per-generation 的 WorldAnchor 统一负责，所以这里只能是 odom。
        """
        self.assertEqual(self.config["target_frame"], "odom")
        self.assertEqual(self.args.get("target_frame"), "odom")


if __name__ == "__main__":
    unittest.main()
