"""换层契约:按层状态必须在 reset_action_state 里归零,上层返航必须交给 mission。

探索器是**一个长驻节点依次服务 0/1/2 层**（日志里的 "reusable floor transaction"）。
任何在运行中被改写、又没在换层时重置的字段，都会把上一层的结论带进下一层。
mf46 暴露的 corridor_probe_barren 就是这一类：只在 __init__ 归零，跨层累加。

沿用本仓库既有的源码级契约测试模式（见 a1_localization/test/localization_contract_test.py、
a1_floor_mapping/test/floor_mapping_contract_test.py）。这类测试不需要 ROS，
在 CI 和本机都能跑，而且能在**加新状态时**就把人拦下来，而不是等到某一轮仿真。
"""

from __future__ import annotations

import re
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]   # .../src/a1
NODE = ROOT / "exploration/scripts/frontier_explorer_node.py"
MISSION = ROOT / "mission_manager/scripts/multifloor_mission_node.py"

# 明确判定为"非按层状态"的字段。加进这里等于声明：这个字段跨层保留是有意的。
# 想加新条目请写明理由，不要为了让测试变绿而加。
NOT_PER_FLOOR = {
    # ROS 句柄与定时器：整个节点生命周期唯一
    "server", "lock", "tf_buffer", "tf_listener", "status_timer",
    "trajectory_timer", "make_plan", "move_client", "dwa_reconfigure",
    "entry_speed_limiter", "floor_mapping_reset",
    # 最新一帧传感/状态缓存：由各自回调持续覆盖，不是按层累积量
    "map_message", "doorway_message", "wall_message", "mapping_status",
    "final_command", "command_freshness", "controller_ready",
    "controller_ready_stamp", "controller_ready_freshness", "safety_locked",
    "last_mapping_healthy_wall", "last_mapping_healthy_sim",
    "map_margin_cache",
    # 对外发布的状态字符串：每次 transition() 覆盖
    "state", "state_message",
    # 按**房间**重置（比按层更细）：explore_room_transaction 开头就归零，
    # 源码里有明确注释 "Coverage is per room: a previous room's covered cells
    # must not make this one look already seen."
    "camera_covered", "last_room_transaction_proven",
}


def _class_block(lines, header_pattern):
    """取出匹配到的 def/class 块（按缩进）。"""
    start = None
    indent = 0
    out = []
    for i, line in enumerate(lines):
        m = re.match(header_pattern, line)
        if m and start is None:
            start = i
            indent = len(line) - len(line.lstrip())
            continue
        if start is not None:
            if line.strip() and (len(line) - len(line.lstrip())) <= indent:
                break
            out.append(line)
    return out


def _assigned(lines):
    names = set()
    for line in lines:
        for m in re.finditer(r"self\.([a-zA-Z_][a-zA-Z0-9_]*)\s*=(?!=)", line):
            names.add(m.group(1))
    return names


class FloorStateResetContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.lines = NODE.read_text().split("\n")
        cls.explorer = _class_block(cls.lines, r"^class FrontierExplorer\b")
        cls.init = _assigned(_class_block(cls.explorer, r"^    def __init__\("))
        reset_block = _class_block(cls.explorer, r"^    def reset_action_state\(")
        cls.reset = _assigned(reset_block)
        # 跟进 reset_action_state 直接调用的辅助函数：configure_floor_completion()
        # 就是这样设置 floor_completion_target 的，靠名字匹配会误报。
        for helper in re.findall(r"self\.([a-zA-Z_][a-zA-Z0-9_]*)\(", "\n".join(reset_block)):
            cls.reset |= _assigned(
                _class_block(cls.explorer, r"^    def %s\(" % re.escape(helper)))

    def test_the_corridor_probe_state_mf46_exposed_is_reset(self):
        """mf46 的具体漏网项，逐个点名，防止回退。"""
        for field in ("corridor_probe_barren", "corridor_probe_exhausted",
                      "corridor_probe_known_before"):
            self.assertIn(
                field, self.reset,
                "%s 是按层状态，必须在 reset_action_state 里归零；"
                "它此前只在 __init__ 初始化，于是跨层累加" % field)

    def test_known_per_floor_state_stays_reset(self):
        """已经做对的部分，不允许在后续重构中被删掉。"""
        for field in ("visited_goals", "failed_goals", "completed_room_branches",
                      "unproven_room_branches", "roi_polygon_map", "corridor_model",
                      "start_pose", "floor_entry_pose", "active_room_branch",
                      "remembered_room_doorways", "maximum_corridor_progress"):
            self.assertIn(field, self.reset, "%s 的换层重置丢了" % field)

    def test_no_new_unreset_runtime_state(self):
        """新增的运行时状态必须要么重置、要么明确声明为非按层。

        这一条是给未来的人看的：加一个会被运行时改写的字段而忘了重置，
        测试就会在这里失败，并把字段名报出来。
        """
        body = "\n".join(self.explorer)
        init_block = "\n".join(_class_block(self.explorer, r"^    def __init__\("))
        runtime_mutated = set()
        for name in self.init:
            outside = len(re.findall(r"self\.%s\s*=(?!=)" % re.escape(name), body)) \
                - len(re.findall(r"self\.%s\s*=(?!=)" % re.escape(name), init_block))
            if outside > 0:
                runtime_mutated.add(name)
        unreset = runtime_mutated - self.reset - NOT_PER_FLOOR
        self.assertEqual(
            set(), unreset,
            "以下字段在运行中被改写但换层时不重置，会把上一层的结论带进下一层："
            "%s。请在 reset_action_state 里归零，或在 NOT_PER_FLOOR 里写明"
            "为何跨层保留是有意的。" % sorted(unreset))


class UpperFloorReturnContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.mission = MISSION.read_text()

    def test_upper_floors_hold_their_endpoint_for_the_elevator_chain(self):
        """上层不得使用单层的 LEGACY_RETURN_TO_START。

        那个模式会让探索器走 execute_return()：先回室内入口锚点，**再回室外
        出发点**。机器人在二层时，室外出发点在一层楼外。mf46 就死在这句
        "returning to outdoor start pose"，而 mission 自己的电梯返航
        (return_upper_floor_over_traversed_segments) 在下一行，永远没轮到。
        """
        self.assertIn("goal.completion_mode = goal.STAY_ON_FLOOR", self.mission)
        self.assertNotIn("goal.LEGACY_RETURN_TO_START", self.mission,
                         "多层任务不应再引用单层返航契约")

    def test_the_mission_still_owns_the_upper_floor_elevator_return(self):
        """把返航交出去的前提是 mission 确实接得住。"""
        self.assertIn("return_upper_floor_over_traversed_segments", self.mission)
        self.assertIn("INSIDE_ELEVATOR", self.mission)


class NavigationBudgetContractTest(unittest.TestCase):
    """导航预算必须以仿真时间计量，墙钟只做兜底。

    动作该花多久是物理问题，不该取决于评测机有多快。mf47 就死在这上面：
    开了 RViz 让 RTF 从 0.238 掉到 0.186，同一句 180 s 墙钟超时从买到 42.8
    仿真秒变成 33.5，而“电梯大堂接近”在 mf46 成功时实测需要 33.6 仿真秒，
    于是它在 16.5 s 处被砍断，move_base 当时还是 ACTIVE。
    """

    @classmethod
    def setUpClass(cls):
        cls.mission = MISSION.read_text()
        cls.config = (ROOT / "mission_manager/config/multifloor.yaml").read_text()

    def test_the_primary_deadline_is_sim_time(self):
        self.assertIn("deadline_sim = rospy.Time.now()", self.mission)
        self.assertIn("rospy.Time.now() < deadline_sim", self.mission)

    def test_a_wall_backstop_still_bounds_a_stalled_clock(self):
        """仿真钟停了的话，仿真预算永远不会到期，必须有墙钟兜住。"""
        self.assertIn("deadline_wall", self.mission)
        self.assertIn("time.monotonic() < deadline_wall", self.mission)

    def test_the_wall_only_navigation_timeout_is_no_longer_active(self):
        """检查它不再是生效的键，而不是字面不得出现。

        沿革说明写在注释里是有价值的（下一个人才知道为什么换掉），所以这里
        用键的模式匹配，而不是粗暴的子串断言——那样会把自己的注释判成违规。
        """
        import re
        self.assertIsNone(
            re.search(r"^\s*navigation_timeout_wall\s*:", self.config, re.M),
            "navigation_timeout_wall 不应再是生效的配置键")
        self.assertNotIn('"~mission/navigation_timeout_wall"', self.mission,
                         "代码不应再读取墙钟专用的导航超时")

    def test_the_sim_budget_covers_the_measured_worst_case(self):
        """mf46 实测最长一段 33.6 s，预算必须留出余量。"""
        import re
        m = re.search(r"navigation_timeout_sim:\s*([0-9.]+)", self.config)
        self.assertIsNotNone(m, "配置里缺少 navigation_timeout_sim")
        self.assertGreaterEqual(
            float(m.group(1)), 2.0 * 33.6,
            "仿真预算需覆盖 mf46 实测最长导航段(33.6 s)的两倍以上")

    def test_the_failure_message_says_which_budget_expired(self):
        """两个阈值语义不同，报错必须能区分，否则下次还要靠猜。"""
        self.assertIn("sim budget", self.mission)
        self.assertIn("wall backstop", self.mission)


if __name__ == "__main__":
    unittest.main(verbosity=2)
