# A1 多层 demo — 交接续篇（2026-08-04，接 HANDOFF_20260803）

> 本文只写**本轮新增/推翻**的内容。基础背景仍看 `HANDOFF_20260803_多层demo.md`。
> 标 ✅ 的是**有实测或算术证据**的结论；标 ⚠️ 的是坑。

---

## 0. 一句话现状

mf08 卡的 ROI 报错**根因已定位并修复**，同时定位出**两个此前无人发现的确定性阻塞**
（一个会让 F1 楼层三分之一无法建图、一个会让**每一次换层都必然超时**）。
三项修复已部署、有单测和算术验证。

---

## 1. ⚠️ 推翻 mf08 的判断：那不是「门口有箱子的房间死循环」

mf08 完整报错（日志被截断的部分）：

```
floor 1 exploration failed: FAILED: invalid single-floor exploration ROI:
ROI is not fully contained in OccupancyGrid with 8.00 m sensor margin;
map=[-20.00, 80.05]x[-60.00, 20.03],
outside_vertices=[(-24.230377557152774, -38.292188562522334)]
```

这是**纯几何前置校验**，在 `explore_floor` 派发后 **0.15 秒**触发（t=95.600 → 95.754），
机器人在 F1 上一步都没走过、一个房间都没进过。与「箱子堵门」无关。

---

## 2. ✅ 根因链（用 mf08 实测位姿反解，不是猜）

**关键坐标学**：odom 系在每次定位重初始化时**重锚到机器人当时的位姿**。
F0 的锚点是室外出生点（朝向正对入口，走廊 = +odom X，确定性的）；
F1/F2 的锚点是**电梯轿厢内**的位姿，其 odom 朝向由「轿厢姿态 + 开口对准 + 固定 95° 转身」
三段死推得到，**不是建筑轴向**。

用 mf08 的 `/Odometry_gazebo` 反解出 F1 的 odom→truth 旋转 = **219.25°**
（两种基线互相验证：短基线 219.2°、长基线 219.3°，长度误差 0.8%）。由此：

| 量 | 数值 | 后果 |
|---|---|---|
| F1 楼层在 F1 odom 系的范围 | x∈[−24.8, 12.0], y∈[−31.6, 8.3] | 旧网格 origin_x=−20 → **楼层远端 4.8 m 根本不在栅格里** |
| F1 声明轴向 vs 真实走廊 | 偏 **17.3°** | 20 m 外的门横向偏移 6.8 m ≫ `door_maximum_lateral` 2.2 → 被当成非房间丢弃 |
| DWA `yaw_goal_tolerance` | **0.80 rad = 46°** | 「右转 95°」合法地可以差 46° 收工，这就是 17° 的来源 |
| DWA `xy_goal_tolerance` | 0.45 m | 而 explorer 的 `entry/position_tolerance` 也是 0.45 → 硬门限和到达容差**等值**，mf08 落在 0.34 m 纯属运气 |

---

## 3. ✅ 本轮三项修复（已部署）

### 3.1 栅格改为「绕锚点旋转无关」— `floor_mapping/config/floor_mapping.yaml`

```
width 100→104, height 80→100, origin (−20,−60) → (−50,−50)
```
旧注释假设「F1/F2 转到 −odom Y」，实测不成立。新尺寸由 ROI 反推：
F0 需 +x 方向 51.5 m；F1/F2 需绕锚点**任意方向** 48.7 m。
新的 8 m 内缩框 x∈[−42,46], y∈[−42,42]，内切半径 42 m。
代价：1387×1334 = 1.85 M 格，比原来 **+30%**（栅格与 static global costmap）。

⚠️ **实测 RTF 无变化**（mf08 0.296 / mf10 0.300），这 +30% 没有拖慢仿真。

### 3.2 上层楼专用 ROI 多边形 — `exploration/config/exploration.yaml`

F0 的 ROI 是 `[0,40]×[±8.65]`，**整层都在锚点前方**（锚点在大门内侧）。
F1/F2 的锚点在楼层中段（出电梯 5 m），**身后还有约 10 m 建筑**（含电梯厅）。
沿用 F0 多边形会把这段静默排除在 frontier / 覆盖率分母 / 完成判定之外。

新增 `roi/elevator_entry_local_polygon: [−13,−10.5, 32,−10.5, 32,10.5, −13,10.5]`，
按 `entry_mode == ALREADY_AT_FLOOR_ENTRY` 自动选用。F0 行为完全不变。

### 3.3 ROI 越界从「硬失败」改为「裁剪到传感器余量」— `frontier_explorer_node.py`

ROI 是**故意放大**的矩形，远角本来就落在远墙之外的虚空里。为这种格子废掉整轮
6–8 分钟仿真不划算。改为与内缩掩码取交集：「ROI 格子不得进入距栅格边 8 m 以内」
这条不变量**一字未改**，只是不再连坐整轮。裁剪时 `logwarn_throttle(10s)` 报出丢失格数。

### 3.4 端到端算术验证（`transform_local_polygon` + `point_in_polygon` 实跑）

| 场景 | ROI 在栅格内 | 楼层在 ROI 内 |
|---|---|---|
| F0 入口锚点（未改动） | OK（余量 2.5 m） | OK |
| F1 新 ROI + 轴向已修正 | **OK（余量 5.4 m）** | **OK** |
| F1 新 ROI + 轴向未修正（17°） | OK | MISS 一个角 ← 轴向修复是必需的 |
| F1 旧 F0-ROI（任意轴向） | CLIP | MISS 三个角 ← 新多边形是必需的 |

---

## 4. ⚠️ 本轮发现的两个**确定性**阻塞（都不是我们的改动引入的）

### 4.1 catkin relay stub 打碎 sibling import（mf09 整轮死在这）

脚本一旦进 `catkin_install_python`，catkin 会在 `devel/lib/<pkg>/` 生成 565 字节
**relay stub**，roslaunch 之后优先启动那份，`sys.path[0]` 变成 devel 目录。
stub 可执行但**不可 import**（它把源码 `exec()` 进临时 dict，不导出任何名字）。

报错极具误导性：指向 `devel/lib/...` 那个文件，像「构建过期」，其实 src 完全正确。

**最小修复**（已加在 `multifloor_mission_node.py` 所有 sibling import 之前）：
```python
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
```
relay 会把 `__file__` 设成 src 真实路径，两种布局下都成立。
**验证方法**：直接跑 `python3 devel/lib/<pkg>/<node>.py`，
修好后应停在缺 ROS param 的 `KeyError`，而不是 ImportError。

正解是把 helper 移进 `catkin_python_setup()` 的真 Python 包（`a1_exploration` 就是这么做的）。

### 4.2 ✅ 换层必然超时 —— `localization_healthy_after` 卡在一个**从未被发布的键**上

23:10 的改动把两处判据从 fail-open 换成 fail-closed 的 `generation_is_new()`：

```python
# 旧（mf03–mf08 一直在用）              # 新
previous_generation is None or          generation_is_new(gen, prev)
generation is None or                   # 任一为 None/空串 → False
generation != previous_generation
```

但 **`localization_generation` 这个键在整个 localization 包里从未被发布过**
（`grep -rn localization_generation localization/` 零命中）。该包只发 `generation`，
且发在 supervisor 的话题上。所以 `localization_healthy_after` 恒为 False。

mf10 实测：定位在估计器重启后 **11.4 秒**就报 `state=TRACKING reason=HEALTHY`，
mission 仍在 **34 秒之后**以 45 s 墙钟超时告败。**这会让每一次换层都失败**，不是偶发。

旧代码的 `generation is None → True` 恰好遮住了这个不存在的键，判据实际退化为
「TRACKING + 时间戳新鲜」——这才是六轮真正在用的逻辑。

**已修**：该键缺失时回到「时间戳新鲜即可」；`supervisor_running_after` 保持 fail-closed
不动（它读的 `generation` 是真实发布的，mf10 也确实通过了那一关）。

---

## 5. ✅ mf11 实测：三项修复全部生效，卡点前移到房间事务

**F1 探索历史上第一次真正跑起来**（t=100 派发 → t=625 超时，500+ 秒）。

| 证据 | 数值 |
|---|---|
| `UPPER_FLOOR_ENTRY_AXIS_LOCKED` | `correction=-0.350` rad = **−20.1°**（与我从 mf08 独立反解的 17.3° 互证） |
| F1 ROI 覆盖分母 | **159,870** 格（F0 为 115,990）→ 上层多边形被正确选用 |
| `ROI clipped` 警告 | **零次** → 新网格完整装下 F1 的 ROI |
| F1 锚点 | odom (0.08, −4.84)，与预测的 (0.06, −5.22) 吻合 |
| 换层 | z 在 t=68.5 跳到 2.916，§4.2 修复生效 |

⚠️ **`minimum_door_longitudinal: 7.0` 的担心没有成真**：房间在 station=6
（≈锚点前 9 m）被正常识别，通过了门限。这条先不要改。

### 5.1 ⚠️ 新卡点：房间门口活锁 —— 这才是学长说的「呆住」，但**不是箱子**

```
room transaction released: station=6 side=left  completed      ← 4 间里只完成 1 间
room transaction acquired: station=6 side=right                ← 取得后再未释放
room scan deferred: no reachable 0.78 m-clear observation point
    found within 2.5 m                                    × 628 次
room scan/forward exit failed; failure 99/2                    ← 上限本是 2
```

**实测位姿对账（状态标签不算证据）**：t=140–625 的 486 秒里，机器人
**只移动了 1.1 m**，被困在 truth (1.26–1.84, 14.87–14.96) 的 0.58×0.08 m 盒子里，
1668 个采样 100% 落在同一个 2×2 m 格。

| 观测 | 值 | 排除了什么 |
|---|---|---|
| `/cmd_vel` 非零采样 | 1668 中仅 34（**2%**） | 不是走不动/横向死区 |
| 脚下地图格值 | **0（free）**，1668/1668 | 不是箱子堵门 |
| 前方 0.6 m 格值 | **0（free）**，1668/1668 | 前方无障碍 |
| 倾角 | roll ≤2.5°, pitch ≤2.4° | 没摔 |
| move_base 报错 | `Failed to get a plan` / `Rotation cmd in collision` **零次** | 规划器没抱怨 |

**根因**：机器人停在门框位置（走廊实测宽 2.138 m，即 x∈[−1.1,1.1]；机器人在 x≈1.5）。
`open_room_scan_pose` 要求先找到半径 **0.78 m 的完全已知自由圆盘**
（`room_priority/scan_clearance: 0.78`，搜索半径 `scan_search_distance: 2.5`）才肯原地旋转。
门洞净宽约 1.2 m（半宽 0.6 m < 0.78 m），房间深处的格子又还是未知——
**先有鸡还是先有蛋：要进去才看得见，要看得见才肯进去。**

同一站位的**左侧房间成功了**（`room scan complete: 6.41 rad`, depth=1.51 m），
所以是临界几何，不是 F1 系统性问题。

### 5.2 ✅ 真正的根因：**多层树跑的是学长的旧版 explorer，不是我们的**

⚠️ 不要去调 `scan_clearance`，那是治标。珺超指出后核实：

| 树 | `scan_room_and_exit`（进门转圈） | `explore_room_transaction`（frontier 房间事务） |
|---|---|---|
| **xjc_multifloor_20260803**（多层主力） | **被调用**（5022 行） | **不存在** |
| integration_20260730 | 定义了但**从未调用**（死代码） | 调用于 5428 |
| codex_single_floor_clean_20260803 | 定义了但**从未调用**（死代码） | 调用于 5413 |

我们在单层阶段**早就**把「进门原地转 360°」换成了基于 frontier 的房间事务
（探到房内无 frontier 为止）。多层树是**学长 SimEnv 的 rsync 拷贝**（交接 §2），
所以它跑的是那份为调试整体流程而写的旧实现。多层树比两棵单层树**少约 380 行**。

⚠️ 我一开始只 grep 了函数定义就说「三棵树里都还在」——**那是错的**，
定义还在但已是死代码，要看调用点。

### 5.3 移植计划（范围已量清，依赖闭包干净）

**方向：把我们的房间事务移植进多层树**，保留多层树的入口模式与电梯机制。
不要反过来整份替换——我们的单层树 `ALREADY_AT_FLOOR_ENTRY` **0 处命中**
（F1/F2 出电梯的入口模式），且 `ExploreFloor.action` 两边 md5 不同
（`009c2a85` vs `f3508d05`），整份替换会同时打断 mission↔explorer 契约。
而多层树的电梯/换层链路**已被 mf11 证明能跑到房间事务那一步**，不该推倒。

**第 1 步 — `frontier.py` 整份换成 codex 版**（自带 102 单测）。
已核实我们的版本是多层树那份的**严格超集**，唯一例外是本次会话新加的
`map_margin_mask` 和 `dominant_axis_correction`，换完把这两个函数补回去即可。

**第 2 步 — 移植 12 个节点方法（约 864 行），并把 5022 行的
`scan_room_and_exit` 调用点改成 `explore_room_transaction`：**

```
explore_room_transaction        189    room_free_component_mask        177
select_from_frontiers           124    frontier_is_admissible           61
room_transaction_frontiers       55    camera_coverage_target           51
wait_for_interstitial_zero_settle 50   mark_camera_coverage             46
record_corridor_probe_outcome    35    room_camera_gap                  28
exit_room_through_mouth          27    flattened_goal_pose              21
```

⚠️ `camera_coverage_target` / `mark_camera_coverage` / `room_camera_gap`
是**相机覆盖**逻辑——直接关系识别得分（`prob ≤ 0.6` 即 0 分，必须 ≥4/5），
多层树完全没有这部分。

**第 3 步 — 保留不动**：多层树的 `ALREADY_AT_FLOOR_ENTRY` 分支、
`estimate_corridor_model` / `recenter_in_corridor` / `refine_scan_heading` /
`settle_corridor_center_and_heading`、以及本次会话的栅格/ROI 修复。

⚠️ **不要半途而废**：当前这棵树是已知可用状态（mf11 能跑到 F1 房间事务），
移植要么做完要么不做，别留半成品。

### 5.4 ✅ 移植已完成（2026-08-04）

节点 5193 → 6206 行。备份在 `/home/xiaoyi-dev/simenv/xjc_backup_20260804_preport/`。

**换进来的（我们的单层实现）**
- `frontier.py` 整份 ← codex 版（40 个符号），补回本会话新增的
  `map_margin_mask` / `dominant_axis_correction`
- `final_zero.py` 整份 ← codex 版。⚠️ 它把 `FinalZeroMonitor` 的分量数做成参数，
  **默认 6**；多层树旧版硬编码 3。已同步把 `final_command_callback` 改为观测
  Twist 全部 6 个分量（残留的 `linear.z`/`angular.x/y` 也是非零指令，3 分量看不见）
- 12 个节点方法约 864 行 + 25 个参数/状态
- `/cmd_vel_nav` 订阅 + `nav_command_callback`
  ⚠️ **差点漏掉**：移植进来的 `wait_for_interstitial_zero_settle` 会读
  `interstitial_cmd_vel_nav_monitor`，但多层树里没有任何东西喂它。
  静态检查只查“属性有没有定义”，查不出“定义了但永远不更新”——这种要靠追数据流。
- 配置：`room/*`、`frontier/room_priority/transaction_*` 等，用我们单层树的调参值
- 9 个配套单测文件 + 换掉旧的 `test_final_zero.py`（它测的是被换掉的旧实现）

**保留不动的（学长的多层骨架）**
`ALREADY_AT_FLOOR_ENTRY` / `LEGACY_MAIN_ENTRANCE` 入口模式、`completion_mode`、
`complete_room_branch` / `configure_floor_completion` 的楼层完成计数、
`estimate_corridor_model` / `recenter_in_corridor` / `refine_scan_heading` /
`settle_corridor_center_and_heading`、多层树那份 `ExploreFloor.action`、
以及本会话的栅格/ROI 修复。

**调用点改动**：`self.scan_room_and_exit(` → `self.explore_room_transaction(`，
状态文案 `scanning 360 degrees...` → `opening the room transaction`。
`scan_room_and_exit` 保留为死代码，与我们两棵单层树的状态一致。

**验证**：107 个 exploration 单测 + 6 个 mission 单测通过（原来 42 个）；
节点在容器 ROS 环境下 import 成功；`src` 1871 文件、`.pt` 策略模型 2 个完好。

### 5.5 mf12 暴露的接口不匹配 → `navigate()` 也一并移植

mf12 在 t=115 报 `internal error: navigate() got an unexpected keyword argument
'allow_backout'`。我的第一版静态检查只验证了「方法存在」，**没验证签名兼容**。

⚠️ 而且检查器本身有 bug：`[n for n in tree.body if isinstance(n, ast.ClassDef)][0]`
取到的是文件里第一个类 `InvalidEntryPose`，不是 `FrontierExplorer`，
所以连报两次「0 处不匹配」。**按类名选，不要按位置选。**

修正后的审计（`FrontierExplorer` + 排除 `@staticmethod` 的 self 误算）只找出两处
同名不同签名：`navigate`（多了 `allow_backout`）和 `entry_near_field_clear`
（多了 `direction` / `half_width`，无调用方受影响）。

于是把 `navigate()` 也整份换成我们的版本 —— 它正是「卡死处理：无进展看门狗」的载体：
- `NoProgressWatchdog`：move_base 可能抱着一个到不了的目标一直不发指令，直到 75 s
  超时。实测 2026-08-01 机器人在 world (2.240, 1.011) 站了 24 s 以上、`cmd_vel`
  恒零，而状态标签仍写着 NAVIGATING。原地转向算进展，所以 yaw 也计入。
- 卡住时先 `bounded_backout` 再重试，而不是干等
- 发目标前等 move_base action server（它带 `respawn:=true`，可能正在重启）
- 第三个返回值从 bool 改成 **`"unreachable"` / `"transient"` / `None`** 契约：
  超时和自己取消**不证明目标不可达**，只应触发冷却，不应计入永久排除预算。
  换进来的 `record_failure(..., kind=)` 正好要这个契约。
  13 个调用方里 11 个忽略第三值，两个消费方都是 `elif recordable_failure:` 的
  真值判断，字符串与 bool 都成立，已按我们的方式补上 `kind=` 透传。
- 新增 `navigation/no_progress/{timeout,distance,yaw}` = 20.0 / 0.20 / 0.35

⚠️ 未移植（属于返航路径，不在本次范围）：`reverse_return_transit`、
`indoor_return_anchor`、`outdoor_return_target`、`freeze_return_anchor`、
`nav_command_callback` 以外的返航逻辑，以及 `return/reverse_transit/*` 配置差异
（`align_tolerance` 0.15 vs 0.25、`yaw_speed` 0.35 vs 0.90）。
单层返航本来就只有约 56% 成功率，要修是另一件事。
`frontier/room_priority/goal_extension` 也**故意没改**（多层 0.5 / 我们 1.2）——
它决定房间目标推进多深，先让移植本身单独可归因。

---

### 5.6 ✅ mf13：移植生效，二楼 4/4 房间完成，机器人上到三楼

```
t=93    EXPLORE_FLOOR floor=1
t=130   room transaction goal 1..6      ← frontier 驱动，探到房内无 frontier 为止
t=264   另一间 goal 1..2
t=344   另一间 goal 1..2
t=397.7 FLOOR_COMPLETE floor=1  "completed 4/4 distinct rooms"
t=432   INSIDE_ELEVATOR 返回 point A
t=450.5 FLOOR_SWITCH_VERIFIED floor=2
```

四间房全部走完整事务：`station=5 left/right`、`station=14 left/right`，
depth 1.47–1.54 m。station 换算成 truth y≈14.3 和 27.8，与 F0 实测的房间位置
（y≈15、29）吻合。

**实测位姿对账（状态标签不算证据）**，同一楼层、同一 seed：

| 二楼探索 | mf11（学长旧版·转圈） | mf13（我们的房间事务） |
|---|---|---|
| 仿真时长 | 525 s | **305 s** |
| 实际走过路程 | **14.2 m** | **119.9 m** |
| 走过的 3×3 m 格子数 | 6 | **24** |
| 完成房间 | 1/4 | **4/4** |
| oracle 覆盖 | 困在 0.58×0.08 m 盒子里 | x∈[−8.02, 7.60]，y∈[7.37, 30.29] |

### 5.7 ⚠️ mf13 的终点：**机器人从三楼摔了下去**

日志只报 `navigation fixed-route 5 m transit failed state=4`。实测 oracle 才看得出真相：

```
t=474.0  oracle=(0.90, 7.66)  z=5.52   ← 三楼
t=475.5  oracle=(1.96, 7.30)  z=3.49   ← 下坠中
t=483.0  oracle=(2.01, 7.65)  z=0.06   ← 地面
```

三楼 `ELEVATOR_EXIT_ALIGN_SKIPPED` 之后，固定的 `2 m → 95° → 5 m` 路线按未校正的
朝向出发，把机器人走进了 truth ≈(1.0–2.0, 7.3–7.7) 的洞口。

**这推翻了 `align_to_car_opening` 原本的 fail-open 设计理由**（§5.1 写的是
「错误对准比现状更糟，所以置信度不足就跳过、保持原朝向」）。现在有实测证据：
对准失败后继续盲走，代价是机器人从 5.5 m 摔下去。固定路线只有相对轿厢开口才有意义，
没有那个基准时它就是一段紧挨开放竖井的 7 m 盲走。

**已改（两处都是收紧，不是放宽）**
1. `align_observation_timeout` 20 → 60 s。20 s 墙钟在 RTF≈0.3 下只有约 6 s 仿真时间、
   约 6 帧地图，不够让轿厢壁变成已知、让开口凸显出来。这是**观测预算不是安全余量**：
   一旦找到开口就立刻退出等待（F1 那次不到 2 s）。
2. 对准失败改为 **fail-closed**：emit `ELEVATOR_EXIT_ALIGN_FAILED` 并抛
   `MissionFailure`，而不是盲走。宁可拿到一个可诊断的失败，也不要把机器人丢掉。

### 5.8 mf14：两处收紧都生效，卡点回到学长的固定路线几何

- 二楼 **4/4 房间再次完成**且更快（335 s vs mf13 的 397 s），station=5 与 15 各两间
  → **移植的房间事务是可复现的，不是一次侥幸**
- 三楼对准**成功**：60 s 预算下找到了开口（`92.1 deg correction` → ALIGN_READY），
  没有再触发 fail-closed
- **oracle z 全程 5.51–5.54，没有摔** —— §5.7 的两处收紧按预期起作用

新卡点（= 交接文档 §4 记录的老问题）：

```
t=391–397  位置恒为 truth (1.79, 2.38)，vx 持续 0.37 却零位移  ← 顶住了推 6 s
t=404 起   位置恒为 (1.29, 1.73)，指令归零
move_base: Rotation cmd in collision x98, DWA planner failed to produce path x35
           -> Aborting because a valid control could not be found
```

⚠️ **最值得追的线索**：三楼的对准修正是 **92.1°**，二楼只有 11.1°／−27°。
这个不对称说明三楼的 `opening_bearing` 很可能选错了方向（把侧壁当成开口），
于是机器人侧着出轿厢、顶墙、转不动。

**下一步建议**（按价值排序，都还没做）
1. 查 F2 的 `opening_bearing` 为什么给出 92°：把三楼的占据栅格和 180 条射线的
   自由行程 dump 出来看，而不是猜。工具已有（`align_to_opening.py` 自带自检）。
2. `2 m → 95° → 5 m` 这三个写死的数是在**一楼**几何上标定的
   （`exit_upper_floor_without_doorway` 注释自陈 "in this fixed building layout"）。
   三楼若电梯厅几何不同，这条路线本就不成立，应改成由感知驱动而非几何写死。
3. 以上都属于电梯/换层链路（学长的边界），动之前先对一下。


### 5.9 ✅ dump 结论：出梯不可靠**不是三楼特有**，是每层都靠运气

同一 seed、同一份代码，三轮三个结果：

| 轮次 | 二楼出梯 | 三楼出梯 |
|---|---|---|
| mf13 | ✅ | 走进洞里，摔下楼 |
| mf14 | ✅ | 95° 右转 `Rotation cmd in collision` ×98 |
| mf15 | ❌ **2 m 直行就 abort**（oscillating，1.2/2.0 m） | 没走到 |

这正是交接 §4 记录的「失败点在四个不同步骤间跳」。当时以为是四个问题，dump 证明是同一个。

**`opening_bearing` 测的是传感器覆盖的各向异性，不是轿厢几何。**
mf15 二楼第一次尝试的实测（`/tmp/align_floor1.jsonl`，分析脚本 `analyze_align.py`）：

```
已知格子 4365 / 1,850,258 = 0.24%
射线剖面: 中位 1.088 m，最长 3.000 m（撞上 max_range 上限），对比度 1.912 ≥ 0.80 -> ACCEPTED
180 条里 29 条打满 3.0 m，被平均的瓣张开 -20°..+36°
选中 bearing = 8.0°，而机器人当时朝向 = 0.1°
```

栅格裁剪图显示：机器人左边一堵墙，**右边一大片已观测自由空间**，其余方向很快撞上未知。
射线遇未知即停（有意设计），所以「自由行程最长」≈「地图观测得最远」≈「刚才朝着看的方向」。
于是它系统性地返回≈当前朝向：机器人碰巧对着开口时修正很小（8.0°/11.1°，看起来正常），
朝向偏了就给出同样偏的答案且置信度照样高（−35.0°/+92.1°）。
**四次结果不是四次测量，是同一个无意义量的四次采样。**

而后面的 `2 m → 95° → 5 m` 是**无反馈的航位推算**，初始朝向误差不会被纠正，只会积累到某步撞上东西。

⚠️ 二楼之所以常常「看起来没问题」，是因为固定路线跑完后 mission 侧
`estimate_corridor_axis` 会用墙面重新测轴向、探索 ROI 又足够宽，**能自我纠正**；
三楼救不回来只是因为**顺序**——固定路线里任一步 `navigate()` 失败就直接抛
`MissionFailure`，探索根本没机会开始。

### 5.10 建议的修法：改用**已经写好但从未被调用**的感知驱动出梯

`exit_to_corridor(floor)`（mission 节点第 1107 行）**存在但调用次数为 0**，是死代码。
它做的正是感知驱动的出梯：等目标层电梯门被检测到 → 用
`elevator_poses()` 从**实测门中心**算出 lobby/threshold → 导航出去。
一楼进梯用的就是这套（`lobby_standoff` 0.85 / `car_depth` 1.45），站位居中、朝向正。

**可行性证据**：上层楼的电梯门**确实能被检测到**——
`ELEVATOR_LOCALIZED floor=1` 在 mf13 t=104.3、mf14 t=138.7 都触发了。
只是目前在固定路线**跑完之后**才收敛（mf13 出梯 t=80–93，检测 t=104）。
非零层的接受条件是门距机器人 0.25–2.2 m，而在轿厢内面对门时门约 1.45 m，几何上满足。

所以方案是：上层楼改调 `exit_to_corridor`，让它的
`wait_until(floor in self.elevator, 20.0)` 在**轿厢内**等检测器收敛，
然后由实测门几何驱动出梯，并**删掉 `align_to_car_opening`**（它测的量本就不成立）。

⚠️ 风险：检测器可能在轿厢内收敛不了（视角受限），那就退化成 20 s 后失败。
需要一轮验证。这属于电梯链路（学长的边界），动之前先对一下。


### 5.11 ⚠️ 我说重了的两处（外部审计指出，已核实成立）

| 我的说法 | 实际 |
|---|---|
| 「二楼 4/4 可复现（mf13+mf14）」 | **mf13 的 4/4 掺了一间未证明的房间**：3 间 `no reachable frontier remains` + 1 间 `room transaction could not bound the room; leaving`（t=212.8），却仍宣布 4/4。只有 **mf14 是干净的 4/4** |
| 「dump 已证明所有出梯失败同源」 | dump 只证明**缺陷机制存在**（mf15 二楼实测）。mf15 没到三楼、无 F2 dump，mf14 三楼那 92.1° 只是**强烈嫌疑**，不是已证事实 |

哈希确认：mf13/mf14 的 explorer 相同（`9b858fcf`），mission 不同（`3dae7c5c`/`63e820f8`）
——只能说「房间模块重复表现」，不能说「整套系统同版本复现」。

### 5.12 ✅ P0 已修：未证明的房间不再能关闭楼层（**我移植时引入的**）

移植时我砍掉了单层树的 proven 记账，理由是「保持调用点改动最小」。那是错的：
`explore_room_transaction` 只把 `last_room_transaction_proven` 写进成员变量，
返回值仅表示「有没有从门里出来」，调用方直接 `complete_room_branch()`，
于是**探索器自己承认没探完的房间照样计入 4/4**。`unproven_room_branches`
连声明都跟着参数块带过来了，却从头到尾没人用。

已修（判据抽成纯函数以便单测）：
- `frontier.room_completion_state(completed, unproven, target, revisit_attempts)`
  → `(floor_complete, revivable)`；未证明且复活次数未用尽的房间**不计入配额**
- `complete_room_branch(branch, proven)` 接收事务裁决并维护 unproven 台账
- `choose_frontier` 在宣布「无目标可选」前**复活**一间未证明的房间

新增 `test_room_completion.py` 共 7 例（含 mf13 那个具体场景）。**107 → 114 个单测**。

### 5.13 ⚠️ P1 已修：测试从未进入 catkin 入口

多层树的 `exploration/CMakeLists.txt` **没有任何测试注册**（codex 树注册了 13 个）。
带过来的 12 个测试此前只在手动 `unittest` 时跑，构建抓不到回归。现已注册全部 13 个。

### 5.14 ⚠️ mf16 负面结果：感知驱动出梯**不成立**，已回退

出梯改为完全由实测门几何驱动后，mf16 在二楼
`timeout waiting for floor 1 car opening measured from inside the car` 失败。
`DOORWAY_RAW floor=1` 在 t=79–105 的计数：**0 条**——不是被门限滤掉，
是探测器在轿厢内根本给不出门。

原因是几何的，调参解决不了：门框两侧各需 ≥ `minimum_wall_length` 0.80 m 的墙段，
1.45 m 深的轿厢内视角给不出；`filter/minimum_range: 0.5` 又丢掉近处回波。
**学长把那个函数命名为 `without_doorway`，记录的就是同一个发现。**

已回退到固定路线（树不能停在比 mf14 更差的状态）。回退时保留的改进：
- 上层楼门检测的证据门槛（`stable`/观测数/置信度）+ `ELEVATOR_CANDIDATE` 拒绝原因日志
- 对准失败 fail-closed（防止再次坠楼）
- 三个写死的数改为读配置：`exit_forward`/`corridor_forward`/`corridor_turn_rad`

⚠️ 回退时抓到一个我差点自己引入的行为变更：config 里 `exit_forward: 1.0`，
而代码体里硬编码 **2.0**，改成读参数会把 2 m 变成 1 m。已把配置对齐为实际运行值。

**下一步（未做）**：必须先离开轿厢才能测门——探测器在**外面**确实触发
（mf13 t=104.3、mf14 t=138.7）。方向是用有界、逐步查图的前进替掉盲走 2 m，
出车后由实测门决定走廊方向，替掉那个盲目的 95°。

## 6. 工具层备注

- ⚠️ **mission.log 是块缓冲的**，进程退出前 `tail` 看不到 MULTIFLOOR 行。
  实时状态要看 `pose.csv`（sampler 是行缓冲）。mf08 能看全是因为它已经退出了。
- ⚠️ **sampler 的 `mission` 列一直是空的**：它订阅 `/a1/mission/status`，
  而 mission 节点发布在 `/a1/mission_manager/status`（`~diagnostics/status_topic`）。
  `exploration` 列是好的。修 sampler 时顺手改掉。
- `pose.csv` 的 `x,y,z` 是 **world→base 的 TF**，而 world 系在每次重定位时
  会重锚到出生位姿——**跨楼层不可直接比较**。跨层比较只能用 oracle 列。
  换算：`odom_x = world_y + 3.2`，`odom_y = −world_x`，`odom_yaw = world_yaw − 90°`。
- 部署用 `scp` 单文件，不要用带 `--exclude` 的 rsync（§6.1）。每次部署后核对
  `find src -type f | wc -l` 和 `find src -name "*.pt" | wc -l`（应为 2）。

---

## 7. 协作

⚠️ 2026-08-03 23:09–23:10 有另一个会话（Codex）在同一棵工作树里改了
`mission_manager` 整包（新增 `corridor_axis.py` + 6 个单测、改
`multifloor_mission_node.py`/`CMakeLists`/`package.xml`/`multifloor.yaml`），
与本会话在 exploration 侧的改动**撞车**（同一个轴向问题的两份实现）。

已按拍板处理：**保留 mission 侧 `estimate_corridor_axis`**，
exploration 侧的 `align_entry_axis_to_walls` 代码保留但
`entry/axis_alignment/enabled: false` 关闭（注释写明为何关、何时该开）。
Codex 已停止远端写入，当前该目录单一 writer。

他们的实现比我的更严格（要求一对**相对的**稳定墙夹持 + generation/session 身份校验，
失败 fail-open 回声明轴向并 emit `UPPER_FLOOR_ENTRY_AXIS_UNVERIFIED`），
且同样把锚点改成了 achieved pose——和我独立得出的结论一致。

---

## 8. 轮次记录

| 轮 | 结果 | 失败点 |
|---|---|---|
| mf09 | ✗ | catkin relay stub → ImportError，机器人没站起来（§4.1） |
| mf10 | ✗ | 换层后 `localization_healthy_after` 恒 False，45 s 超时（§4.2）。**RTF 与 mf08 一致，证明 +30% 栅格没有拖慢仿真** |
| mf16 | ✗ | 感知驱动出梯证伪：轿厢内 0 条门检测（§5.14），已回退 |
| mf15 | ✗ | **二楼**出梯 2 m 直行 abort（oscillating）。对准 dump 已采集，见 §5.9 |
| mf14 | ✗ 但**二楼 4/4 可复现，三楼没摔** | 对准 60 s 预算生效（修正 92.1°，ALIGN_READY）；卡在固定路线的 95° 右转（§5.8） |
| mf13 | ✗ 但**二楼完整探索 + 上到三楼** | 移植生效，见 §5.6。摔下三楼（§5.7） |
| mf12 | ✗ | `navigate() got an unexpected keyword argument 'allow_backout'`（§5.5） |
| mf11 | ✗ 但**大幅前进** | 换层成功、F1 探索首次真正运行 500+ s、4 间完成 1 间，卡在右侧房间门口活锁 628 次，耗尽 `action_timeout_wall: 1800 s`（§5.1） |

**卡点推进轨迹**：出梯朝向(mf03–06) → 进轿厢(mf07) → ROI 几何(mf08) →
catkin import(mf09) → 换层定位判据(mf10) → **F1 房间事务(mf11)**。
每一轮都推进到了新的、更靠后的阶段，没有回退。
