# A1 多层 demo — 问题台账

> **维护规则**（2026-08-05 用户要求，持续有效）
> 1. 操作员反馈的每一个现象、分析中发现的每一个问题，**都必须进这个台账**，不论大小、不论当时是否要修。
> 2. 未解决的条目**不删**，只更新状态。后续会话继续沿用本文件。
> 3. 每条必须有 **证据** 字段，落到具体工件的具体位置（bag 的哪个话题哪一帧、日志哪一行、哪个 JSON 字段）。写不出证据的，状态只能是「已观测未定位」。
> 4. 被推翻的假设**保留在 §F**，不要静默删除——重复走同一条死路的成本比留着高。
> 5. 状态取值：`已修已验证` / `已修待验证` / `已定位未修` / `已观测未定位` / `已知未排期` / `已排除`

最后更新：2026-08-06（**mf61 = MISSION_COMPLETE，完整多层 demo 首次跑通**）

---

## A. 已修改代码

| # | 问题 | 状态 | 证据 | 修法与理由 |
|---|---|---|---|---|
| **A1** | 返程时 C 点被多转约 180°：`entry`(点 C) 的 yaw 是走廊向外方向，直接当导航目标 → MoveBase 在 C 转到背对 B，下一个 C→B 目标再转回来 | **已修待验证** | mf49 bag：C 的 yaw −2.2263 rad，C→B 方位角 +0.8151 rad，相差 **174.3°**；yaw 序列 85.1°(621s) → 165.8° → −161.9°(623s) → 129.6° → 71.3° → 57.9°(626s) | `complete_upper_floor_and_return_to_a()` 改用 staging pose：深拷贝 entry，x/y 保留、yaw 换成 `atan2(B−C)`。**不改 entry 本身**，因为 `return_upper_floor_over_traversed_segments()` 还要用它的声明轴向做 C–B 几何校验，改了就等于自己校验自己 |
| **A2** | DWA 在航向未对正时同时加速 → 横向甩出走廊、踩空坠楼 | **已修待验证** | mf49 bag：623.57s `vx=0.409 / wz=−1.698`；624.5s `vx=0.968 / wz=−0.999`；625.27s `vx=1.269`。相对去程实测轨迹横向偏差 0.32→0.47→0.63→0.77 m；625.570s 首次倾角>15°；真值 z 2.915→0.057 | 新增 `align_return_bearing_in_place()`：只发 `angular.z`，**从不写 `command.linear`**，仿真钟 10 s + 墙钟 30 s 双超时，容差 0.12 rad 后零速 settle 0.5 s，`finally` 连发 5 次零 Twist（behavior 通道在 twist_mux 里优先级高于导航）。转向后**重新**校验 C 位置和航向才发目标。独立参数块 `elevator/return_alignment/*`，不与开梯转向共用 |
| **A3** | `path_exists()` 返回 `None`（规划器暂不可用）时，房间被判「已探完」并标记 **proven** | **已修（代码确定）** | 代码读出：`break` 后 target/chosen 均为 None → 落入完成分支 → `last_room_transaction_proven = True`；且 `if chosen is None` 是**死代码**（target is None 时 chosen 必为 None）。mf49 未触发（`make_plan` 失败 0 次） | 置 `planner_unavailable` 标志后重试，事务自身预算兜底，耗尽则留 **unproven**，接上已有的 unproven-room 复活机制。「规划器没回答」不等于「房间探完了」 |
| **A4** | 房间事务期间 RViz 绿色 frontier 是进房前的陈旧快照 | **已修已验证** | 代码：`publish_frontiers()` 全文件仅一处调用，在 `run()` 主循环、进事务之前。操作员 mf49/mf51 目视复现 | 事务内每次算完候选就发布（空集也发）。这不是修 bug，是**让证据可用**——在此之前绿线与内部状态无关，无法作为任何结论的依据 |
| **A5** | 返航段是硬编码占位实现（`2.3 m` / `3.5 m` 重建「大概是起点」），且从未执行过 | **已修已验证** | `run()` 尾部；backlog §2 记载「这段代码从来没有真正执行过」 | 新增 `mission/final_return_to_start`，**默认 false**：`transfer(2,0)` 后直接 `MISSION_COMPLETE`。事件带 `return_to_start_performed=False`。⚠️ 关闭返航后 `exploration_time` **不符合 PDF 定义**，不可用于对分 |
| **A6** | 探索器与 mission 共用 `/move_base`，交接时无握手 → actionlib 状态机错乱 → `get_state()` 返回 ACTIVE(1) 被当成终止状态（原 B4） | **已修待验证** | mf51 三楼 `715.582 FLOOR_COMPLETE` → 同毫秒发 goal → `[ERROR] Received comm state PREEMPTING when in simple state DONE` → `716.584 failed state=1`。**二楼 311.684 报了一模一样的错却侥幸通过** → 时序运气非某层缺陷 | 新增 `settle_move_base_handover()`：发 goal 前若 client 处于 PENDING/ACTIVE/PREEMPTING/RECALLING(0,1,6,7) 则 `cancel_all_goals()` 并等它回到空闲，仿真钟 5 s + 墙钟 20 s 双超时。空闲时是 no-op，不改任何 goal/容差/包线 |
| **A7** | `timeouts/wall_factor: 5.0` 的前提「RTF 约 0.3」从未成立，导致墙钟兜底长期变成主判据，提前掐断本来够用的等待 | **已修待验证** | 实测 RTF：mf46 0.238 / mf49 0.209 / mf51 0.186 / **mf53 0.165**，一次都没到 0.3。factor 5 要求 RTF > 0.20。mf53 死于 `entry remained explicitly occupied for 4.0 sim s (wall fallback 20.0 s)`：4.0/0.165 = **24.2 墙钟秒 > 20 秒兜底**，仿真时间根本没等满 | 改为 **10.0**，与 mission 侧 `navigation_wall_factor` 对齐（"tolerates RTF down to 0.10"）。节点代码默认值本来就是 20.0，是配置把它压到 5。**非放宽安全门限**：全部 10 个使用点（等地图/入口通行/入口障碍等待/规划/backout/房间事务/出房间对齐/goal 超时）都是 `/clock` 停摆兜底 |
| **A8** | mission 侧五个墙钟兜底的倍数是 3.0–4.5，需要 RTF > 0.22–0.33 才能让仿真预算先到期 | **已修待验证** | mf54 死于 `active opening scan 339.9/360.0 deg`：90 墙钟秒在 RTF 0.165 下只买到 14.85 仿真秒，仿真钟那 20 秒预算根本没用上。五个配对：`return_alignment/turn`(3.0)、`opening_alignment/turn`(3.0)、`move_base_handover`(4.0)、`transfer_turn`(4.5)、`active_scan`(4.5) | 全部对齐到 **factor 10**（容忍 RTF 到 0.10）。新增 `test_wall_clock_backstops.py` 自动枚举全部配对并用实测最差 RTF 验算，防止再次「沿用旁边的数值不验算」 |
| **A9** | `entry/obstacle_hold_timeout: 4.0` 的前提「瞬时的门/台阶回波」不成立——主入口门是官方文档写明的约 25 秒插值过程 | **已修待验证** | `/set_door_state` 只用 **26 毫秒**返回（mf55: 12.176 请求 → 12.202 返回），根本没等门动完。mf53/mf55 死于 `entry remained explicitly occupied`；mf54 同位置通过。三轮踩中两轮，撞/过时刻距门响应均约 6.3 秒，差别只在走位 | 改为 **30 仿真秒**（覆盖完整门动作 + 余量；地图 0.5 Hz 即 15 帧新栅格）。等待期间机器人**停着**，不是安全门限只是耐心值：代价上限 30 仿真秒，踩中一次代价是整轮 40+ 分钟 |
| **A10** | `mission/action_timeout_wall: 1800.0` 是纯墙钟、无仿真钟配对，而它是**一层楼探索的总预算**（物理任务时间） | **已修待验证** | mf56 死于 `floor 2 exploration wall timeout`：三楼那段 RTF 约 **0.12**（机器上另有仿真），1800 墙钟秒只买到 209 仿真秒；而二楼几分钟前在 RTF 约 0.25 时用掉了 226.8 仿真秒 | 改为 `action_timeout_sim: 600` + `action_timeout_wall_factor: 10`（600 = 实测单层最大用量 226.8 s 的 2.6 倍）。⚠️ **我漏过两次**：①上轮盘点误把它归为「服务超时」②本轮又用了测试抓不到的 `_wall_factor` 命名。测试已扩到覆盖两种写法 + 检查孤儿 `*_sim` + 统一命名 |
| **A11** | 进轿厢的 A 平面后置条件是**零容差**，而 MoveBase 的 latch 行为可以合法地停在 A 平面前 0.05 m —— 判据比依赖方的保证更严 | **已修已验证** | 算术：`inset 0.40 < xy_goal_tolerance 0.45` 且 `latch_xy_goal_tolerance: true` ⇒ latch 后落点 = A + (−0.05 到 +0.40)。实测四次 `arrival_error` 0.331/0.356/0.368/0.42 全在 0.45 内即全部合法。mf59 三楼余量 **−0.02 m** 整轮报废（此前二楼 +0.071 通过纯属落点运气） | 新增 `upper_transfer_plane_tolerance: 0.10`（覆盖 −0.05 下界并留一倍余量；远小于本检查真正要防的 mf37 −0.361 m，那次机身仅 0.26 m 在开口内、电梯一动横滚 96°）。测试固化上下界：容差必须 > `xy_goal_tolerance − inset` 且 < 0.20。**mf61 验证**：二楼 +0.034、三楼 +0.044，两次都在 ±0.05 区间内晃，容差生效而非运气 |
| **A12** | `upper_transfer_safe_inset` 命名误导：机器人**从未到达过 A_safe** | **已定位未修（仅命名）** | 四次 `arrival_error` 0.331–0.42，从未接近 0 | 它的真实语义是「把 latch 圈中心往轿厢内推 0.40 m」，不是「要走到的点」。功能正确，但读代码时易被名字带偏。改名需同步 yaml/代码/测试，低优先级 |

---

## B. 已定位，未修

| # | 问题 | 状态 | 证据 | 为什么没修 / 下一步 |
|---|---|---|---|---|
| **B1** | lobby rule（`room_minimum_door_longitudinal = 7.0`）把每层**第一个房间**靠电梯厅那一侧整片切出连通域 | **已定位未修** | mf51 五个房间连通域下界：(5,1) **7.00**..13.97、(5,−1) **7.00**..15.32、(6,1) **7.00**..15.65、(6,−1) **7.00**..15.65，而 (14,1) 14.66..28.04、(14,−1) 14.36..28.08。**7.00 精确命中参数值四次**，全部是「每层第一个房间」。面积：station 5/6 房间 23.5–30.8 m²，station 14 房间 45.3–46.1 m²，相差近一倍 | 判据在 `room_free_component_mask()`。它的**全部证据基础来自一楼**（`return_harness01`、`run13`，注释里的电梯井 footprint 是 F0 world 坐标），一楼原点是大门内侧、前 7 m 确是门厅；上层楼原点是点 C，同一条判据切进房间。**不能简单删**：上层电梯厅在 C 后方，也被这个半平面覆盖，删了就裸奔。方案见 §B1-方案。**下一步**：从 bag 取 `map_seq=394 / stamp=423.882` 一帧，量出房间在无此规则时的真实 longitudinal 起点 |
| **B2** | 电梯厅旁的**开放竖井对 2-D 占据栅格不可见**：井口格是 UNKNOWN 不是 occupied，costmap 不反对走进去 | **已定位未修** | 两次坠落：mf13 真值 (1.0–2.0, 7.3–7.7)、mf49 真值 (1.5–2.3, 5.1–5.9)。mf49 实测横向余量约 **0.5 m**（去程 y≈5.9 处 x=0.847，返程同 y 处 x=1.484，偏 0.63 m 即已倾覆），远小于走廊半宽 1.07 m | 修它要么改障碍高度带、要么加楼板边缘识别，两条都会碰**障碍余量**（用户红线）。A1+A2 已消除本次触发方式，但**没有让这个洞变得可见**。需单独排期与决策 |
| **B3** | FAST-LIO 在快速原地转向后、机器人静止时自身位姿跳变，触发 `STATIONARY_TRANSLATION_DRIFT` → 整轮 FAILED | **已定位未修** | mf50 169.51–171.09 s：**真值只动 0.018 m，估计器跳 0.148 m**（门限 0.12）。同轮其余 11 个静止段估计器与真值吻合到毫米。mf49 同期 5 个静止段最大 0.078 m 且一致。170.798 LOST → floor_mapping 降级 → 171.682 探索器 3.5 s 宽限用尽 | 定位模块的检测是**正确**的，不得放宽门限。相关但**非充分**：转向峰值 `wz = −1.800` = `scan_angular_speed`，也正是交接记载的三次摔倒 signature（RL 训练包线 0.9 的 200%）。但 mf50 另 4 次 1.8 转向脱节仅 0.005–0.020 m，mf49 四次 ≤0.037 m，所以 1.8 不是充分条件。**跳过一楼只降低了触发概率，不是修复**。方案 A/B/C 见 §B3-方案 |


### B1-方案（未拍板）

| 方案 | 做法 | 一楼保护 | 上层电梯厅保护 | 房间恢复 |
|---|---|---|---|---|
| A | lobby rule 只在 `LEGACY_MAIN_ENTRANCE`（一楼）生效 | 不变 | **丢失** | 完全 |
| B | 一楼保持 7.0，上层楼单独阈值（覆盖 C 后方电梯厅，不碰前方房间） | 不变 | 保留 | 恢复 |
| C | 探索器接入电梯**实测 footprint**，排除实测矩形而非盲目半平面 | 变强 | 变强 | 完全 |

C 是正解，但探索器目前**拿不到电梯位置**（只有 `elevator_roi_local` ROI 多边形和扫描参数；电梯实测位置在 mission 节点的 `self.elevator[floor]`）。B 是本轮可安全落地的。

### B3-方案（未拍板）

| 方案 | 做法 | 性质 | 把握 |
|---|---|---|---|
| A | `scan_angular_speed` 1.80 → 0.90（回到 RL 训练包线内） | **收紧**，不违反纪律 | 中等，能降概率无法证明能消除 |
| B | 楼层内 LOST 触发重定位恢复而非整轮 FAILED | 很重：重定位会作废探索器的 ROI/房间轴/走廊进度 | 低 |
| C | 当作偶发，多跑几轮 | 无 | 赌 |

---

## C. 已观测，未定位

| # | 问题 | 状态 | 证据 | 下一步 |
|---|---|---|---|---|
| **C1** | 房间事务提前结束，房间里仍有大片 UNKNOWN 灰区 | **已观测未定位**（至少两条独立路径：B1 切连通域、C4 失败反噬） | mf49 (5,−1)：raw=4 → 通过 3 → **选中 0**。mf51 (5,1)：raw 3→**0**，1 个 goal；(6,1)：raw 3→**0**，1 个 goal。操作员三次目视确认灰区残留。**反例**：(5,−1) 同样被 7.00 切但表现尚可 → B1 是必要非充分条件 | 取 bag 一帧，逐格列出「全局有、房间没有」的候选各死在哪一级：lobby rule / lateral 带 / 门站切分 / `obstacle_clearance` 膨胀 / goal 落点找不到 |
| **C2** | `source_keepout`（本轮新增，门内 1.70×±1.20 m）是否连带挡掉房间内部 frontier | **已观测未定位** | mf51 实测挖掉 **434–705 格 = 1.1–1.8 m²**，配置矩形 4.08 m²，即约 27–43% 落在房间连通域内 | 与 C1 同一帧一起归因。数字已有，等对照 |
| **C4** | 追微小 frontier 碎片 → 贴住家具转不过来 → 失败记录反噬，把房间里最后一个候选也拉黑，房间提前结束 | **已定位未修** | mf51 station 15 left：`goal 4: length=0.38 m score=-0.01`（558.984）→ `Rotation cmd in collision` ×14（517.3–519.0）→ 569.684 超时失败 → 569.782 `room transaction complete after 4 goals: all 1 admitted candidates were filtered (score=0 visited=0 **history=1** unreachable=0)`。代价 **10.7 仿真秒**换 0.38 m 信息增益，并赔上整个房间 | 与 C1 的 lobby rule 路径**互相独立**：这条不是「没生成」，是「生成了被自己刚才的失败连累」。涉及 `room_frontier_minimum_score: -0.5`、`min_length: 0.35`、`failed_radius: 0.75`、`record_failure` 的 kind。**先不调阈值**——两条路径并存时分别调参会互相掩盖，等 bag 归因完一起判断。代码注释已记过同类模式（competition run10：三个 0.42–1.13 分碎片烧掉约两分钟墙钟），当时对策「短超时+不退避」治了时间没治失败反噬 |
| **C5** | 出房间时机器人头贴障碍转不过来，`Rotation cmd in collision` 持续刷屏，靠 `bounded_backout` 倒退约 0.36 m 才脱困（操作员目视） | **已观测，按设计工作** | mf51：`bounded backout finished: distance=0.366 m sim_time=1.402 s`（576.684）、`distance=0.357 m sim_time=1.396 s`（604.484），与 `navigation/backout/step_distance: 0.35` 吻合。房间事务内 `allow_backout=False`，两次 backout 均发生在房间释放之后 | 单次代价仅约 1.4 仿真秒，不致命。**保留观察**：若同一位置反复触发或 backout 次数上升，说明入位姿态本身有问题（`goal_clearance: 0.40` / 房间目标推进深度 `goal_extension: 0.5`）。目前不改 |
| **C6** | **未扫完的障碍被当成可达空间**：障碍侧面/背面仍是 UNKNOWN 时，旁边的 free 格是合法 frontier，goal 也通过安全检查；机器人走近后那片 UNKNOWN 变成 occupied，goal 落进障碍，DWA 到不了 → 超时。**这是 C4 和 C5 的共同上游**（操作员提出，代码验证成立） | **已定位未修** | 机制：`extract_frontiers` 的 `unsafe = _dilate(occupied, 0.30/0.05=6格)` 与 `_nearest_safe_cell` **都只作用于已知 occupied**，UNKNOWN 不在其中。特征尺寸小：失败 goal 的 length 为 0.38 / 0.75 / 1.50 m。mf51 三楼段（405→715 s 共 310 仿真秒）：`goal N failed: state=4` **4 次**（470.184 / 558.886 / 569.684 / 650.432），单次 10.7–17.6 s；`Rotation cmd in collision` 3 段合计 10.3 s；`bounded backout` 3 次各约 1.4 s。**粗计约 60 仿真秒 ≈ 该段 19%** | 既烧时间又损覆盖（经 C4 路径连累整个房间）。可能方向（均未验证，勿直接调参）：① goal 推进途中随地图更新重新验证目标可达性，早放弃而非走到贴住再超时 ② frontier 的 unknown 邻域「厚度」检查，太薄的判为障碍内部 ③ 这类失败确实是真不可达，拉黑是对的，问题在 `failed_radius: 0.75` 连累了旁边合法候选 | 
| **C7** | demo 视频交付：需第一人称 + 第三人称 + 未探索阴影三部分（罗智阳 8/6 验收要求） | **已接入待验收** | 核对结论：**bag 无法还原**——`/real_sense/rgb/image_raw` 未录（只录 camera_info）、`/a1/third_person/image_raw` 我们的 robot.xacro 里根本不存在；只有 `/a1/floor_mapping/map` 可还原。mf53 实测三路源：第三视角 10.4 Hz ✓、第一人称 19.2 Hz ✓、平面图 ✓，产出 1.79 MB / 170 帧 mp4 | 已接入学姐 cty 的 `mission_video_recorder`：xacro 第三视角相机（质量 0.001 kg，包在 `<xacro:if ENABLE_FRONT_CAMERA>` 内，默认 false 时展开 0 处，正式比赛不受影响）、recorder 脚本+launch、CMakeLists 注册、harness 5 处改动（含 **SIGINT 优雅收尾**，否则 mp4 损坏）。**遗留**：①第一人称走 fallback 是无检测框裸 RGB，要带框需开 `publish_debug_images` ②`ENABLE_FRONT_CAMERA` 捆着官方 800×800@30Hz 前视相机，实测 RTF 0.186→0.165（**−11%**），拆分需改含 auto.sh 在内的三个文件 |
| **C3** | `floor_mapping` 把房间内**家具之间的开口**识别成门（操作员 RViz 目视） | **已观测，目前无影响** | 至今全程仅 6 个 branch：(5,±1)(6,±1)(14,±1)，全部对应真实门站；采纳门宽 1.14–1.33 m；二楼已正常判定 4/4；station 6 两房间无 `bounded along the corridor` 日志即无门参与切分。被 `doorways_callback` 的 `abs(lateral) > 2.2` 挡掉（房间连通域 lateral 达 9.50 m） | **残余风险真实**：若某开口 lateral < 2.2 且宽 1.0–1.6 m 将全部通过 → ①`room_axis_bounds()` 拿它当相邻门站切连通域 ②造假 branch ③污染 `completed_room_count: 4` 配额。本轮结束后从 bag 拉全部 doorways 的 width/lateral/longitudinal 核验有无擦边通过 |

---

## D. 已知，按优先级未排期

| # | 问题 | 状态 | 说明 |
|---|---|---|---|
| **D1** | 跨层世界坐标失效，二层实测差 x 19.2 / y 22.3 / z 2.98 m | **已知未排期** | 识别 14 分 + 虚警 8 分在多层场景拿不到。完整方案见 `OPTIMIZATION_BACKLOG_坐标系与返航.md` §0–§4（电梯轿厢做跨 generation 不变量重算 T_g）。用户定的优先级第 3 位 |
| **D0** | **`detected_danger_sources` 为空**：12 个房间全部走完，提交文件一个危险源都没有 | **已观测未定位** ⚠️ **最高价值** | mf61 `detected_danger.json`：`{"exploration_time": 891.21, "detected_danger_sources": []}`。按官方脚本 `detected_count==0` → 虚警率 0 分；`correct/truth_count=0` → 识别概率 0 分，**客观 37 分里的 22 分直接归零** | 候选原因未区分：①danger_perception 未检出 ②检出但 result_manager 未收下（帧/置信度/generation 门槛）③跨层坐标 D1 导致被扣下 ④相机覆盖 D2 关闭导致没走到能看见的位置。**必须先分清是哪一个**，不要直接调参 |
| **D2** | `room/camera_coverage/enabled: false`，相机覆盖兜底从不运行 | **已知未排期** | 后果：「房间里没 frontier 了」直接等于「房间探完了」，无相机覆盖检查。代码注释记载 run18 因此漏掉一个源（在 FOV 里 13 次但从未近于 5.55 m），run14 走到 1.49 m 才找到。识别概率 ≤0.6 直接 0 分，与 14 分强相关。**未动**，因为关它可能有我不知道的原因 |
| **D3** | 返航模块未实现 | **已门控关闭**（见 A5） | 依赖 D1 的 T_g |
| **D4** | RTF 只有约 0.19–0.24，走路慢 | **部分缓解** | 已做：清掉自己容器 151% 残留、关掉无人订阅的点云转换省 17–20%、跳过一楼省 44% 仿真时间。**阻塞**：学姐 `simenv-cty-full` 占 260–350%，按规矩不动，需协调。根本原因是 Gazebo 物理主循环单线程（500 Hz × 40 ODE 迭代） |

---

## E. 工具与流程问题

| # | 问题 | 状态 | 说明 |
|---|---|---|---|
| **E1** | `analyze_mf49_bag.py` 的 `analyze_fall()` 命令循环里有 `return`，**它的逐帧时间线从未运行过** | **已发现，未修** | 交接引用的数字方向正确（已独立复算验证），但该脚本不能直接再用 |
| **E2** | `mission.log` 和 `status.log` 都是块缓冲（`rostopic echo` 重定向也是），实时观测滞后数分钟 | **已知** | 实时判断要用 `rostopic echo -n1` 或 `pose.csv`（行缓冲）。完整事件要等进程退出 |
| **E3** | harness 的 cleanup 没杀掉 `auto.sh` 的孙进程 gzserver | **已知未修** | mf50 结束后 gzserver 仍占 **114% CPU**，加 junior_ctrl、rviz。已用 `docker restart` 清场 |
| **E4** | 容器 PID 1 是 `sleep`，不回收子进程 → 僵尸堆积 | **已知** | 交接 §6.2 已记载「用 `docker restart` 清场」 |
| **E5** | 我挂的状态监控 grep 写错（`status.log` 引号是转义的 `\"state\"`），一直静默而我当成「无新状态」 | **已修正** | 教训：监控的静默不等于成功，必须覆盖失败信号 |
| **E6** | 交接文档记「第三个房间 station 15 left 三个 goal」，实际是 station 15 **right** 三个、left 两个 | **已订正** | 不影响结论 |

---

## F. 已排除的假设（保留，避免重走）

| # | 假设 | 结论 | 推翻依据 |
|---|---|---|---|
| **F1** | 房间事务期间 OccupancyGrid 停更 | **已排除** | mf51 直接观测：连通域 7110→9403→…，`map_seq` 44→52→66 持续递增 |
| **F2** | 绿线不动 = 内部候选没更新 | **已排除** | 是发布路径问题（A4），绿线与内部状态本来就无关 |
| **F3** | mf50 的失败是我这轮改动造成的 | **已排除** | 机器人还在一层，返程代码一行未执行；房间诊断只加日志 |
| **F4** | `wz = 1.8` 的原地转向是估计器脱节的充分条件 | **已排除** | mf50 另 4 次 1.8 转向脱节仅 0.005–0.020 m；mf49 四次 ≤0.037 m。167.64 那次 0.113 m 是离群值 |
| **F5** | lobby rule 是房间灰区的**充分**解释 | **已排除**（仍是必要条件，见 B1/C1） | (5,−1) 同样被 7.00 切但表现尚可 |
| **F6** | RViz 里看到的门 = 探索器采纳的门 | **已排除** | 探索器 `doorways_callback` 另有五道过滤；RViz 画的是 `floor_mapping` 原始输出（C3） |

---

## G. 轮次记录

| 轮次 | 结果 | 卡点 | 关联条目 |
|---|---|---|---|
| mf49 | ✗ MISSION_FAILED | 二楼 C→B 起步坠楼（真值 z 2.915→0.057） | A1 A2 B2 C1 C2 |
| mf50 | ✗ MISSION_FAILED | **一楼** station 10 right 释放后，FAST-LIO 静止漂移 0.148 m → LOST → floor_mapping 降级 → 3.5 s 宽限用尽 | B3 |
| mf52 | ⊘ 主动中止 | 未接视频模块，跑完无法交付 demo；已留 `outcome.txt=ABORTED_FOR_VIDEO_MODULE` + `aborted_reason.txt` | C7 |
| mf53 | ✗ MISSION_FAILED | sim 21.3 s 死于 `entry remained explicitly occupied`（wall_factor 兜底提前触发，RTF 0.165）。**但视频链路验证成功**：三路源齐全、mp4 正常收尾 | A7 C7 |
| **mf61** | ✅ **MISSION_COMPLETE** | **完整多层 demo 首次跑通**：一楼 4 房间(231.9) → 换层(301.8) → 二楼 4 房间(549.6) + 返程(587.3) → 换层(604.6) → 三楼 4 房间(851.7) + 返程(892.8) → 回一楼(903.6)。进轿厢余量二楼 +0.034 / 三楼 +0.044，A11 生效。视频 75 MB | A1–A11 |
| mf60 | ⊘ 门禁拦截 | 门禁文件写入失败（shell 变量未展开），harness 正确拒绝启动；目录被容器 root 文件污染，改用 mf61 | — |
| mf59 | ✗ 差 0.02 m | **三层全探索 + 录像**，RTF 0.272，视频进度条分母 1200 s | A6–A10 C7 |
| mf58 | ⊘ 主动中止 | 操作员要求调高视频进度条分母；已测得 RTF 0.266（学姐容器空闲） | C7 |
| mf57 | ⊘ 主动中止 | 操作员要求改为一楼也完整探索 | — |
| mf56 | ✗ MISSION_FAILED，**走到三楼探索** | 一楼 ingress → 二楼 **4 房间 + 返程全通过**（A1/A2/A6 验证）→ 换层三楼 → 三楼探索 209 仿真秒后 action_timeout 墙钟超时。视频 51.7 MB | A10 |
| mf55 | ✗ MISSION_FAILED | 一楼 `entry remained explicitly occupied`（wall fallback 已是 40 s，证明 A7 生效；这次真等满 4 仿真秒） | A9 |
| mf54 | ✗ MISSION_FAILED | **通过一楼**，死于二楼 `active opening scan 339.9/360.0 deg` | A8 |
| mf51 | ✗ MISSION_FAILED，但**走到历史最远** | 一楼跳过探索（省 244 仿真秒）→ 换层 → 二楼 **4/4 房间 + 返程成功**（A1/A2 首次通过 mf49 的死亡段）→ 换层三楼 → **三楼探索完成**（FLOOR_COMPLETE floor=2, 715.582）→ 三楼 endpoint→C 死于 **actionlib 竞态**（B4），非 A1/A2 问题 | A1 A2 A4 B1 B4 C1 C3 C4 C5 C6 |
