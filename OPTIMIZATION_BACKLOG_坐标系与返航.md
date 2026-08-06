# 优化待办：世界坐标系与返航

记录时间：2026-08-05
记录人：Claude（本文所有数字均为实测，推断部分已显式标注）

这三件事表面上是三个问题，实际是**同一个根因**的三个面：我们的世界坐标系只在
第 0 代（出生点那一代）成立，换层重锚之后就失效了。

---

## 0. 根因：世界锚点每代重建，且假定"此刻机器人在出生点"

`src/a1/localization/src/localization_pose_adapter.cpp:473-479`：

```cpp
if (world_alignment_enabled_ && !world_anchor_established_)
{
    world_to_odom_ = initial_world_to_base_ * odom_to_base.inverse();
    world_anchor_established_ = true;
}
```

`initial_world_to_base_` 来自 `config/frames.yaml`，是**写死的出生点位姿**
`(0.0, -3.2, z, yaw 1.5708)`。配置注释自己写明了前提：

> "A generation must only start while the robot is at this pose."

而 `world_anchor_established_` 会在 `reinitialization_required` 时被清零
（同文件 line 371），多层每次换层都会重新定位 —— **那一刻机器人在电梯轿厢里，
不在出生点**。于是新一代的"世界系"被钉在了错误的位置上。

### 实测证据（mf46，sim 448 s，机器人在二层走廊）

```
真值 /Odometry_gazebo      x =  6.93   y = 15.77   z =  2.918
我们声称 /a1/localization/odom  x = -12.23  y = -6.50   z = -0.064
差                          19.2 m      22.3 m      2.98 m
```

另有 mf41 全程统计：`/a1/localization/odom` 与裁判位姿**中位差 19.15 m**，
定位状态里 `world_anchor_established: false`。

### 影响

| 楼层 | 世界坐标可用性 | 后果 |
|---|---|---|
| 一层（第 0 代） | 成立 | 单层轮次能记分：run20 correct=2、run23 correct=1 |
| 二/三层 | **整体失效** | 检测点差 20+ m，必然 0 匹配；返航目标也无法用世界坐标表达 |

**这意味着识别概率 14 分 + 虚警率 8 分（共 22 分）在多层场景下目前拿不到**，
与机器人实际有没有看到危险源无关。

---

## 1. 用出生点变换求危险源 world 坐标

### 已建好的部分

`src/a1/result_manager/src/a1_result_manager/scoring.py` 里的 `WorldAnchor`：
把"机器人站在出生点那一刻的 map 位姿"与 `team_scene_info.json` 的 `robot_start`
配对，得到刚体变换。烟雾测试实测**零误差**还原（喂 map 点 → 吐出的 world 坐标
与真值球逐位相同，误差 0.000000）。

锚点**按 localization generation 分别持有**，拿不到锚点的检测点在 `apply` 模式下
会被扣下不写 —— 错帧坐标同时算漏报和虚警，扣两次，宁可不报。

### 为什么现在是 `audit` 模式而不是 `apply`

`a1_localization` 已经自带同款世界对齐（用的就是 `robot_start` 那组数），单层记过分
的轮次走的就是那条路。**在它之上再叠一次变换会把已经对的坐标搬歪。**
所以 `world_anchor_mode` 默认 `audit`：照算锚点、记进 audit 文件，但**不改动提交的
坐标**。

### 待办

一旦第 3 节的每代重锚做好，`WorldAnchor` 就有了上层的锚点来源，届时：

- 把 `world_anchor_mode` 翻到 `apply`
- 上层检测点从"必然落空"变成"可匹配"

**这套机器已经写好并测过，缺的只是上层的锚点输入。**

---

## 2. 返航到出发点

### 官方要求

比赛 PDF 的探索时间定义是「完成全覆盖探索**并返回出发点**所需的时间」。
但官方发布的文件里**没有任何一处检查返航位置**：`robot_start` 只被写出、从不被
消费，也没定义任何容差。官方保留了 `referee_only` 真值通道
（`/Odometry_gazebo` + `/ground_truth/*`，`ENABLE_REFEREE_ODOM` 默认 1），
判定流程很可能在包外，所以自报的数要经得起事后核对。

### 现状（有问题）

`multifloor_mission_node.py` 的 `run()` 尾部，从电梯几何 + **写死的 2.3 m / 3.5 m**
偏移重建一个"大概是起点"的位置：

```python
entrance_inside = ... lobby + 2.3 * (fx, fy) ...
final          = ... entrance_inside - 3.5 * (fx, fy) ...
```

这段代码**从来没有真正执行过**（多层链路还没跑到过那里）。
`MISSION_COMPLETE` 现在会带上残差 `return_residual_m`，但那只证明"到了重建点"，
不证明"到了真起点"。

### 正确解法（队友提出的思路，方向正确）

直接把 `robot_start` 真值喂给导航是不行的 —— `robot_start` 是 world 系，
move_base 只认 map 系，而回到一层时 world↔map 变换正是缺的那一块。
但思路本身指出了正解，链条是：

1. **出发时（第 0 代）**：机器人站在 `robot_start` 上 → 得到第 0 代的
   world↔map 变换 **T₀**（这一步已实现）
2. **仍在第 0 代、进一楼电梯时**：靠轿厢四壁测量轿厢位置（不是靠机器人自己站哪），
   用 T₀ 换算成**轿厢的 world 坐标 C**
3. **任意后续代 g**：机器人又站在**同一个物理轿厢**里，再测一次轿厢四壁得到它在
   第 g 代 map 系的位置，与 C 配对 → **反解出 T_g**
   - 电梯井不动，轿厢 world XY 逐层不变 —— 这是跨代的物理不变量
   - 朝向依据已验证事实：电梯传送保持机身 yaw（mf17 实测 0.3°）
4. **有了 T_g**：把 `robot_start` 换算进第 g 代 map 系，直接导航过去，
   **替掉那两个硬编码数字**
5. **同一个 T_g 顺带解决第 1 节的上层锚定问题**

**注意**：这不能"消除漂移"。`robot_start` 是真值，但中间每一步（第 0 代从起点走到
电梯的漂移、每次到达时测轿厢的误差）都会进到 T_g 里。它做到的是**每次换代重新
校准一次**，把误差从单调累积变成有界。

### 需要的前置能力

- 轿厢的**帧级**定位（同一个物理点在两代里都能测到），不只是"机器人停在哪"
- 朝向消歧：轿厢近似矩形，靠开口方向区分（mission 已有 `arrival_exit_yaws`）

---

## 3. z 锚：`0.6` vs `0.30`（两棵树目前不一致）

| 树 | `initial_world_to_base_translation` |
|---|---|
| `integration_20260730`（单层，记过分） | `[0.0, -3.2, 0.30]` |
| `xjc_multifloor_20260803`（我们的多层） | `[0.0, -3.2, 0.30]` ← 已同步 |
| `xjc_fix_frontier_mapping_20260805`（fix 树） | `[0.0, -3.2, 0.6]` ← **已回退，见下** |

单层树的注释写明理由：机器人生成时 z=0.6 然后落地，定位初始化时实际站高
**实测 0.289 m**（跨轮 0.289–0.313）。用 0.6 会让所有 world 输出**高 0.366 m**，
`competition run05` 有一个源因此差 **37 mm** 掉出 1.0 m 阈值。

### 为什么在 fix 树回退了

mf45（z=0.30）在 sim 23 s 失败：`entry remained explicitly occupied for 4.0 sim s`。
mf46（z=0.6，其余完全相同）顺利通过同一位置并一路上到二层。
**这是一组干净的 A/B，唯一变量就是 z 锚**，但每边只有 n=1。

**推断（未证明）**：与 fix 树本轮的 floor_mapping 改动交互 ——
`map_ground_clearing_max_height: 0.05` 要求"距估计地面 5 cm 内的回波"才能清除
已占据格，`occupied_clear_confirmations: 3` 要求 3 次确认；z 参考下移 0.30 m 后
近地面回波可能不再落进那个 5 cm 窗口，占据格就永远清不掉。
**这个机制需要和 floor_mapping 的改动一起验证，不能单独搬。**

### 优先级说明

z 锚只值 **0.31 m 的精度，且只在一层有意义**。二层上偏差是 2.98 m（且 x/y 各差
二十米），常数修正救不了。**如果只能做一件事，做第 3 节的每代重锚，不是这个。**

---

## 4. 建议的执行顺序

1. **每代重锚（T_g，靠电梯轿厢）** —— 一次解决上层坐标（22 分）、返航、z 三件事
2. `world_anchor_mode` 翻到 `apply`，上层检测点开始可匹配
3. 用 T_g 替掉 `run()` 尾部的 2.3 / 3.5 m 硬编码
4. z 锚在 fix 树的正确处理（需与 floor_mapping 改动一起评估）

---

## 附：相关文件索引

| 内容 | 位置 |
|---|---|
| 世界锚点建立逻辑 | `src/a1/localization/src/localization_pose_adapter.cpp:473` |
| 锚点清零 | 同上 line 371 |
| z 锚配置 | `src/a1/localization/config/frames.yaml` |
| WorldAnchor 变换 | `src/a1/result_manager/src/a1_result_manager/scoring.py` |
| 模式开关 | `src/a1/result_manager/config/result_manager.yaml` → `world_anchor_mode` |
| 返航硬编码 | `src/a1/mission_manager/scripts/multifloor_mission_node.py` → `run()` 尾部 |
| 轿厢返航点 | 同上 `elevator_return_points` / `arrival_exit_yaws` |
| 官方评分脚本 | `SimEnv/src/building_obstacles/scripts/evaulate_danger.py`（无 ROS 依赖） |
| 允许读取的场景文件 | `generated_building/team_scene_info.json` |

---

# 追加：现场观察到的问题（2026-08-05 mf48 轮，操作者目视 + RViz）

## 5. 房间事务提前结束：房间里还有**雷达根本没扫到**的区域就出去了

**现象**（操作者在 RViz 前目视 + 截图，mf48）：一层最后一个房间、二层第一个房间
都出现"房间事务判定结束并离开，但房间里还有明显未探索区域"。**有的房间没问题**，
所以不是全局失效，是条件性的。以前没有这个现象。

**关键订正（依据操作者截图）**：那些未探索区域是**障碍物背后的灰色遮挡阴影**，
也就是栅格里仍为 **UNKNOWN** 的格子 —— 不是"雷达扫过、相机没看"，是**雷达从未
扫到**。机器人从门口/房间一侧观察时，家具挡住了后方，形成扇形阴影；它没有绕到
能看见那片区域的位置，就宣布房间探完了。

**这为什么是 bug**：UNKNOWN 区域紧邻 known-free，按定义**就该产生 frontier**。
既然还有 frontier 却判定"无可达 frontier"，只有两种可能：

1. frontier **没被生成** —— ROI 掩膜、房间连通域(`room_free_component_mask`)或
   栅格边界把那片区域排除在外了
2. frontier **生成了但被否决** —— 被 `frontier_is_admissible` / 最低分阈值 /
   `failed_goals` 黑名单 / 队友本轮新增的 `room_source_keepout` 门口保留区拦掉了

**排查起点**（尚未做，按此顺序）：
- 从 bag 里取房间退出瞬间的 `/a1/floor_mapping/map`，**直接统计**该房间连通域内
  UNKNOWN 格子数与位置，确认阴影区确实还在（而不是目视错觉）
- 打开探索器的 frontier 候选日志：是"一个都没生成"还是"生成了被 admissible 否决"。
  这一步直接把上面两种可能二选一，**不要跳过**
- 若是被否决：逐个查 `room_frontier_minimum_score`、`minimum_frontier_score`、
  `failed_radius/maximum_failures` 黑名单、`room_source_keepout`（fix 树本轮新增，
  depth 1.70 / half_width 1.20，需确认它有没有连带挡掉房间内部的 frontier）
- 若是没生成：查 `room_free_component_mask` 的连通域是否把阴影区切在域外、
  以及 ROI 多边形边界

**注意**：这是操作者目视 + 截图报告，尚未从工件复核。**定位前不要下结论**，
尤其不要照搬"相机覆盖"那套解释——截图证据指向的是 UNKNOWN 遮挡阴影，不是覆盖问题。

## 6. 走路太慢，空走廊里也慢

**现象**：即使在完全没有障碍的走廊里，狗的行进速度也明显偏低。

**为什么值得单列**：探索时间 15 分直接和这个挂钩，而且慢会放大其它一切问题
（每个导航段更接近超时、整轮更容易撞上 600 s 满分线）。

**已知的相关事实**：
- `dwa_local_planner_params.yaml`: `max_vel_x: 1.40`
- mf46 实测 `/cmd_vel` 峰值 |vx| 到过 1.28，说明**指令端并没有被限死**
- 但 mf46 全程 966 仿真秒只走完一层多一点；每个房间约 135 仿真秒
- 所以瓶颈更可能在"实际跟踪速度"或"频繁的加减速/重规划"，而不是速度上限配置

**排查起点**（尚未做）：
- 从 bag 里统计 `/cmd_vel` 指令速度 vs `/Odometry_gazebo` 实际速度的比值，
  区分"没敢下指令"和"下了跟不上"
- 统计 `Got new plan` 的频率——如果重规划过密，机器人会一直在加减速
- `sim_time`、`vx_samples`、`path_distance_bias` 等 DWA 参数是否让它过于保守

## 7.（已定位，正在修）STAY_ON_FLOOR 之后缺 endpoint→C 一段

mf48 死于 `ExploreFloor reported return success but finished 21.22 m from entry
point C`。原分工写在 `return_upper_floor_over_traversed_segments` 的 docstring 里：
探索器负责 endpoint→C，mission 只负责 C→B→A。上层改用 `STAY_ON_FLOOR` 后探索器不再
做 endpoint→C，而 mission 没接上，于是校验必然失败。修法：mission 自己 `navigate(C)`，
并把那条"探索器应已回到 C"的校验改为在实际产生该后置条件的地方检查。

> 操作者看到的"二层第四个房间门口呆住不动"就是这个失败的表象：mission 进程在
> 21:24:12 退出，`/cmd_vel` 无消息，gzserver 和 RViz 还开着，所以画面是活的但机器人不动。
