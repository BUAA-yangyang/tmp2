# A1 四足机器人 · 多层危险源自主搜索

我们队（xjc）在官方 SimEnv 之上实现的多层探索系统。官方环境说明见根目录
`README.md` 和 `docs/`，本文只讲**我们写的部分**（全部在 `src/a1/`）。

---

## 当前能跑到哪一步

`mf61` = **MISSION_COMPLETE**，完整多层 demo 首次跑通：

```
231.9  一楼探索完成（4 房间）
284.4  进一楼电梯
301.8  换层 → 二楼
549.6  二楼探索完成（4 房间）
587.3  二楼返程进梯
604.6  换层 → 三楼
851.7  三楼探索完成（4 房间）
892.8  三楼返程进梯
903.6  换层回一楼 → MISSION_COMPLETE
```

12 个房间、3 次换层、2 次上层返程，903.6 仿真秒。

**尚未做的**（不是坏了，是有意关闭或还没做）：

| 项 | 状态 | 原因 |
|---|---|---|
| 危险源识别 | 关闭 | 跨层 world 坐标差 20 m、z 锚未定，开了也拿不到分（见台账 D1） |
| 返回出生点 | 关闭 | `run()` 尾部那段是从电梯几何 + 硬编码 2.3/3.5 m 重建的占位实现，从未执行过 |
| `exploration_time` | **不可用于对分** | 因为没有返航段，不符合 PDF 的「完成全覆盖探索**并返回出发点**」定义。`MISSION_COMPLETE` 事件里有 `return_to_start_performed=false` 标注 |

---

## 怎么跑

### 环境

容器 `simenv-xjc-fix`，镜像 `simenv:xz-runtime-snapshot-20260731`
（⚠️ 这个镜像自带 `move_base_msgs`，用 `simenv:noetic-cu128` 新建容器会
`ModuleNotFoundError`）。工作树挂在 `/workspace/SimEnv`。

```bash
source /opt/ros/noetic/setup.bash
cd /workspace/SimEnv
catkin build            # 这棵树用 catkin build，不是 catkin_make
```

### 完整多层轮

固定 seed `382835531`。参考 `/tmp/a1/run_inside_fix_full.sh`，它做了：

```bash
GUI=false ENABLE_POINTCLOUD_CONVERTER=0 ENABLE_FRONT_CAMERA=1 \
  SEED=382835531 bash ./auto.sh          # 起 Gazebo

roslaunch a1_danger_perception danger_perception.launch publish_debug_images:=false
roslaunch a1_navigation_tests mission_video_recorder.launch \
  output:=<run_dir>/mission_video.mp4 mission_timeout_s:=1200.0
rosbag record --lz4 --split --size=4096 -O <run_dir>/run <topics...>

roslaunch a1_mission_manager multifloor_test.launch \
  use_rviz:=false \
  force_floor_complete_after_ingress:=false \
  upper_floor_special_test_mode:=false
```

关键开关：

| 开关 | 作用 |
|---|---|
| `force_floor_complete_after_ingress:=true` | 一楼只走「进门→主走廊」就算完成，直接去电梯（只跳过一楼，二三楼照常完整探索）。调试上层时省 4 分钟仿真 |
| `upper_floor_special_test_mode:=true` | 上层跳过 `explore_floor`，只测电梯链路 |
| `mission/final_return_to_start` | 默认 `false`；真返航做好前不要打开 |
| `ENABLE_POINTCLOUD_CONVERTER=0` | `/livox/Pointcloud2` 我们零处订阅（它那条链路用 Gazebo 真值做变换，比赛不能用），关掉省约 20% CPU |
| `ENABLE_FRONT_CAMERA=1` | 打开第三视角相机（录 demo 用）。⚠️ 它同时打开官方 800×800@30Hz 前视相机，RTF 掉约 11% |

RViz 单独起（`DISPLAY=:1`）：

```bash
rviz -d $(rospack find a1_navigation)/rviz/a1_nav.rviz
```

### 单测

```bash
cd src/a1/mission_manager && python3 -m unittest discover -s test -p "test_*.py"   # 40
cd src/a1/exploration    && PYTHONPATH=$PYTHONPATH:$PWD/src \
                            python3 -m unittest discover -s test -p "test_*.py"   # 146
```

---

## 代码结构（`src/a1/`）

| 包 | 职责 |
|---|---|
| `mission_manager` | 多层总调度：楼层顺序、电梯进出、换层、上层返程。`multifloor_mission_node.py` 是主节点 |
| `exploration` | 单层探索：frontier 提取/选择、房间事务、走廊模型。`frontier_explorer_node.py` + 纯函数库 `src/a1_exploration/frontier.py` |
| `floor_mapping` | 从点云建当层 OccupancyGrid，识别墙/门；地图持久化（占据格不被穿透射线擦除） |
| `localization` | FAST-LIO 适配 + 换层时的重定位/generation 隔离 |
| `navigation` | move_base / DWA / costmap 配置 |
| `danger_perception` | 危险源识别（本轮关闭） |
| `result_manager` | 写 `results/detected_danger.json` + 旁写 audit 溯源文件 |
| `cmd_mux` / `building_behavior` / `navigation_tests` | 速度多路复用与安全锁 / 门电梯服务封装 / 诊断与录像工具 |

### 多层返程的 A / B / C 三个点

上层楼没有可用的建筑坐标，所以返程完全靠**本代实测过的位姿**：

```
A = 出电梯时机器人在轿厢内的位姿（achieved pose，不是门推算的）
B = 出梯走出来、侧向空间打开后的位姿（轿厢外）
C = 沿走廊走进去、用墙面拟合校正过轴向的位姿（探索起点）

返程 = endpoint → C → B → A_safe，逐段反向重走
```

⚠️ **C 的 yaw 是走廊向内方向**（背对电梯）。它同时是 ROI 原点和房间纵横轴，
所以不能直接当导航目标——那会让 MoveBase 在 C 转 180°，下一个 C→B 目标再转
回来。正确做法是深拷贝一份 staging pose，只换 yaw（见
`complete_upper_floor_and_return_to_a`）。

---

## 三条硬约束（踩过的坑）

### 1. 时间判据一律用仿真钟，墙钟只做停摆兜底

这套仿真的 RTF 实测 **0.165–0.272**，从未接近 1.0，而且会随机器上还跑着什么
剧烈波动（mf56 前半段 0.25、三楼段掉到 0.12）。任何用墙钟做主判据的地方，
在慢机器上都会提前掐断本来够用的等待——**这类 bug 一轮死了四次**。

规则：`X_timeout_sim` 是主判据，`X_timeout_wall`（或 `X_wall_factor`）必须
≥ **10 倍**，即容忍 RTF 低到 0.10。`test_wall_clock_backstops.py` 会自动枚举
所有配对并用实测最差 RTF 验算，新增参数漏配会直接失败。

例外：等 ROS 服务响应（`make_plan_retry_delay_wall` 之类）本来就该用墙钟。

### 2. 判据不能比它依赖的系统的保证更严

DWA 的契约是「距目标 `xy_goal_tolerance`(0.45 m) 以内就 latch」，不是「精确
到达」。写后置条件时必须把这个误差算进去，否则健康的导航也会失败——
`upper_transfer_plane_tolerance` 就是这么来的（`inset 0.40 < tolerance 0.45`
⇒ 落点区间 `A + (−0.05 … +0.40)`，零容差的检查在最坏情况下不可满足）。

同理：`/set_door_state` **26 毫秒就返回**，但门是约 25 秒的插值过程，
不要以为服务返回了门就开好了。

### 3. 真值话题只能离线用

`/Odometry_gazebo`、`/ground_truth/*` 是裁判通道，**算法绝对不可订阅**
（`docs/algorithm-interfaces.md`）。只能在 bag 事后分析里用来验证。
唯一允许读的场景文件是 `generated_building/team_scene_info.json`。

---

## 排障

| 现象 | 原因 |
|---|---|
| `mission.log` / `status.log` 看不到最新事件 | 都是**块缓冲**，进程退出才刷。实时看用 `rostopic echo -n1` |
| 报错指向 `devel/lib/<pkg>/xxx.py` 像是构建过期 | catkin relay stub 打碎 sibling import。脚本顶部要 `sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))`，或把 helper 移进真 Python 包 |
| 一轮结束后 CPU 仍占 100%+ | harness 的 cleanup 杀不掉 `auto.sh` 的孙进程 gzserver；容器 PID 1 是 `sleep` 不回收僵尸。用 `docker restart <容器>` 清场 |
| RViz 绿色 frontier 不动 | 房间事务内外发布的是**两批不同的候选**（房间连通域 vs 全楼层）。这是设计，不是卡住 |
| 机器人在电梯厅附近莫名坠落 | 开放竖井的井口格是 **UNKNOWN 不是 occupied**，costmap 不反对走进去。实测横向余量只有约 0.5 m（台账 B2） |

---

## 已知未解决

全部记在 **`ISSUE_LEDGER_问题台账.md`**（36 条，含 6 条被推翻的假设——保留是为了
不重走死路）。按影响排：

1. **D1 跨层 world 坐标失效** — 二层实测差 x 19.2 / y 22.3 / z 2.98 m。
   识别 14 分 + 虚警 8 分共 22 分在多层场景拿不到。方案见
   `OPTIMIZATION_BACKLOG_坐标系与返航.md` §0–§4（用电梯轿厢作跨 generation
   不变量重算 T_g）
2. **B2 开放竖井对 2-D 栅格不可见** — 已坠落两次
3. **B1 / C1 / C4 / C6 房间探索质量** — 房间事务提前结束，有两条独立路径，
   已加逐级计数日志（`ROOM_FRONTIER_PIPELINE` / `ROOM_FRONTIER_SELECTION`）
   但尚未归因
4. **B3 FAST-LIO 快速转向后的静止漂移** — 触发 `STATIONARY_TRANSLATION_DRIFT`
   整轮失败，偶发
