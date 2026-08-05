# a1_result_manager

产出官方评分文件 `results/detected_danger.json`,并在旁边写一份可追溯的
`detected_danger.audit.json`。

## 为什么这些逻辑必须在我们这边

官方评分脚本 `src/building_obstacles/scripts/evaulate_danger.py` 只 import 了
`argparse/json/math/pathlib/sys/numpy` —— **没有 rospy**。它不具备观测仿真的能力,
读的就是我们写出去的两个字段。所以下面两件事没有任何外部校验兜底:

| 字段 | 官方怎么处理 | 因此谁负责 |
|---|---|---|
| `exploration_time` | 只检查字段存在,然后 `if <= 600` | 本节点 |
| `detected_danger_sources[].position` | 按 world 系三维欧氏距离匹配 | 本节点 |

## exploration_time 的定义

官方 docs 只写了"探索耗时,单位秒",没定义起止,也没说时钟。定义取自比赛 PDF:

> 机器人在规定区域内完成全覆盖探索并**返回出发点**所需的平均时间

因此:

- **时钟 = 仿真时间**(`rospy.Time.now()`,`/use_sim_time=true`)。墙钟会让成绩取决于
  评测机负载:mf41 用了 569 s 墙钟才推进 157 s 仿真(RTF 0.276)。
- **起点 = `MISSION_TIMING_START`**,由 mission 在第一次自主运动之前发出。之前的
  控制器/定位握手不算 —— auto.sh 单是那段就允许 240 s。
- **终点 = `MISSION_COMPLETE`**,并**锁死**。之前的实现是"进程被杀时恰好写下的那个
  数",任务成功和被 kill 写出来的是同一种东西。
- 任务失败(`MISSION_FAILED`)会冻结数值并把 `clock.valid` 置 false —— 按 PDF 定义,
  没走完全覆盖+返航的那一轮根本没有合法的探索耗时。
- **不跨轮累加**。旧实现会把上一轮的 `exploration_time` 读回来当起始偏移。

## 坐标系

官方要求 world 系,`a1_danger_perception` 的 `target_frame` 是 `map`,而 TF 里
**根本没有 world 系**。唯一合法的桥是 `generated_building/team_scene_info.json`
的 `robot_start`(docs/competition-rules.md 明确允许读取),在机器人正站在起点上的
那一刻和它的 map 位姿配对,得到刚体变换。

FAST-LIO 每次定位重初始化都会重锚,所以变换**按 localization generation 分别持有**。
拿不到锚点的 generation 里的检测点会被**扣下不写**,而不是用错误坐标写出去 ——
错帧坐标不是"差一点",它同时算漏报和虚警,扣两次。扣下的数量和 generation 记在
audit 文件里。

> 当前只有 generation 0(一层出发那一刻)能建立锚点。二/三层的检测点会被扣下。
> 补齐上层锚点需要一个跨 generation 的物理不变量(电梯轿厢 world XY 逐层不变),
> 这部分尚未实现。

## 事件契约

mission 侧 `emit()` 会在**每一条**状态里带上 `sim_time` 和 `mission_generation`。
本节点额外消费:

| 状态 | 需要携带 |
|---|---|
| `MISSION_TIMING_START` | `anchor_x/y/z/yaw`(map 系起点位姿) |
| `MISSION_COMPLETE` | `return_residual_m`、`return_tolerance_m`、`final_x/y`、`target_x/y` |

## 测试

```bash
cd src/a1/result_manager/test && python3 -m unittest test_scoring -v   # 19 项纯函数
python3 /tmp/smoke_scoring.py                                          # 联机烟雾(需 roscore+/clock)
```
