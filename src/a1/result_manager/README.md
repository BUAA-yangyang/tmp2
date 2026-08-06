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

## 坐标系(默认只观测,不改写)

官方要求 world 系。**`a1_localization` 已经自带定点世界对齐**
(`localization/config/frames.yaml` 的 `world_alignment_enabled` +
`initial_world_to_base_translation`,用的正是 `robot_start` 那组数),单层里记过分的
那几轮(run20 correct=2、run23 correct=1)走的就是这条路。所以本节点**默认不再叠加
一次变换** —— `world_anchor_mode: audit` 只把"独立锚点会算出什么"记进 audit 文件,
提交出去的坐标保持原样。

多层这条路上这个问题是**未决**的:mf41 实测 `/a1/localization/odom` 与裁判位姿
中位差 **19.15 m**,定位状态里 `world_anchor_established: false`。等查清检测点最终
落在哪个系里,再决定是否把模式翻到 `apply`。

`apply` 模式下锚点**按 localization generation 分别持有**(FAST-LIO 每次重锚就失效),
拿不到锚点的检测点会被扣下不写 —— 错帧坐标同时算漏报和虚警,扣两次。

> 已知偏差:多层树 `frames.yaml` 的 z 锚是 `0.6`(Gazebo 生成高度),而单层那棵树已经
> 修成 `0.30` 并写明原因——机器人落地站稳后实测 0.289 m,用 0.6 会让所有 world 输出
> 高 0.366 m,competition run05 有一个源就是因此差 37 mm 掉出 1.0 m 阈值。这个修正
> **尚未回移到多层树**。

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
