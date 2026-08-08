# mf82 交付材料索引（给技术报告用）

跑通那一版 = **mf82_lobby_handover**，2026-08-08 凌晨完成。
这是目前唯一一轮同时做到「三层全覆盖 + 危险源识别非零 + 零坠落 + 任务正常结束」的运行。

---

## 一、代码在哪

⚠️ **不要直接用服务器工作树** —— 它已经被后续实验改动了（mf84/mf85 的改动还未提交，
`git status` 有 5 个已修改文件）。mf82 的确切代码只在提交里。

|          |                                                              |
| -------- | ------------------------------------------------------------ |
| 远端分支 | `mf82-multifloor-recognition-20260808`                       |
| 提交     | `61671a1b3a5fa482713b975253c490f52c30a52d`                   |
| 仓库     | `git@github.com:BUAA-yangyang/tmp2.git`                      |
| GitHub   | https://github.com/BUAA-yangyang/tmp2/tree/mf82-multifloor-recognition-20260808 |

取出确切代码：

```bash
git clone git@github.com:BUAA-yangyang/tmp2.git
cd tmp2
git checkout 61671a1
```

服务器工作树（仅供参考，状态已变）：
`xiaoyi-dev@10.139.197.230:/home/xiaoyi-dev/simenv/xjc_fix_frontier_mapping_20260805`
容器 `simenv-xjc-fix` → `/workspace/SimEnv`

---

## 二、运行产物在哪

容器内：`/workspace/SimEnv/results/competition/mf82_lobby_handover/`

| 文件                              | 大小            | 用途                                               |
| --------------------------------- | --------------- | -------------------------------------------------- |
| `detected_danger.json`            | 330 B           | **提交给评分脚本的结果文件**                       |
| `detected_danger.audit.json`      | 2.1 KB          | 每个检测的置信度、观测数、首末见时刻、坐标变换来源 |
| `mission.log`                     | 559 KB          | 全部节点日志（含逐层 FLOOR_DONE 记录）             |
| `status.log`                      | 38 KB           | 任务状态事件流（JSON，含每个事件的结构化字段）     |
| `sim.log`                         | 2 KB            | Gazebo / 控制器日志                                |
| `mission_video.mp4`               | 98 MB           | 全程双视角录像（已下载到项目文件夹）               |
| `run_0.bag` + `run_1.bag`         | 4.3 GB + 1.9 GB | 完整 rosbag（含真值 `/Odometry_gazebo`）           |
| `source.diff`                     | 235 KB          | **该轮实际运行的代码全文改动**                     |
| `head.txt`                        | —               | 该轮基于的提交                                     |
| `runtime_exploration_params.yaml` | 5.6 KB          | **探索模块运行时生效参数**（不是配置文件内容）     |
| `runtime_multifloor_params.yaml`  | 4.2 KB          | 任务模块运行时生效参数                             |

本地已有：

- `/Users/max/Documents/挑战杯/mf82_lobby_handover_video.mp4`
- `/Users/max/Documents/挑战杯/AUDIT_20260808_独立审查报告.md`

---

## 三、成绩（官方脚本实算，可复现）

```bash
python3 ./src/building_obstacles/scripts/evaulate_danger.py \
  --truth-file ./generated_building/danger_truth.json \
  --detected-file ./results/competition/mf82_lobby_handover/detected_danger.json \
  --output-file /tmp/eval.json --verbose
```

```
真值危险源数量: 4      选手检测数量: 3      探索时间: 1167.01 秒
正确识别数: 3          漏报数: 1            虚警数: 0

探索时间得分:        6.00/15
危险源识别概率得分: 10.50/14      (prob = 3/4 = 0.75)
危险源虚警率得分:    8.00/8       (far = 0)
技术实现客观部分总分: 24.50/37
```

三个命中的定位误差（真值 ↔ 检测）：

```
(-5.418, 34.421, 5.35) ↔ (-5.451, 34.341, 5.258)   0.126 m
(-4.778, 32.371, 5.35) ↔ (-4.812, 32.294, 5.247)   0.133 m
(-6.425, 22.660, 2.75) ↔ (-6.519, 22.542, 2.713)   0.155 m
漏掉 (-8.895, 24.123, 5.35)
```

场景 seed：**382835531**（3 层 × 4 房间，4 个危险源、4 个干扰源）

---

## 四、逐层数据（可直接引用）

```
一楼  rooms 3/4   coverage 69.9%   FLOOR_DONE @ sim 445.3
二楼  rooms 4/4   coverage 61.9%   FLOOR_DONE @ sim 779.5
三楼  rooms 4/4   coverage 64.3%   FLOOR_DONE @ sim 1120.7
房间事务合计 12（三层各 4 间全部完成 door → interior → 360° scan → exit）
三次换层：floor 0 → 1 → 2 → 0
全程坠落 0 次、定位误杀 0 次
任务结束 MISSION_COMPLETE @ sim 1179.7
```

一楼 `rooms 3/4` 的说明：四个房间**都进去了**，其中一间因预算耗尽退出、被标为
`unproven` 并从配额中撤销（这是设计行为，防止用未探完的房间关闭楼层）。
录像里能看到四个房间都有轨迹。

---

## 五、写报告时必须如实写明的两点

**1. 未包含返航。** `mission/final_return_to_start` 为 false。任务结束时机器人停在
电梯轿厢内，距出发点约 6 m。日志原文：

> `three floors explored and the car returned to floor zero; return-to-start is
> disabled so exploration_time does NOT include a return leg and is not
> comparable to the PDF definition`

所以 1167.01 s 这个探索时间**不符合 PDF 的定义**（PDF 要求"探索全部可通行区域后
返回起点"）。报告里引用时应注明。

**2. 完成率。** 同一份代码的重复性：

```
mf82  MISSION_COMPLETE   12/12 房间  识别 3/4
mf83  MISSION_FAILED     12/12 房间  识别 4/4   （返程对准差 0.2°）
```

探索与识别可重复（连续两轮 12/12 房间），但整轮完成受返程/换层环节影响。
mf83 的识别达到 4/4（满分 14/14），但因任务中止，其探索时间在审计文件里被标为
`clock.state = ABORTED, valid = false`，不可直接引用。

---

## 六、已定位但未解决的问题（若报告需要"后续工作"一节）

详见 `AUDIT_20260808_独立审查报告.md` §9。三条最主要的：

1. **门厅楼板空洞**：每层门厅由三条互不相连的板拼成，中间两条缝是贯穿 7.84 m 的
   开放空洞（楼梯井、电梯井侧）。井口在 2-D 占据栅格里是 `free(0)` 而非 unknown，
   costmap / 路径规划 / 已知自由检查全部不反对走进去。已记录 6 次坠落，全部落在
   门厅段（y < 7.84），经由 4 条不同的代码路径。
2. **相机覆盖判据未启用**：楼层"探完"由激光地图无未知区域判定，而 Livox 360°/40 m
   从房间任一点即可扫完整间房 ⇒ 机器人身体可能从未接近危险源。mf82 漏掉的那个源，
   感知实际生成过候选但未确认（最近仅到 5.31 m，确认上限 5.5 m）。
   配置里 `room/camera_coverage` 已实现但 `enabled: false`。
3. **房间内楔死无脱困**：`allow_backout=False` 在房间目标上禁用了倒退重试，
   而房间是唯一有家具的地方。mf82 一楼实测：机器人卡在边几前 0.25 m 处 64 秒零位移。

---

## 七、其他可引用的运行

| 轮次                     | 用途                                                   |
| ------------------------ | ------------------------------------------------------ |
| `mf83_repeat_unchanged`  | 同代码复跑，识别 **4/4 满分**，证明识别链路上限        |
| `mf61_aplane_video`      | 历史上首次三层 12/12（但当时感知坐标系有误，识别 0/4） |
| `mf72_completion_window` | 改进前基线，21.00/37，三楼仅探索 1.9 秒                |

产物同在 `/workspace/SimEnv/results/competition/<轮次名>/`。