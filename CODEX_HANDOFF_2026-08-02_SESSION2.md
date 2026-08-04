# SimEnv 多层自主探索与电梯任务交接（2026-08-02 第二阶段）

更新时间：2026-08-02（Asia/Shanghai）  
覆盖范围：从读取 `CODEX_HANDOFF_2026-08-02.md`、恢复 r15 工作开始，直到二楼固定出梯右转最终调整为 95°并完成验证。  
本文是上一份交接文件的续篇；接手者应先读上一份，再读本文。不要覆盖现有已验证修改。

## 1. 当前结论摘要

目前已经打通并反复验证的主链路如下：

1. 机器人固定站立、FAST-LIO 定位、楼层建图、RL 控制器切换正常。
2. 打开建筑主入口时，一楼电梯门同步打开。
3. 跨过入口后继续前进 2.5 m；入口侧向扫描约 80°已经足够，禁止继续优化该扫描角度。
4. 独立 L 型/短墙电梯模板能稳定产生 `MULTIFLOOR[ELEVATOR_TEMPLATE_LOCALIZED]`，不依赖普通房门 `Doorway`。
5. 一楼可依次到达电梯大厅等待点、门槛和轿厢内部。
6. 轿厢内部导航点已加深，机器人四足能够完整进入轿厢；当前 `car_depth=1.45 m`。
7. 机器人在一楼轿厢内先原地转身面向电梯门，再调用电梯服务。
8. 电梯换层后会重启 FAST-LIO 和楼层地图，并通过 localization generation 隔离坐标。
9. 二楼、三楼不再识别电梯门洞，采用固定语义路线：锁定本 generation 的电梯返回点 A，向前 2 m 出轿厢，顺时针右转，再向主走廊前进 5 m。
10. 二楼和三楼复用同一套出梯、探索、返回 A 的逻辑；三楼完成后可乘电梯回到一楼。此前专项轮次已经成功完成一楼→二楼→三楼→一楼的完整往返。
11. 三层正式探索的固定完成条件是每层完成 4 个不同房间事务；一楼可通过测试参数临时在进入主走廊后强制完成，二楼、三楼仍执行正式房间探索。
12. 上层出梯右转目标最终从 90°试验为 100°，再收敛到 **顺时针 95°**。95°已验证可以完成二楼 2 m 出梯、转向和 5 m 主走廊直行；100°会使其后的 5 m `move_base` 目标以 state=4 中止。

当前最优先未解决问题不是电梯路线，而是二楼房间探索中的失败候选循环：同一房间找不到 0.78 m 净空环视点，却没有在超过失败预算后被淘汰，导致无限重复，无法继续到三楼。

## 2. 服务器、容器和副本

- SSH：`xiaoyi-dev@10.139.197.230`
- SSH key：`C:\Users\20424\.ssh\id_rsa`
- 服务器工程：`/home/xiaoyi-dev/simenv/SimEnv`
- 容器：`simenv-gpu-gui`
- 容器工程：`/workspace/SimEnv`
- 本地工作目录：`D:\Code\TZB\NewSimEnv`
- 本地逐文件恢复副本：`D:\Code\TZB\NewSimEnv\.codex_rollback`

本文生成时状态：

- SSH 正常。
- `simenv-gpu-gui` 已停止，状态为 `Exited (0)`。
- 没有正在运行的测试。
- 最新有效测试日志 UUID：`263c7c3c-8df0-11f1-94be-ac3dcb52715f`。
- 服务器工作树包含大量未提交修改；`src/a1/mission_manager/` 整包目前仍是新增未跟踪目录。不要执行 `git reset --hard`、`git clean` 或用主分支文件覆盖。

## 3. 固定启动与停止流程

继续使用已验证的快速流程，不要重新设计启动顺序：

1. `docker restart simenv-gpu-gui`
2. 容器内后台启动：`env GUI=false /workspace/SimEnv/auto.sh`
3. 紧密轮询 `/clock`，收到第一条消息后再启动任务，不要固定盲等 30 秒。
4. 启动任务：

   ```bash
   roslaunch a1_mission_manager multifloor_test.launch \
     use_rviz:=true \
     force_floor_complete_after_ingress:=true
   ```

   该命令只让一楼在强制进入主走廊后结束；`upper_floor_special_test_mode` 默认仍为 `false`，所以二楼、三楼执行正式探索。

5. 单独后台启动 `gzclient`。
6. 读取 `/root/.ros/log/latest` 指向的 UUID，持续监听：
   - `multifloor_mission-14.log`
   - `frontier_explorer-11.log`
   - `rosout.log`
   - 必要时 `a1_localization-localization_supervisor-2.log`
7. 测试直接退出时，必须先查上述日志的直接原因，禁止盲目改阈值或连续重启。
8. 彻底停止使用 `docker stop -t 15 simenv-gpu-gui`，然后确认容器为 `Exited (0)`。

GUI 必须同时包含 RViz 和 Gazebo。历史上 RViz 曾因运行环境/资源问题退出；不得把“RViz 窗口消失”直接等同于任务算法失败，应检查进程和 ROS UUID 日志。

## 4. 本阶段实现与代码变化

### 4.1 电梯模板与冻结地标

相关文件：

- `src/a1/floor_mapping/include/a1_floor_mapping/door_wall.h`
- `src/a1/floor_mapping/src/floor_mapping_node.cpp`
- `src/a1/floor_mapping/config/floor_mapping.yaml`
- `src/a1/mission_manager/scripts/multifloor_mission_node.py`

电梯入口继续使用独立短墙/L 型结构模板，只在入口侧向扫描事务激活时工作，不创建普通 `Doorway`，避免与主入口和房门竞争。

模板冻结数据包括：

- center
- width
- outward
- observation count / age
- localization generation
- floor session

典型成功样本：中心约 `(5.78, -1.35)`，宽度约 `1.32 m`，5 次稳定观测后冻结。不同轮次坐标会因 localization generation 改变，不能跨轮或跨 generation 比较绝对 XY。

### 4.2 电梯内部目标加深

文件：

- `src/a1/mission_manager/config/multifloor.yaml`
- `src/a1/mission_manager/scripts/multifloor_mission_node.py`

`elevator/car_depth` 当前为 `1.45 m`。该值用于：

```text
car = center - outward * car_depth
```

目的：保证机器人两只后足也越过门槛，完整进入轿厢后再转身、关门和换层。

### 4.3 轿厢内换层前转身

机器人在一楼进入轿厢后，不再保持面向轿厢内部直接调用电梯，而是先转到 `yaw_out`，即面向电梯门外，再调用服务。

转身使用专用较快角速度配置，已经解决此前轿厢内原地转向异常缓慢的问题。当前参数位于 `multifloor.yaml`：

- `transfer_turn_gain: 1.8`
- `transfer_turn_min_speed: 0.55 rad/s`
- `transfer_turn_max_speed: 1.8 rad/s`
- `transfer_turn_tolerance: 0.2 rad`
- `transfer_turn_settle_wall: 0.5 s`

不要把该转身与一楼入口约 80°的电梯扫描视角混淆；后者已经确认足够，不继续优化。

### 4.4 二楼、三楼固定出梯路线和返回点 A

核心函数：

- `exit_upper_floor_without_doorway(floor)`
- `complete_upper_floor_and_return_to_a(floor, special_test)`

目标楼层开门后：

1. 立即锁定当前位置为该 localization generation 的电梯返回点 A。
2. 直行 2 m 出轿厢。
3. 顺时针右转。
4. 沿新航向直行 5 m 到主走廊。
5. 从该主走廊入口姿态调用通用楼层探索事务。
6. 探索完成后返回同 generation 的 A 点。
7. 二楼完成后去三楼；三楼完成后回一楼。

当前最终右转代码为：

```python
yaw_corridor = yaw_out - math.radians(95.0)
```

注意：`multifloor.yaml` 中仍可看到 `upper_floor.exit_forward: 1.0` 和 `corridor_forward: 5.0`，但当前固定路线实现把 2.0 m 和 5.0 m直接写在 Python 中。实际执行以 Python 为准。这是后续应清理的配置/实现不一致技术债，但不要在未回归测试前擅自重构。

### 4.5 上层 ROI 和地图范围

此前二楼探索 ROI 会因 generation 后的入口航向/局部坐标旋转而落出 OccupancyGrid。当前做法是：

- ROI 始终由本楼层 `floor_entry_pose` 在当前 generation 中重新变换；
- 不复用 generation 1 的绝对坐标；
- 扩大 `floor_mapping` 栅格范围，使上层旋转后的 40 m ROI 保持在地图边界内。

当前 `floor_mapping.yaml` 的关键设计是扩大负 Y 方向覆盖；不要恢复为较小旧地图，否则会重新出现 upper-floor ROI 越界/错误旋转表象。

RViz 中 generation 1 残留导航点/标记曾被观察到。原则上它们不应参与新 generation 导航；数值目标必须带 generation 所有权。接手者仍应检查 MarkerArray 是否显式发布 DELETE/DELETEALL，因为仅 UI 残留也会误导现场判断。

### 4.6 四房完成条件

相关文件：

- `src/a1/exploration/config/exploration.yaml`
- `src/a1/exploration/scripts/frontier_explorer_node.py`

三层布局相同，每层完成 4 个不同房间事务即可判定该层探索完成：

```yaml
floor_completion:
  completed_room_count: 4
```

代码使用 `completed_room_branches` 集合按 `(station, side)` 去重，并输出：

```text
fixed-layout room completion: N/4 distinct rooms
```

此前“完成 4/4 后仍继续探索/导致流程退出”的问题已经针对完成路径做过修复：第 4 个新房间完成时应立即返回楼层成功，不能再进入下一轮 frontier 选择。

### 4.7 房间深入与环视

用户要求机器人约深入房间 1.5 m 后环视。这里不是简单把“1.2 m 改成 0.5 m”代表绝对深度；当前目标由门中心、goal extension、可达净空搜索共同决定。

当前配置/代码关键值包括：

- `goal_extension`（本阶段调整过，最终以服务器 `exploration.yaml` 为准）
- `completion_depth: 2.0`
- `scan_clearance: 0.78 m`
- `scan_search_distance: 2.5 m`
- `scan_angular_speed: 0.50 rad/s`

最新有效轮次第一个房间日志显示实际深入 `depth=1.52 m`，符合约 1.5 m 的目标。

### 4.8 RViz 和保存含义

RViz 的“Save”通常保存当前 RViz display、视图、topic 和面板配置到 `.rviz`，不是自动保存整段机器人运动轨迹。轨迹是否持久化取决于对应 Marker/Path 发布者、rosbag 或显式数据导出；不要把 RViz 配置文件当作完整测试记录。

## 5. 重要测试演进

### 5.1 电梯专项阶段

经过多轮专项测试，以下均已成功观察：

- 入口后 2.5 m。
- 约 80°侧向视角足够。
- `ELEVATOR_TEMPLATE_LOCALIZED` 稳定产生。
- 一楼大厅→门槛→轿厢内部成功。
- 轿厢加深后四足完整进入。
- 一楼轿厢内快速转身面向门外。
- 一楼→二楼定位/建图重启成功。
- 二楼固定 2 m、右转、5 m 到主走廊成功。
- 二楼返回 A、进入电梯并去三楼成功。
- 三楼复用相同逻辑，完成后返回一楼成功。

### 5.2 完整探索阶段与 RViz 退出

解除专项限制后恢复主走廊/房间探索，并加入 4 房完成条件。此后若干轮出现 RViz/任务退出。分析表明不能简单归因于“4 房条件”本身，至少出现过以下不同故障：

- 4/4 后完成路径未立即结束；后来已修复。
- 上层 ROI 超出或错误旋转；后来通过 generation-local ROI 和地图范围修复。
- GUI/资源退出；需要区分 GUI 进程问题与 mission failure。
- 定位 `ODOM_POSE_JUMP`；见下一节。

### 5.3 三楼 ODOM_POSE_JUMP 轮次

日志 UUID：`fe7d1760-8dea-11f1-b37d-ac3dcb52715f`。

关键链路：

1. generation 3 完成三楼 `FLOOR_SWITCH_VERIFIED floor=2`。
2. 2 m 出梯和右转成功。
3. 5 m 主走廊直行期间，定位变为 `LOST reason=ODOM_POSE_JUMP`。
4. supervisor 主动正常关闭 generation 3（FAST-LIO 和 adapter 返回 0，不是崩溃）。
5. generation 4 没有启动；大概率卡在 `WAITING_FOR_INPUTS`，现有日志不能确认是 pointcloud 还是 IMU。
6. `move_base` 因 TF 超时和无法取得起始位姿而 state=4，任务失败。

当前跳变阈值：

- 相邻 odom 平移 > 1.0 m；或
- 相邻姿态旋转 > 1.0 rad（约 57.3°）。

判定没有结合两帧 `dt`，所以真实 FAST-LIO 突跳和资源拥塞造成的长发布间隔假阳性尚不能区分。建议后续先增加前后 pose、dx/dyaw/dt 和各原始输入 freshness 日志，不要直接放宽阈值。

任务的 `navigate()` 也没有在导航执行期间持续处理 localization LOST / supervisor generation 变化。即使自动启动 generation 4，generation 3 的 A 点和绝对目标也不能复用。后续应实现：取消目标、零速、等待新 generation TRACKING、清 costmap、废弃旧坐标、按语义阶段重新锚定，并限制重试次数。

### 5.4 90°/100°/95°右转对比

原始固定路线为顺时针 90°。考虑 move_base 航向容差后，用户要求增加目标角度。

100°轮次 UUID：`01666b30-8def-11f1-8132-ac3dcb52715f`。

- 二楼 2 m 出梯成功。
- 100°转向成功。
- 随后的 5 m 目标以 `move_base state=4` 中止。
- 没有 `ODOM_POSE_JUMP`；这是导航几何/规划执行失败，不是定位重启问题。

95°轮次 UUID：`263c7c3c-8df0-11f1-94be-ac3dcb52715f`。

- 二楼 `FLOOR_SWITCH_VERIFIED floor=1` 成功。
- 2 m 出梯成功。
- 95°转向成功。
- 5 m 直行成功。
- 在仿真时刻约 91.0 s 输出：

  ```text
  MULTIFLOOR[UPPER_FLOOR_MAIN_CORRIDOR] floor=1
  ```

- 未出现 `ODOM_POSE_JUMP`。
- 因此当前保留 95°。

有一轮重新运行 UUID `a44a1d44-8ded-11f1-be69-ac3dcb52715f` 被用户明确要求“不算作记录”，不要拿它作为验收依据。

## 6. 当前首要阻塞：失败房间候选无限循环

最新 95°轮次进入二楼正式探索后：

1. 第一个房间 `station=5 side=left` 完整进入、环视、退出，记录为 `1/4`。
2. 随后选择 `station=5 side=right`。
3. 日志反复出现：

   ```text
   room interior reached through frozen door geometry (depth=1.71 m)
   room scan deferred: no reachable 0.78 m-clear observation point found within 2.50 m
   exploration state UPDATE_COVERAGE: room scan/forward exit failed; failure 58/2
   ```

4. 尽管失败计数早已超过显示预算 `/2`，同一个结构门仍不断被重新选择，失败次数从 53 增至 58，任务没有淘汰该 branch，也没有转向其他房间。

这说明至少有一个状态管理缺陷：

- room scan observation-point failure 没有把 active branch 加入本 floor session 的失败/冷却集合；或
- 失败集合使用的 identity 与再次生成候选的 `(station, side)` identity 不一致；或
- room-stage 失败只更新普通 frontier failure，而结构门优先选择路径没有查询该失败状态；或
- `failure 58/2` 的分母并不是淘汰阈值，导致日志含义与行为不一致。

推荐下一步：

1. 从 `select_structural_doorway`、`room_scan` 失败返回、`UPDATE_COVERAGE`、`mark_failed`/黑名单路径追踪同一 branch key。
2. 对不可达环视点增加一次性明确状态，例如 `ROOM_SCAN_POINT_UNREACHABLE`。
3. 达到有限次数后冻结/跳过该 branch，清理 `active_room_branch` 和 stage，再选择其他房间。
4. 黑名单必须限定在当前 floor session + localization generation，避免跨楼层污染。
5. 增加单元测试：同一 `(station, side)` 连续两次 observation-point failure 后不得再次立即被选中。
6. 修复后先用当前一楼 shortcut 配置重跑，确认二楼能从 1/4 继续到 4/4，再验证返梯和三楼 95°路线。

不要通过降低 `scan_clearance` 或无限增大 `scan_search_distance` 来掩盖状态机问题。先保证失败候选能被有限、有解释地淘汰。

## 7. 其他仍需处理的问题

### 7.1 定位监督器可观测性和自动恢复

`localization_supervisor.py` 当前等待 pointcloud、IMU、clock 均在重启请求后变新并持续 settle，但状态只报告笼统的 `WAITING_FOR_INPUTS`。应增加每个输入的 age、seen-after-request 和 missing list，并给等待状态设置 wall-time 上限。

### 7.2 导航中的定位健康联锁

`multifloor_mission_node.py:navigate()` 应持续监视 localization 和 supervisor；当前只等待 move_base 结果。需要在 LOST 或 generation 变化时取消目标并进入受控恢复。

### 7.3 generation 残留可视化

确认 RViz MarkerArray 在楼层/generation 切换时发布 DELETE/DELETEALL。即使残留只影响 UI，也会误导操作人员；如果旧 marker 仍被算法容器持有，则必须同时清空数据。

### 7.4 配置硬编码

固定路线 Python 中硬编码 2 m、5 m、95°；YAML 中 `exit_forward` 当前仍为 1.0。后续可以统一参数来源，但必须在房间循环和三楼定位问题稳定后进行，以免扩大变更面。

## 8. 当前关键参数快照

### 电梯

- `car_depth: 1.45 m`
- `lobby_standoff: 0.85 m`
- 模板观测次数：至少 5
- 模板最小年龄：0.8 wall s
- 换层前转身目标：面向 outward / 电梯门外

### 上层固定路线

- 出轿厢：实际代码 2.0 m
- 右转：顺时针 95°
- 主走廊前进：5.0 m
- 点 A：本 localization generation 内轿厢位置

### 房间探索

- 每层完成房间数：4
- 环视净空：0.78 m
- 环视点搜索半径：2.5 m
- 环视速度：0.50 rad/s
- 最新成功房间实际深入：约 1.52 m

### 定位跳变

- 平移阈值：1.0 m / 相邻消息
- 旋转阈值：1.0 rad / 相邻消息
- supervisor input timeout：3.0 wall s
- input settle：2.0 wall s

## 9. 代码与运行副本同步要求

Python 修改后必须同时确认：

1. 源码：`/workspace/SimEnv/src/...`
2. catkin 运行副本：`/workspace/SimEnv/devel/lib/...`

本阶段多次遇到 `devel/lib/a1_mission_manager/multifloor_mission_node.py` 由 root 所有，普通 SCP 无法覆盖。采用的安全方式是：

1. SCP 到服务器 `/tmp`；
2. 容器停止时或启动后用 `docker cp` 写入容器目标；
3. 对源码和运行副本都执行 `python3 -m py_compile`；
4. 用 `grep` 确认实际运行副本包含目标代码。

本地 `.codex_rollback` 目前包含任务管理器、探索器、建图器、定位、配置和测试脚本的逐文件副本。它不是 Git commit，但在当前未提交工作树中是重要恢复依据。

## 10. 接手后的建议执行顺序

1. 阅读旧交接文件和本文。
2. 检查 SSH、容器状态、服务器源码与 `.codex_rollback` 的核心文件是否一致。
3. 不启动测试，先修复“不可达环视点候选无限重选”的 branch 失败状态。
4. 增加针对 `(station, side)` 失败淘汰的单元测试和 Python 语法检查。
5. 同步源码与 `devel/lib`。
6. 按固定流程启动，一楼 shortcut、上层正式探索。
7. 验证二楼：2 m → 95° → 5 m → 房间 1/4 到 4/4 → 返回 A。
8. 验证三楼同路线，重点监听 `ODOM_POSE_JUMP`。
9. 如果出现定位重启，先读取 supervisor、mission、rosout；不要改 jump 阈值。
10. 完整成功后再整理 YAML/硬编码、marker 清理和 supervisor 可观测性。

## 11. 验收日志关键字

```text
STARTUP_LOCALIZATION_READY
STARTUP_MAPPING_READY
ELEVATOR_TEMPLATE_LOCALIZED
FLOOR_COMPLETE
elevator lobby approach
elevator threshold
INSIDE_ELEVATOR
ELEVATOR_PRETRANSFER_TURN_READY
ELEVATOR_CALL
FLOOR_SWITCH_VERIFIED
ELEVATOR_RETURN_POINT_LOCKED
fixed-route 2 m exit from elevator car
fixed-route right turn toward main corridor
fixed-route 5 m transit to main corridor
UPPER_FLOOR_MAIN_CORRIDOR
fixed-layout room completion: N/4 distinct rooms
MISSION_COMPLETE
MISSION_FAILED
ODOM_POSE_JUMP
WAITING_FOR_INPUTS
```

最后提醒：目前电梯感知、进轿厢、换层、上层固定路线和三层往返已经取得实质性成功。下一阶段应保持这些已验证路径不变，集中修复房间失败候选的有界退出；不要重新设计电梯识别，也不要继续优化一楼约 80°扫描视角。
