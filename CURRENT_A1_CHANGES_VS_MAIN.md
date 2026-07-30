# A1 单层楼自主探索版本变更说明

## 1. 对比基线与范围

- 对比基线：`origin/main`
- 基线提交：`b704ff2d560c366c7a67b51b787e486e1681a2b0`
- 当前开发分支：`new`
- 整理日期：2026-07-31
- 范围：从出生点稳定站立，经入口台阶和建筑大门进入一楼，使用定位、建图和导航结果探索一楼房间，完成后返航。
- 暂不包含：危险源识别、危险源上报、多楼层和电梯任务。

当前工作树相对上述 `main` 基线修改了 25 个已跟踪文件，约
`2256 insertions / 318 deletions`，并新增
`src/a1/navigation/launch/single_floor_exploration_dev.launch`。

## 2. 总体架构变化

当前版本明确采用以下职责划分：

1. 探索与导航层只生成期望机体速度，经 `move_base -> twist_mux ->
   cmd_vel_guard -> /cmd_vel` 输出。
2. `State_RL` 接收 `/cmd_vel`，由 RL 策略生成 12 个关节的目标位置。
3. 上游定位由 Livox、IMU 和 FAST-LIO 提供；楼层栅格及门结构由
   `floor_mapping` 产生。
4. 探索器使用定位、栅格、门结构和 move_base 规划结果决策，不读取场景真值。
5. 仿真真值只保留在独立验收 oracle 中，用于评价轨迹、姿态和是否进入 ROI，
   不向探索器、定位器或 move_base 提供 TF、里程计或目标坐标。

## 3. 修改文件与主要代码

### 3.1 探索决策

#### `src/a1/exploration/scripts/frontier_explorer_node.py`

这是本阶段改动最大的文件，新增或重构的主要能力如下：

- 将室外出生点和室内入口锚点区分为两个任务状态。
- 基于出生点定位和初始朝向推导入口通行轴，不再要求场景提供入口真值坐标。
- 增加入门近场障碍检查、短时等待、局部路径验证和门打开后的地图重置。
- 临时切换 DWA 的入口速度配置，并保存、恢复切换前的完整动态参数。
- 订阅 `/a1/floor_mapping/doorways`，把稳定门结构与 frontier 联合用于房门识别。
- 引入入口后 7 m 排除区，跳过大门左右的大型入口分支走廊。
- 按门宽、走廊纵向位置、横向位置和空间关联识别真实房门。
- 建立房间分支记忆：
  - 门中心阶段；
  - 房间内部观察点阶段；
  - 360°扫描；
  - 沿冻结的门几何退出；
  - 恢复进入房间前记录的主走廊方向；
  - 将完成的房间分支永久标记，避免重复进入。
- 房门选择采用同站点侧向优先策略，并在完成左右房间后恢复走廊纵向推进。
- 新增 `corridor_probe_target()`：当房间扫描使前方走廊成为已知自由区、
  但没有形成传统 frontier 时，沿地图验证过的自由空间继续推进。
- corridor probe 保留机器人当前横向车道，避免强制回到错误的走廊中心线。
- 无 frontier 完成条件改为“多个不同地图内容 + 稳定时间”联合确认。
- 新增转向优先恢复：若原地转向不可行，仅执行有限距离倒退，然后重新尝试转向；
  不再允许无限倒车沿走廊运行。
- 返航拆成“走廊深处到室内入口锚点”和“入口锚点到室外出生点”两段。
- 加强取消、超时、动态参数恢复、最终零速度和安全锁的失败闭合处理。

#### `src/a1/exploration/config/exploration.yaml`

新增入口、房门、房间、走廊推进和恢复策略的参数组。关键配置包括：

- 入口速度目标 `0.8 m/s`；
- 入口临时 DWA 前进范围 `1.25–1.40 m/s`；
- 入口临时最大角速度 `0.01 rad/s`；
- 入口障碍保持时间 `6 s`；
- 入口后 `7 m` 内大侧向开口不视为房门；
- 房门宽度范围 `1.0–1.6 m`；
- 房内观察深度、开阔扫描点搜索和 `1.8 rad/s` 扫描速度；
- 走廊无 frontier 时每次推进 `3 m`；
- 有限倒退速度 `0.45 m/s`、单步 `0.35 m`、最多三步；
- 墙钟兜底倍率由 20 调整为 5。

#### `src/a1/exploration/src/a1_exploration/entry_speed_limit.py`

- 动态参数切换由只修改最大速度扩展为同时处理
  `min_vel_x/min_vel_trans/min_vel_theta/sim_time`。
- 保存服务返回的完整实时快照，退出入口模式时精确恢复。
- 对参数服务失败采用失败闭合，不允许在未知速度配置下继续探索。

#### `src/a1/exploration/test/test_entry_speed_limit.py`

- 更新入口动态参数的保存、应用、恢复及失败路径测试。

#### `src/a1/exploration/README.md`

- 更新单层探索入口和运行说明。

### 3.2 定位

#### `src/a1/localization/src/localization_pose_adapter.cpp`

- 从连续 FAST-LIO 位姿估计机体系线速度和角速度。
- 增加时间间隔、最大线速度和最大角速度异常门控。
- 增加低通滤波与可信 twist 协方差。
- 静止命令下将速度估计稳定归零。
- TRACKING 后增加静止漂移监控宽限期，避免把 FAST-LIO 初始收敛误判为漂移。
- FSM 状态只有真实变化时才重置静止锚点。
- 增加相关诊断字段。

这使 DWA 的 `odom_topic` 可以使用真正的上游
`/a1/localization/odom`，不再依赖 Gazebo 里程计真值。

#### `src/a1/localization/config/frames.yaml`

- 启用 twist 估计与滤波参数。
- 初始化有效样本数由 5 增加到 30。
- 增加 3 秒静止漂移监控宽限期。

#### `src/a1/localization/scripts/localization_supervisor.py`

- 控制器就绪状态由单一字符串扩展为状态集合。

#### `src/a1/localization/config/supervisor.yaml`

- FAST-LIO 初始化期间允许 `fixed stand`、`State_RL` 和 `RL` 作为稳定控制状态。

#### `src/a1/localization/config/map.yaml`

- 删除固定出生点的 world 对齐平移和旋转配置。
- 不再把固定世界出生位姿作为定位初始化真值。

### 3.3 楼层建图

#### `src/a1/floor_mapping/include/a1_floor_mapping/core.h`

- `OccupancyIntegrator` 支持显式栅格原点，不再只能以机器人出生点为地图中心。

#### `src/a1/floor_mapping/src/floor_mapping_node.cpp`

- 读取和校验 `grid/origin_x`、`grid/origin_y`。
- 将显式原点传给栅格积分器。

#### `src/a1/floor_mapping/config/floor_mapping.yaml`

- 栅格分辨率改为 `0.075 m`。
- 地图尺寸改为 `100 m × 40 m`，原点为 `(-20, -20)`，覆盖完整长走廊。
- 发布频率保持已验证的 `1 Hz`。
- 地面净空阈值提高到 `0.14 m`，把入口约 8 cm 台阶视为可跨越地面，
  避免二维代价地图将其标记为致命障碍。

### 3.4 move_base 与速度链

#### `src/a1/navigation/config/dwa_local_planner_params.yaml`

- `odom_topic` 从 `/Odometry_gazebo` 改为 `/a1/localization/odom`。
- 平移速度上限提高到 `1.40 m/s`。
- 普通导航禁止连续倒车：`min_vel_x: 0.0`。
- 平移最小有效速度提高到 `0.35 m/s`，避开 State_RL 实测无净位移死区。
- 最大角速度提高到 `1.80 rad/s`，最小有效角速度为 `0.75 rad/s`。
- 加速度提高到 `4.0/6.0`，与 RL 速度跟踪特性匹配。
- 目标位置和航向容差调整到 `0.45 m / 0.80 rad`。

#### `src/a1/cmd_mux/config/guard.yaml`

- 唯一速度出口的硬上限改为 `vx=1.40`、`vy=0.30`、`wz=1.80`。
- 加速度上限改为 `4.0/3.0/6.0`。
- 保留固定频率输出、输入超时归零、安全锁和急停。
- 当前关闭强制 ready 门控；控制器心跳仍由 State_RL 发布并由验收程序检查。

#### `src/a1/cmd_mux/launch/cmd_mux.launch`

- 补充本阶段新增 guard 参数和话题的 launch 接线。

### 3.5 State_RL 与站立切换

#### `src/unitree_guide/unitree_guide/unitree_guide/src/FSM/State_RL_test.cpp`

- State_RL 正式订阅 `/cmd_vel` 作为导航速度入口。
- 进入 RL 时从当前关节角到首个策略目标做 1 秒插值，避免首帧关节跳变。
- 插值期间速度命令保持为零。
- 发布 `/a1/controller_ready` 心跳。
- 增加 RL 安全站立握手：
  - 收到站立请求后先让策略保持零速度；
  - 验证四足接触力；
  - 验证横滚、俯仰和滤波后的角速度；
  - 连续稳定后才切换到 FixedStand。
- 发布 `/a1/safe_stand_ready`。

#### `src/unitree_guide/unitree_guide/unitree_guide/include/FSM/State_RL_test.h`

- 增加 ready 发布器、足端力订阅器、安全站立状态和陀螺滤波成员。

#### `src/unitree_guide/unitree_guide/unitree_guide/include/FSM/NavigationGaitProfile.h`

- 导航步态周期从曾尝试的 `0.90 s` 调整为 `0.60 s`，保留抬腿净空，
  同时避免单对角支撑时间过长造成侧倾。

#### `src/unitree_guide/unitree_guide/unitree_guide/CMakeLists.txt`

- 增加 State_RL 安全站立相关依赖和构建接线。

### 3.6 启动与验收

#### `auto.sh`

- 启动前清理中断后残留的 rosmaster/rosout，避免旧 ROS 图与新一轮启动竞争。

#### `src/a1/navigation/launch/single_floor_exploration_dev.launch`

- 新增正式单层探索组合 launch，统一接入定位、建图、move_base、cmd mux、
  建筑行为和探索器。

#### `src/a1/navigation_tests/launch/single_floor_exploration_dev.launch`

- 与新的正式 navigation launch 对齐，测试包不再维护另一套分叉的运行图。

#### `src/a1/navigation_tests/launch/single_floor_gazebo_acceptance.launch`

- 去掉部分重复/失效参数，使用统一单层探索 launch 和受控物理启动。

#### `src/a1/navigation_tests/scripts/single_floor_gazebo_acceptance.py`

- 增加受控暂停/解暂停、State_RL 就绪、固定站立、足端力和姿态预检。
- 加强 rosbag 必需话题检查、控制器唯一性检查和安全停止。
- 记录探索状态、覆盖率、轨迹、最大姿态、目标成功/失败及完成原因。
- 仿真真值只在该验收节点内部作为 oracle 使用，不发布导航 TF。

#### `src/a1/navigation_tests/test/exploration_runtime_test.py`

- 更新运行时参数与新接口对应的测试。

## 4. 已废弃或不再用于当前流程的设计

以下内容有些仍存在于仓库中供其他任务使用，但不再属于本单层探索主链路：

- 经典足端轨迹/State_move_base 直接承担导航步态控制的方案。
- 探索模块直接生成 12 个关节目标的方案。
- DWA 使用 `/Odometry_gazebo` 真值速度的方案。
- 固定 world 出生坐标对齐定位的方案。
- 由场景真值显式注入 `floor_entry_pose` 的方案。
- 把大门入口左右的大型分支开口当作真实房门的方案。
- 只依赖传统 frontier、无 frontier 就立即结束的方案。
- 只用一次目标附近圆形 visited 区域表示“房间已经探索”的方案。
- 房间退出后只看当前朝向、不恢复走廊行进方向的方案。
- 空间不足时持续倒车沿走廊行驶的恢复方案。
- 入口台阶全程极低速通过的方案。
- `tools/` 下为逐轮诊断临时创建的启动、监控和 bag 分析脚本。

## 5. 第 59 轮截至目前实现的功能

已实现并在 GUI 中观察到：

- 机器人在仿真开始后稳定进入 State_RL。
- 从出生点出发，通过入口台阶和打开的建筑大门。
- 使用 FAST-LIO 定位和 Livox 楼层栅格导航，不使用导航真值。
- 跳过大门后 7 m 范围内的入口分支走廊。
- 识别主走廊第一组和第二组共四个真实房门。
- 对四个房间逐一完成：
  - 导航到门中心；
  - 进入房间内部；
  - 在开阔观察点完成约 360°扫描；
  - 沿原门退出；
  - 恢复主走廊朝向；
  - 标记房间完成且不重复进入。
- 在两组房间之间和第二组房间之后继续沿已知自由走廊推进。
- 稳定确认没有剩余有效 frontier 后宣布探索完成。
- 从走廊深处成功返回室内入口锚点。

第 59 轮关键指标：

- 完成判定仿真时刻：`153.27 s`
- 室内入口锚点返航成功：`194.78 s`
- 最终覆盖率：`0.7609`
- 轨迹总长度：约 `96.92 m`
- 最大横滚：约 `0.132 rad`
- 最大俯仰：约 `0.257 rad`

## 6. 当前已知问题

最后一段“室内入口锚点到室外出生点”错误复用了单向进门配置：

- `min_vel_x = 1.25 m/s`
- `max_vel_theta = 0.01 rad/s`

机器人需要约 180°掉头时几乎无法转向，却被迫高速向前，最终进入墙边/未知区。
DWA 从仿真时刻约 `197.52 s` 开始无法产生有效局部轨迹，并在
`207.32 s` 中止。后续开发应把返航最后一段拆成：

1. 普通 DWA 参数下对准出口；
2. 验证直行门槛通道；
3. 仅在已经对准时启用门槛直行配置；
4. 出门后立即恢复完整转向和减速能力。

参见根目录的 `REPRODUCE_R59_SINGLE_FLOOR_TEST.md`。
