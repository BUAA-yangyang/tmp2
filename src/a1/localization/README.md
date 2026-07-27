# A1 Localization

`a1_localization` 是 Unitree A1 的三维激光惯性定位与在线建图模块。它适配仿真 Livox
点云、机体 IMU 和 FAST-LIO，将第三方算法的内部输出转换为项目统一的 odometry、TF、
注册点云、在线三维地图和定位健康状态，并提供受控重新初始化与地图产品保存能力。

本模块当前交付的是单个 estimator generation 内的局部连续三维定位和在线地图，不是加载
已有地图的全局重定位系统，也不保证跨进程重启或机器人位姿 Reset 后坐标连续。

## 1. 模块职责

本模块负责：

- 将 Gazebo `/scan` 转换为 FAST-LIO 可消费的 XYZI `PointCloud2`；
- 配置并管理 FAST-LIO 估计器；
- 发布项目标准的 `odom -> base` 位姿和 TF；
- 发布注册点云和 FAST-LIO ikd-tree 在线三维地图；
- 监测点云、IMU、clock 和 odometry 的新鲜度、时间戳及基本有效性；
- 在定位失效时停止发布可信结果；
- 对时间回退或严重位姿跳变执行受控 estimator 换代；
- 将可信的新鲜在线地图保存为可校验的 PCD 产品。

本模块不负责：

- 楼层识别、地面过滤和二维 `OccupancyGrid`；
- 路径规划、探索目标或任务调度；
- `/cmd_vel` 仲裁、机器人站立、倒地恢复或关节控制；
- 门、电梯等建筑行为；
- 使用 Gazebo 真值修正正式定位结果；
- 跨 generation 地图拼接、`map -> odom` 全局校正或已有地图重定位。

## 2. 运行结构

```text
/scan (sensor_msgs/PointCloud)
    |
    v
pointcloud_adapter
    |
    +--> /a1_localization/livox_pointcloud (PointCloud2 XYZI)
             |
             +-------------------+
             |                   |
             v                   v
       FAST-LIO mapping     health monitoring
             |
             +--> raw odom / registered cloud / ikd-tree map
                             |
                             v
                  localization_pose_adapter
                             |
                             +--> /a1/localization/odom
                             +--> odom -> base TF
                             +--> /a1/localization/cloud_registered
                             +--> /a1/localization/map
                             +--> /a1/localization/status
                             |
                             v
                  localization_map_manager
                             |
                             +--> /a1/localization/save_map

/trunk_imu ------------------> FAST-LIO + health monitoring
/clock ----------------------> health monitoring

localization_supervisor
    +--> owns exactly one FAST-LIO + pose-adapter process group
    +--> /a1/localization/reinitialize
    +--> /a1/localization/supervisor_status
```

时间语义：传感器新鲜度、clock/odom/点云时间一致性和 estimator 健康门控使用
ROS 时间；在Gazebo中即为`/clock`仿真时间。墙钟仅用于进程退出、重启关闭和外部
测试watchdog。低实时因子会拉长墙钟耗时，但不应把仿真时间内连续的10 Hz输入误报为
`INPUT_TIMEOUT`。

`pointcloud_adapter` 和 `localization_map_manager` 在主 launch 生命周期内持续运行。
FAST-LIO 与 `localization_pose_adapter` 组成受 supervisor 管理的 estimator 进程组，发生必须
重新初始化的故障时整体停止并以新 generation 启动，避免多个 FAST-LIO 实例并存。

## 3. 输入

| 话题 | 类型 | 默认频率 | 用途 |
|---|---|---:|---|
| `/scan` | `sensor_msgs/PointCloud` | 约 10 Hz | Gazebo Livox 原始点云 |
| `/trunk_imu` | `sensor_msgs/Imu` | 数百 Hz | FAST-LIO 正式 IMU 输入 |
| `/clock` | `rosgraph_msgs/Clock` | 随仿真物理更新 | 仿真时间和时间回退检测 |

默认要求 `/scan` 的 frame 为 `laser_livox`。适配器会丢弃非有限点，并在缺少 intensity 时
填入配置的默认值。

`/trunk_imu` 与当前外参是已完成仿真验证的组合。额外 `/livox/imu` 不是本模块默认正式输入，
不要仅为规避仿真 Reset 问题直接替换 IMU；两路 IMU 使用相同 Gazebo 插件，且定位性能和
外参需要重新完整验收。

## 4. 公开输出

### 4.1 标准输出

| 接口 | 类型 | frame/语义 |
|---|---|---|
| `/a1/localization/odom` | `nav_msgs/Odometry` | `header.frame_id=odom`，`child_frame_id=base` |
| `/a1/localization/cloud_registered` | `sensor_msgs/PointCloud2` | 固定起点模式下转换到 `world` |
| `/a1/localization/map` | `sensor_msgs/PointCloud2` | FAST-LIO 当前 ikd-tree 地图，固定起点模式下 frame 为 `world` |
| `odom -> base` | TF | 与标准 odometry 同时间戳 |
| `world -> odom` | TF | 由配置的固定出发位姿和首个有效 FAST-LIO 位姿建立 |
| `/a1/localization/status` | `diagnostic_msgs/DiagnosticStatus` | 定位状态与结果有效性 |
| `/a1/localization/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 诊断聚合输出 |
| `/a1/localization/supervisor_status` | `diagnostic_msgs/DiagnosticStatus` | estimator generation 和重启状态 |

只有状态为 `TRACKING` 时，pose adapter 才发布标准 odometry、TF、注册点云和地图。
`/a1/localization/map` 不是 latched topic，避免定位失效后新订阅者仍收到一份看似有效的旧图。

FAST-LIO 当前不提供可信 twist。本模块保留输入中的零值占位，但将 twist 对角协方差设为较大
值；下游不得把它解释为机器人真实静止。

### 4.2 原始内部接口

`/a1_localization/...` 命名空间中的话题是内部适配或第三方 raw 接口，例如：

```text
/a1_localization/livox_pointcloud
/a1_localization/fast_lio/odom_raw
/a1_localization/fast_lio/cloud_registered_raw
/a1_localization/fast_lio/map_raw
```

下游业务模块应优先消费 `/a1/localization/...` 标准接口，不应依赖 FAST-LIO raw frame 名称。

## 5. 坐标系与 generation

FAST-LIO 内部使用 `camera_init -> body`。pose adapter 将其转换为项目使用的：

```text
odom -> base
```

仿真默认启用固定出发点 world 对齐。适配器在每个 estimator generation 的首个有效
FAST-LIO 位姿处建立固定的 `world -> odom`，使配置的初始 base 位姿为
`(0.0, -3.2, 0.6, yaw=1.5708)`。该过程不订阅 Gazebo 真值；参数位于
`config/frames.yaml`，必须与仿真出生参数同步。注册点云和在线地图的点坐标会实际转换到
`world`，并以 `frame_id=world` 发布，而不是只改 frame 标签。

固定出发点模式要求新 generation 只在机器人回到配置起点后建立。在任意位置直接重启
estimator 会把当前位置误认为配置起点；这种情况下必须先执行机器人 reset 并等待稳定。

当前 A1 URDF 中 `base -> trunk -> imu_link` 为固定等价关系，因此配置的 `imu -> base` 变换
为单位变换。若 URDF 改变，必须同步审查 `config/frames.yaml`。

这里的 `odom` 仍是每个 FAST-LIO estimator generation 启动时建立的局部连续坐标：

- 同一 generation 内应保持连续；
- 受控重新初始化后会建立新原点；
- 不保证跨 generation 连续；
- 不提供全局 `map -> odom` 修正；
- 下游不得直接拼接不同 generation 的轨迹或地图。

当前 generation 可在 `/a1/localization/supervisor_status` 的 `values` 中查看。正式下游接口
后续应把 generation/frame epoch 纳入固定消息字段。

## 6. 健康状态

状态机包含：

| 状态 | 含义 |
|---|---|
| `WAITING_FOR_SENSORS` | 尚未收到全部必要输入 |
| `INITIALIZING` | 正在积累连续有效 odometry |
| `TRACKING` | 当前基本输入和输出健康，允许发布标准结果 |
| `DEGRADED` | 输入短时超时，暂不应视为完全健康 |
| `LOST` | 输入丢失、时间回退、frame/数值错误或严重位姿跳变 |
| `STOPPED` | estimator 未运行 |

重点字段：

```text
state
reason
results_valid
reinitialization_required
consecutive_valid_odometry
pointcloud_age_sec
imu_age_sec
odom_age_sec
clock_age_sec
```

`results_valid=true` 是当前阶段下游能否消费标准输出的权威标志。
`reinitialization_required=true` 表示旧 estimator 不能仅凭后续健康样本恢复，必须换代。

当前健康门控能够检测输入中断、时间戳回退、非有限位姿、frame错误和单帧大跳变，但它不是
完整的地图几何质量评估器，不能保证识别渐进漂移、重复环境误匹配或机器人倒地。

## 7. 编译与启动

在工作区根目录执行：

```bash
source /opt/ros/noetic/setup.bash
catkin_make --pkg a1_localization fast_lio -j2
source devel/setup.bash
```

仿真、Livox点云和 `/trunk_imu` 已启动后运行：

```bash
roslaunch a1_localization localization.launch
```

不要同时手工启动第二个 `localization_estimator.launch`。正常运行时 estimator 应由
`localization_supervisor.py` 独占管理。

## 8. 启动后检查

检查输入：

```bash
timeout 10 rostopic hz /scan -w 3
timeout 10 rostopic hz /trunk_imu -w 10
```

检查状态：

```bash
rostopic echo -n 1 /a1/localization/supervisor_status
rostopic echo -n 1 /a1/localization/status
```

开始使用定位前应满足：

```text
supervisor: RUNNING
state: TRACKING
reason: HEALTHY
results_valid: true
reinitialization_required: false
```

检查输出：

```bash
timeout 10 rostopic hz /a1/localization/odom -w 5
timeout 10 rostopic hz /a1/localization/cloud_registered -w 5
timeout 12 rostopic hz /a1/localization/map -w 2
rosrun tf tf_echo odom base
```

地图默认约每 2 秒仿真时间导出一次。短观察窗口没有收到地图时，先确认点云、IMU、odometry
和 `TRACKING` 状态，再延长观察窗口。

## 9. 受控重新初始化

手动请求 estimator 换代：

```bash
rosservice call /a1/localization/reinitialize
```

然后监测：

```bash
rostopic echo /a1/localization/supervisor_status
rostopic echo /a1/localization/status
```

预期过程：

```text
旧 estimator 停止
→ generation 增加
→ WAITING_FOR_SENSORS / INITIALIZING
→ TRACKING
→ 新地图重新建立
```

重新初始化会清除 FAST-LIO 内存地图并重建局部 `odom`，不能用来“无损续接”旧地图。

## 10. 地图更新与保存

FAST-LIO 将当前扫描增量加入 ikd-tree，并按照 `map_publish_interval` 低频导出当前有效地图。
该地图不是简单无限累加原始点云：重复点受体素/近邻过滤，局部滑动范围外的点也可能被删除。

保存前设置项目内产物目录和唯一地图 ID：

```bash
RUN=/workspace/SimEnv/artifacts/localization_validation/manual_run
mkdir -p "$RUN"

rosparam set /a1_localization/localization_map_manager/output_root "$RUN"
rosparam set /a1_localization/localization_map_manager/map_id map_product
rosservice call /a1/localization/save_map
```

成功后生成：

```text
map_product/
├── map.pcd
└── metadata.yaml
```

保存条件包括：

- localization 为 `TRACKING` 且 `results_valid=true`；
- 已收到有效在线地图；
- 固定起点模式下地图 frame 为 `world`；
- 地图不超过配置的时效；
- 点数达到最小值；
- XYZI 全部有限；
- PCD 写入后能够重新读取并保持点数一致。

元数据包含点数、边界、分辨率、输入话题、外参、固定世界起点、版本和 SHA-256。保存成功表示文件和基础
数据契约有效，不表示地图几何必然无漂移或无重影。

`overwrite=true` 当前未实现原子替换；重复保存应使用新的 `map_id`。

## 11. RL 按键“8”Reset的重要限制

当前 `unitree_guide` 中按键“8”会通过 Gazebo服务把机器人和关节直接送回出生状态，但不会
重置 `/clock`，也没有通知 localization。真实测试已经确认：机器人移动约 1.64 m 后按“8”，
旧 generation 仍保持 `TRACKING`，reset后点云继续进入旧 ikd-tree，并产生约 1.9 m 的出生点
坐标错位和异常地图扩张。

因此，在完成跨模块Reset生命周期修复之前：

```text
按键8之后，当前generation的位姿、TF、注册点云和地图必须视为无效。
```

临时安全流程：

```text
停止运动
→ 按8
→ 按2重新站立
→ 等待机器人稳定
→ rosservice call /a1/localization/reinitialize
→ 等待generation增加且重新进入TRACKING
→ 从新地图重新开始
```

按“8”前后的地图不能直接拼接。完整修复需要控制器声明Reset事件、localization提供外部挂起
接口，并由 bringup/协调层在机器人稳定后恢复新 generation。

## 12. 测试

运行包测试：

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
catkin_make run_tests_a1_localization -j2
catkin_test_results build/test_results/a1_localization
```

当前测试覆盖：

- 点云适配接口和配置契约；
- 标准odometry、TF和frame转换；
- 初始化、超时、时间回退和结果抑制；
- supervisor连续受控换代和单实例约束；
- 地图frame、有限值、时效、保存、回读和元数据；
- 验收记录器不向正式定位链路反馈Gazebo真值。

真实仿真验收还应覆盖正常静止、直线、旋转、长期运行、传感器中断、地图保存以及Reset场景。

## 13. 常见问题

### 状态一直是 `WAITING_FOR_SENSORS`

检查：

```bash
rostopic hz /a1_localization/livox_pointcloud
rostopic hz /trunk_imu
rostopic hz /clock
```

### `save_map`返回 `online map is stale`

等待下一帧 `/a1/localization/map` 后立即重试，不要绕过时效检查。

### RViz仍显示旧地图

RViz可能保留最后一帧画面。必须以 `/a1/localization/status`、消息时间戳和generation判断结果
是否有效，不能仅凭画面仍存在判定地图正在发布。

### `reset_simulation`后长时间没有IMU

Gazebo IMU插件可能保留reset前更新时间，直到新仿真时间追上旧时间才恢复。这与按键“8”
不是同一问题。长时间仿真后不要把裸 `reset_simulation` 作为快速恢复方式。

## 14. 当前交付边界

当前可交付：

- 稳定运行、无外部位姿瞬移条件下的单generation三维定位；
- 标准odometry、TF、注册点云和在线三维地图；
- 基础健康状态、结果失效和受控重新初始化；
- 可信新鲜地图的PCD产品保存。

当前未交付：

- RL按键“8”的自动安全生命周期联动；
- 跨generation地图连续性和全局重定位；
- 地图几何质量在线判定；
- 正式固定字段的跨模块状态消息；
- 可直接替代系统Reset协调器的多模块编排。
