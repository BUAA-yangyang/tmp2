# A1 Floor Mapping

`a1_floor_mapping` 是面向 A1 单楼层 navigation/exploration 的实时建图模块。它消费 localization 输出的 Livox 点云、里程计、健康状态和定位 generation，在 `odom` 坐标系中估计当前地面、过滤障碍并累计二维 OccupancyGrid，同时发布保留真实 Livox 传感器原点语义的实时观测云。

当前版本定位为单楼层 V0：可以用于受健康门控保护的下游集成，但不是跨楼层全局 SLAM。

## 主要功能

- 在机器人附近初始化并持续跟踪楼层地面高度。
- 将地面回波作为 free-space ray，将有效高度范围内的非地面回波作为 occupied endpoint。
- 发布固定大小的 `odom` OccupancyGrid。
- 发布传感器坐标系中的仅障碍点云和地面/障碍联合点云；costmap
  分别用前者 marking、后者 clearing。
- 使用 localization generation 和本地 floor session 隔离地图生命周期，避免跨定位重启拼图。
- 对 localization 失效、输入超时、非法消息、时间回退、TF 缺失和不支持的换层行为安全失效。
- 对点云与 TF 的正常异步到达进行有界等待，始终使用点云采集时刻的精确 TF。
- 提供 reset 服务、诊断、下游 health gate、路线运行器和验收记录器。

## 功能范围

当前保证：

- 单个 localization generation 内的单个连续楼层；
- 在线局部障碍观测和累计二维栅格；
- generation/reset 后清除旧证据；
- 检测到不支持的持续楼层变化时安全停止；
- 正式算法不读取 Gazebo 真值、建筑布局或裁判数据。

当前不提供：

- 正式建筑楼层号识别；
- 多楼层缓存、切换和历史楼层恢复；
- 跨 localization generation 拼图；
- 回环、全局重定位和 `map -> odom`；
- 地图加载或可跨 generation 使用的持久化地图产品；
- 动态扩图。

## 数据流

```text
/a1_localization/livox_pointcloud ─┐
/a1/localization/odom              ├─> floor_mapping_node
/a1/localization/status            │      ├─> marking_cloud (laser_livox)
/a1/localization/supervisor_status ┘      ├─> OccupancyGrid (odom)
TF odom -> base/laser_livox                ├─> obstacle_cloud (laser_livox)
                                            └─> status + diagnostics
```

点云与 localization TF 经过不同处理链路，同一 stamp 的点云可能先于 TF 到达。节点先将点云放入有界队列，只在该 stamp 的 `odom -> laser_livox` 和 `odom -> base` 都可用后按时间顺序处理。节点不会使用 `Time(0)` 或最新 TF 代替消息时刻 TF。

队列同时受 ROS 时间、单调墙钟和容量约束。永久 TF 缺失或队列溢出仍会令输出失效；generation 更新、reset 或 localization 失效会立即丢弃所有待处理旧点云。

## ROS 接口

### 输入

| 名称 | 类型 | frame/关键字段 | 用途 |
| --- | --- | --- | --- |
| `/a1_localization/livox_pointcloud` | `sensor_msgs/PointCloud2` | `laser_livox`，非零且单调 stamp | 地面与障碍原始观测 |
| `/a1/localization/odom` | `nav_msgs/Odometry` | `odom -> base` | localization 活性和位姿契约 |
| `/a1/localization/status` | `diagnostic_msgs/DiagnosticStatus` | `state=TRACKING`、`results_valid=true` | 定位结果有效性门控 |
| `/a1/localization/supervisor_status` | `diagnostic_msgs/DiagnosticStatus` | `generation` | 定位坐标系世代 |
| TF | tf2 | `odom -> base`、`odom -> laser_livox` | 点云时刻机器人与传感器位姿 |

点云至少必须包含 FLOAT32 `x/y/z` 字段；非法布局、零时间戳、非单调时间或无有限点会触发安全失效。

### 输出

| 名称 | 类型 | frame | 语义 |
| --- | --- | --- | --- |
| `/a1/floor_mapping/marking_cloud` | `sensor_msgs/PointCloud2` | `laser_livox` | 仅包含按 `point.z-floor_z` 分类的障碍回波；与输入同 stamp，只在 `MAPPING` 时发布，供 costmap marking |
| `/a1/floor_mapping/obstacle_cloud` | `sensor_msgs/PointCloud2` | `laser_livox` | 当前有效地面与障碍回波；保留真实传感器原点供 clearing 使用 |
| `/a1/floor_mapping/map` | `nav_msgs/OccupancyGrid` | `odom` | 累计单楼层二维栅格，latched |
| `/a1/floor_mapping/status` | `diagnostic_msgs/DiagnosticStatus` | — | 当前状态、有效性、generation/session 和统计量 |
| `/a1/floor_mapping/diagnostics` | `diagnostic_msgs/DiagnosticArray` | — | 与 status 相同的标准诊断包装 |

OccupancyGrid 的值为：

- `-1`：unknown；
- `0`：free；
- `100`：occupied。

地图是 latched 输出。收到旧地图不代表它当前可用，消费者必须同时要求 `map_valid=true`。

### 服务

```text
/a1/floor_mapping/reset  std_srvs/Trigger
```

reset 会递增 `floor_session_id`，清除地面估计、栅格、恢复计数和待处理点云。reset 不改变 localization generation。

## 状态机

| 状态 | 含义 | 自动恢复 |
| --- | --- | --- |
| `WAITING_FOR_LOCALIZATION` | localization 或 generation 尚未有效 | 是 |
| `WAITING_FOR_TF` | 精确时间戳 TF 等待超时或队列异常 | 是，随后经过 recovery gate |
| `INITIALIZING_GROUND` | 地面证据不足或仍在收集稳定初始化帧 | 是 |
| `DEGRADED` | 短时输入问题、无效地面帧超限或正在恢复 | 是 |
| `MAPPING` | 地图和实时观测均有效 | — |
| `LOST` | 非法输入、时间回退或长期输入中断 | 否；需要 reset 或新 generation |
| `FLOOR_CHANGE_UNSUPPORTED` | 持续检测到超出单楼层假设的地面变化 | 否；需要 reset 或新 generation |

只有以下条件同时成立时才进入 `MAPPING`：

- localization 为 `TRACKING/results_valid=true`；
- supervisor generation 已知；
- 点云消息契约有效；
- 点云 stamp 的 `odom -> base/laser_livox` TF 可用；
- 地面已完成稳定初始化；
- recovery valid-frame gate 已满足。

## generation、session 与有效性

- `localization_generation`：localization 坐标系世代，由 supervisor 提供。
- `floor_session_id`：mapping 进程内的连续楼层会话编号；generation 改变或显式 reset 时递增。
- `floor_id=unassigned`：模块没有正式建筑楼层号证据。

消费者必须把 generation、session 和数据有效性一起绑定，至少检查：

```text
state == MAPPING
map_valid == true                  # 消费累计地图时
obstacle_cloud_valid == true       # 消费任一实时点云时
marking_cloud_valid == true        # 消费仅障碍 marking 点云时
localization_generation == expected_generation
floor_session_id == expected_session
```

常用诊断字段：

| 字段 | 含义 |
| --- | --- |
| `pointcloud_input_age_sec` | 最后收到点云距当前 ROS 时间的年龄 |
| `pointcloud_age_sec` | 最后成功处理点云的 ROS 时间年龄 |
| `last_success_tf_age_sec` | 最后成功精确配准的 ROS 时间年龄 |
| `*_wall_heartbeat_age_sec` | 对应进程链路的墙钟心跳年龄 |
| `tf_pending_clouds` | 当前等待精确 TF 的点云数 |
| `tf_failure_count` | TF 等待超时或队列溢出的累计次数 |
| `ground_candidates/inliers/points` | 地面选择和最终过滤统计 |
| `floor_z/floor_dispersion` | 当前地面高度和离散度 |
| `occupied/free/unknown_cells` | 当前栅格统计 |
| `processing_time_ms` | 最近一帧处理耗时 |
| `minimum_boundary_margin_m` | 机器人轨迹距固定地图边界的最小余量 |
| `map_update_sequence` | 成功积分帧计数 |

## 时间语义

- 输入 freshness、DEGRADED/LOST 门槛使用 ROS 时间；仿真中即 `/clock`。
- 点云、odom、TF 同步和时间回退使用消息 header stamp。
- TF 队列另有墙钟上限，用于 `/clock` 或上游进程冻结时退出等待。
- 墙钟 heartbeat 只判断进程链是否仍在推进，不用于替代仿真业务时间。

低 Gazebo real-time factor 不应被当作输入丢帧。

## 建图算法概述

1. 在机器人附近、配置高度范围内收集地面候选。
2. 冷启动使用低分位种子、高度带中值、最少点数、内点比例和离散度检查。
3. 初始化后锚定可信 `floor_z` 高度带，以绝对支持点数持续跟踪地面。
4. 地面高度带内的回波只积分 free ray。
5. 有效障碍高度内的回波积分 free ray 和 occupied endpoint。
6. 仅相对当前 `floor_z` 通过障碍高度判据的点进入 `marking_cloud`；
   `obstacle_cloud` 保持地面+障碍联合输出，兼容既有清障和可视化消费者。
7. 每帧先处理地面 free ray，再处理障碍 endpoint，减少射线对墙体的过度清除。
8. 持续的新地面高度候选会触发 `FLOOR_CHANGE_UNSUPPORTED`，不会自动切层。

默认地图为 `40 m x 40 m`、`0.05 m/cell`，以 `odom` 原点为中心，发布频率 1 Hz。地图不会动态扩展，运行时应监控 `minimum_boundary_margin_m`。

## 启动

先启动并确认 localization 健康：

```bash
roslaunch a1_localization localization.launch
rostopic echo -n 1 /a1/localization/status
```

启动 mapping：

```bash
roslaunch a1_floor_mapping floor_mapping.launch
```

带下游停车门控启动：

```bash
roslaunch a1_floor_mapping floor_mapping_with_health_gate.launch
```

检查状态：

```bash
rostopic echo -n 1 /a1/floor_mapping/status
```

## 主要配置

默认配置为 `config/floor_mapping.yaml`。

| 参数 | 默认值 | 含义 |
| --- | ---: | --- |
| `timeouts/tf_queue_ros` | 0.30 s | 点云 stamp 相对 ROS 时间的最大 TF 等待窗口 |
| `timeouts/tf_queue_wall` | 2.0 s | TF 等待墙钟上限 |
| `queues/pointcloud` | 10 | 待配准点云容量 |
| `timeouts/input_degraded` | 1.0 s | 输入短时失效门槛 |
| `timeouts/input_lost` | 3.0 s | 输入粘滞 LOST 门槛 |
| `recovery/valid_frames` | 3 | 恢复到 MAPPING 前的连续有效帧 |
| `ground/minimum_candidates` | 80 | 初始化/跟踪所需最少支持点 |
| `ground/minimum_inlier_ratio` | 0.20 | 冷启动和新楼层候选的最低内点比例 |
| `ground/initialization_frames` | 6 | 地面初始化稳定帧数 |
| `ground/invalid_frame_tolerance` | 3 | 已知地面允许的连续无效帧数 |
| `ground/floor_change_frames` | 10 | 判定不支持换层的连续帧数 |
| `filter/maximum_range` | 8.0 m | 点云积分最大距离 |
| `grid/resolution` | 0.05 m | 栅格分辨率 |
| `grid/width,height` | 40 m | 固定地图范围 |

调整地面参数必须基于失败点云和空间证据，并补充回归测试；不要只为让某条路线通过而降低支持门槛。

## navigation/costmap 集成

同一个 `/a1/floor_mapping/obstacle_cloud` 应配置为两个 `costmap_2d::ObstacleLayer` observation source：

- marking source：过滤地面，`marking=true, clearing=false`；
- clearing source：允许地面回波，`marking=false, clearing=true`。

可直接参考 `config/costmap_mapping_sources.yaml`。单个 source 会让 marking 和 clearing 共用同一高度过滤，无法同时正确标记障碍和利用地面回波清除动态障碍。

导航必须使用 `/a1/localization/odom` 和同一 TF 树。若消费累计 OccupancyGrid，需要 static layer 或等价消费者，并以 mapping status 作为强制健康门控。

## 测试

```bash
catkin_make --pkg a1_floor_mapping -DCATKIN_ENABLE_TESTING=ON
source devel/setup.bash
catkin_make run_tests_a1_floor_mapping -DCATKIN_ENABLE_TESTING=ON
catkin_test_results build/test_results/a1_floor_mapping
```

测试覆盖：

- 地面选择、稳定初始化和持续换层检测；
- free/occupied 栅格积分和边界裁剪；
- localization/generation/reset 生命周期；
- 点云先到、TF 短时延迟后的精确配准；
- 永久 TF 缺失、恢复门控和 generation 切换清队列；
- 非法输入与时间回退；
- costmap marking/clearing 和传感器原点；
- health gate 的状态、generation/session 和超时关闭。

## 验收工具

静止或耐久记录：

```bash
rosrun a1_floor_mapping floor_mapping_validation_recorder.py \
  --duration 600 --output /tmp/a1_mapping_validation
```

健康门控路线运行：

```bash
rosrun a1_floor_mapping floor_mapping_route_runner.py \
  --route src/a1/floor_mapping/config/validation_route.yaml \
  --output artifacts/floor_mapping/route-001
```

路线运行器仅在 mapping 健康时发布限速速度；失效、超时、路线段结束和进程退出都会归零。产物包含 manifest、状态 CSV、odom/旁路真值轨迹和最终 PGM 地图。旁路真值只用于验收，不反馈给 localization 或 mapping。

## 已知限制与交付状态

当前模块可作为有条件初步交付版本用于单楼层受控集成。已知风险包括：

- 固定时间开环路线不能证明实际完成指定空间路径；
- localization 的长期高度漂移会反映到 `odom` 中的 `floor_z`；
- 门口等遮挡场景的地面支持可能接近默认绝对门槛；
- 固定 40 m 地图可能在长距离任务中触边；
- 完整交付仍需重复 seed、地图几何、动态障碍和长时间连续运动验收。

任何消费者都不得绕过 status 有效性，仅凭 latched 地图或最近一次点云继续运行。
