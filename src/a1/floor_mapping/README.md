# A1 Floor Mapping

第一阶段提供 generation 安全的单连续楼层实时建图。节点消费 localization 的 Livox 点云、里程计、健康状态与 supervisor generation，在 `odom` 中估计地面并累计二维栅格，同时将有效地面回波和障碍回波变换回原始 `laser_livox` frame 发布，保留标准 costmap clearing 所需的真实传感器原点语义。

## 启动

```bash
roslaunch a1_floor_mapping floor_mapping.launch
```

只有 localization 为 `TRACKING/results_valid=true`、generation 已知、点云时刻 TF 可用且地面初始化完成后，状态才进入 `MAPPING`。

## 接口

- 输入：`/a1_localization/livox_pointcloud`、`/a1/localization/odom`、`/a1/localization/status`、`/a1/localization/supervisor_status`、TF `odom -> laser_livox`
- 输出：`/a1/floor_mapping/obstacle_cloud`（`laser_livox`）、`/a1/floor_mapping/map`（`odom`）、`/a1/floor_mapping/status`、`/a1/floor_mapping/diagnostics`
- 服务：`/a1/floor_mapping/reset`

generation 变化会立即清空地面状态和全部栅格证据。localization 失效、TF 缺失或点云时间回退时不发布新的可信观测。OccupancyGrid 是 latched 消息，消费者必须同时检查 status 的 `map_valid=true`，不能仅凭收到旧地图判断有效。

## 当前算法与边界

地面高度由机器人附近候选点的低分位数带内中值初始化，并通过逐帧限幅更新；高度带内地面点只写自由证据，障碍点写射线自由证据和终点占据证据。第一阶段使用固定、有界的 `odom` 栅格，不发布 `map -> odom`，不跨 generation 拼图，不处理正式楼层编号、回环和全局重定位。

## 测试

```bash
catkin_make --pkg a1_floor_mapping -DCATKIN_ENABLE_TESTING=ON
catkin_make run_tests_a1_floor_mapping
catkin_test_results
```

runtime 测试验证健康门控、带时间戳 TF、sensor-frame 输出、自由/占据栅格以及 generation 清图。实际仿真用于复核地面候选数量、点云频率、处理开销和地图几何。

## 后续 navigation 适配要求

本阶段不修改 `src/a1/navigation`。后续接入应将实时观测源指向 `/a1/floor_mapping/obstacle_cloud`，使用 `/a1/localization/odom` 与同一 TF 树；costmap 对地面回波设置高于 mapping `ground_clearance` 的最小 marking 高度，并保持 clearing 开启。若使用累计 OccupancyGrid，应增加 static layer 或等价消费者，并以 mapping status 作为失效门控。当前地图 frame 是 `odom`，在实现全局重定位前不得把保存结果当作跨 generation 全局地图。
