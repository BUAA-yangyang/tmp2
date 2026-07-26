# A1 Floor Mapping

当前版本提供 generation 安全的单连续楼层实时建图，并完成第二阶段的生命周期、地图质量和楼层变化检测强化。节点消费 localization 的 Livox 点云、里程计、健康状态与 supervisor generation，在 `odom` 中估计地面并累计二维栅格，同时将有效地面回波和障碍回波变换回原始 `laser_livox` frame 发布，保留标准 costmap clearing 所需的真实传感器原点语义。

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

## 状态与恢复

- `WAITING_FOR_LOCALIZATION`：定位或 generation 未就绪；自动等待。
- `WAITING_FOR_TF`：点云时刻 TF 暂时不可用；恢复后需要连续有效帧门控。
- `INITIALIZING_GROUND`：地面候选不足或仍在初始化。
- `DEGRADED`：输入短时中断、odom 不够新鲜或正在恢复；地图和实时云均无效。
- `MAPPING`：地图与实时观测有效。
- `LOST`：时间回退、非法点云/TF 或长期输入中断；只有 reset 或 generation 更新可恢复。
- `FLOOR_CHANGE_UNSUPPORTED`：连续多帧检测到超出单层假设的高度变化；只有 reset 或 generation 更新可恢复。

消费者必须分别检查 `map_valid` 和 `obstacle_cloud_valid`，并结合 `localization_generation`、`pointcloud_age_sec`、`odom_age_sec` 和 `last_success_tf_age_sec`。状态还提供地面候选、置信度、换层连续帧数、栅格统计、处理时间和地图边界余量。

## 当前算法与边界

地面高度由机器人附近候选点的低分位数带内中值初始化，并检查候选数量、内点比例和离散度。高度带内地面点只写自由证据，障碍点写射线自由证据和终点占据证据；每帧先积分自由射线，再积分障碍终点，降低相邻射线过度抵消墙体的风险。楼层高度变化必须连续多帧成立才会触发安全停止。

当前继续使用固定、有界的 `40 m x 40 m` `odom` 栅格。实际单层稳定性测试的最小边界余量约 `19.04 m`，因此第二阶段没有加入动态扩图。不发布 `map -> odom`，不跨 generation 拼图，不处理正式楼层编号、回环和全局重定位。

## 测试

```bash
catkin_make --pkg a1_floor_mapping -DCATKIN_ENABLE_TESTING=ON
catkin_make run_tests_a1_floor_mapping
catkin_test_results
```

测试包括纯 C++ 地面/栅格算法、ROS 生命周期与故障注入，以及标准 `costmap_2d::ObstacleLayer` marking/clearing 夹具。实际仿真记录可使用：

```bash
rosrun a1_floor_mapping floor_mapping_validation_recorder.py \
  --duration 600 --output /tmp/a1_mapping_validation
```

2026-07-26 的 10 分钟静止实测结果：全程状态为 `MAPPING`，输入与输出均约 `10 Hz`，TF failure 为 0，floor z 峰峰值 `0.000174 m`，处理耗时 P50/P95 为 `3.42/4.70 ms`，最终 occupied/free 为 `300/14306`，RSS 约 `53.7 MB`。

## 后续 navigation 适配要求

本阶段不修改 `src/a1/navigation`。后续接入应将同一个 `/a1/floor_mapping/obstacle_cloud` 配置为两个 ObstacleLayer observation source：marking source 使用较高的最小高度并设置 `marking=true, clearing=false`；clearing source 允许地面回波并设置 `marking=false, clearing=true`。标准 costmap 的单一 source 会对 marking 和 clearing 共用高度过滤，不能同时正确满足这两个需求。测试配置见 `test/costmap_test.yaml`。

导航应使用 `/a1/localization/odom` 与同一 TF 树。若消费累计 OccupancyGrid，应增加 static layer 或等价消费者，并以 mapping status 作为失效门控。当前地图 frame 是 `odom`，在实现全局重定位前不得把结果当作跨 generation 全局地图。
