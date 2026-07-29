# a1_navigation

单楼层二维导航模块。基于 ROS `move_base` 完成目标接收、全局规划、局部避障和速度规划，
只输出 `/cmd_vel_nav`；最终 `/cmd_vel` 必须由 `a1_cmd_mux` 仲裁与保护后发布。

## 当前状态

截至 2026-07-27，第一个里程碑已在仿真中通过：

- 使用 Gazebo Livox Mid-360 原始 `/scan` 作为真实障碍输入；
- 使用 Gazebo 真值 odom/TF 作为 **dev-only 定位替身**；
- RViz 或 `MoveBaseAction` 可以发送二维目标；
- 机器人会拒绝落在墙内的目标，也能绕过墙体走到自由目标；
- 全链路为 `move_base -> /cmd_vel_nav -> twist_mux -> guard -> /cmd_vel`；
- 到站、取消、输入超时后 `/cmd_vel` 会归零。

这还不是正式定位和持久楼层地图版本。FAST-LIO2、楼层二维地图和多楼层任务调度仍需后续接入。

## 三个核心输入如何解决

### 1. 地图来源

里程碑 1 不加载静态地图。全局和局部 costmap 都使用：

```text
global_frame: odom
static_map: false
rolling_window: true
```

代价地图跟随机器人滚动，只由实时障碍点云填充；`GlobalPlanner/allow_unknown=true` 允许在尚未
观测的自由方向规划。后续接入 `a1_floor_mapping` 时，应改为标准
`nav_msgs/OccupancyGrid`、`map` 坐标系和 `static_layer`，不需要自定义一份地图消息。

### 2. 定位

开发期使用：

- TF `odom -> base`：`state_from_gazebo` 从 Gazebo 真值派生；
- `/Odometry_gazebo`：DWA 读取当前 base-frame twist。

必须用 `PUBLISH_REFEREE_TF=1` 显式启用真值 TF；默认关闭，避免与正式定位争抢 TF 所有权。

正式系统应由 `a1_localization` 独占发布：

- `/a1/localization/odom`
- TF `odom -> base`

当前团队版 `/a1/localization/odom` 的 twist 是高协方差零占位，不能直接作为 DWA 的可信速度。
正式替换前必须由 localization 提供可信 base-frame twist，或新增经过评审的速度估计适配器。

### 3. 障碍感知

仿真以 `ENABLE_LIVOX=1` 加载 Livox。Gazebo 插件发布的是
`sensor_msgs/PointCloud` `/scan`；`scan_to_obstacle_cloud.py` 会：

- 丢弃非有限点和无回波 `(0,0,0)`；
- 按默认 0.6 m 盲区和 20 m 最大距离过滤；
- 转成 costmap 使用的 `sensor_msgs/PointCloud2`；
- 发布 `/a1_nav/obstacle_cloud`。

正式导航只消费 `/a1_nav/obstacle_cloud` 这个规范入口。传感器或楼层映射后端变化时，通过 launch
重映射接入，不在 costmap 配置中散落多套私有话题。

## 对外接口

### 输入

| 接口 | 类型 | 说明 |
|---|---|---|
| TF `odom -> base` | tf2 | 机器人位姿；dev 期真值替身，正式版归 `a1_localization` |
| `odom_topic` | `nav_msgs/Odometry` | DWA 当前速度，dev 默认 `/Odometry_gazebo` |
| `/a1_nav/obstacle_cloud` | `sensor_msgs/PointCloud2` | 统一障碍点云 |
| `/move_base_simple/goal` | `geometry_msgs/PoseStamped` | RViz “2D Nav Goal” |
| `/move_base` | `move_base_msgs/MoveBaseAction` | 任务模块程序化调用入口 |

### 输出

| 接口 | 类型 | 说明 |
|---|---|---|
| `/cmd_vel_nav` | `geometry_msgs/Twist` | 导航候选速度；不得直接占用 `/cmd_vel` |
| `/move_base/GlobalPlanner/plan` | `nav_msgs/Path` | 全局路径 |
| `/move_base/DWAPlannerROS/local_plan` | `nav_msgs/Path` | 局部轨迹 |
| `/move_base/{global,local}_costmap/costmap` | `nav_msgs/OccupancyGrid` | 代价地图 |
| `/move_base` action result/feedback | 标准 action | 成功、失败、取消和进度 |

标准 ROS 接口能够完整表达的内容直接复用。未来若增加项目自定义导航状态、楼层任务或 epoch
语义，统一在 `a1_navigation_interfaces` 中定义并做跨模块评审。

## 开发演示启动

终端 1 启动仿真和真实 Livox：

```bash
GUI=true \
ENABLE_SENSOR_DATA=0 \
ENABLE_LIVOX=1 \
ENABLE_LIVOX_IMU=0 \
ENABLE_REALSENSE=0 \
ENABLE_POINTCLOUD_CONVERTER=0 \
ENABLE_REFEREE_ODOM=1 \
PUBLISH_REFEREE_TF=1 \
./auto.sh
```

等待控制器反馈就绪后按：

```text
2  -> fixed stand
5  -> State_move_base
```

必须看到：

```text
Switched from fixed stand to move_base
```

终端 2 启动导航、cmd_mux 和 RViz：

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch a1_navigation_tests nav_dev.launch \
  obstacle_source:=livox \
  use_cmd_mux:=true \
  use_rviz:=true
```

RViz 已按远程桌面降负载：10 FPS、默认关闭原始 2.4 万点 `/scan` 和 20 m 全局 costmap，
保留过滤点云、局部 costmap、路径和机器人模型。需要排查时再手动打开重显示项。

只启动业务包：

```bash
roslaunch a1_navigation navigation.launch
```

主要参数：

| 参数 | 默认值 | 说明 |
|---|---|---|
| `obstacle_cloud_topic` | `/a1_nav/obstacle_cloud` | 障碍输入 |
| `cmd_vel_topic` | `/cmd_vel_nav` | 导航速度输出 |
| `odom_topic` | `/Odometry_gazebo` | DWA 速度输入，正式版必须替换 |

### 累计楼层地图模式

当 `/a1/floor_mapping/status` 为 `MAPPING` 且 map/cloud 均有效时，可启动：

```bash
roslaunch a1_navigation navigation_floor_mapping.launch
```

该入口保持标准 `/move_base` Action 和 `/cmd_vel_nav` 输出不变，但：

- global costmap 使用 `/a1/floor_mapping/map` 的 static layer，不再滚动；
- GlobalPlanner 禁止路径穿越未知区，frontier 目标必须位于已知自由区；
- marking 使用 `/a1/floor_mapping/marking_cloud` 中已按相对楼面高度
  分类的障碍点；clearing 使用兼容的地面+障碍
  `/a1/floor_mapping/obstacle_cloud`，两者保留同一真实 Livox
  传感器原点和时间戳；
- costmap 不再用绝对 `odom.z` 二次判定 marking 高度，因此入口地面高度
  尚在收敛或未来楼层高度变化时，不会把支撑面本身整片标成障碍；
- map 的 generation/session 有效性由 `a1_exploration` 强制门控；move_base
  不应脱离上层健康门独自接收自动探索目标。

原 `navigation.launch` 的已验证无地图滚动窗口模式仍是默认，未被替换。

## 速度与安全链

`State_move_base` 复用 Unitree 的 `State_Trotting` 经典步态，不使用当前无法有效行走的 RL
模式 4/6。导航限制为：

```text
vx: [-0.15, 0.40] m/s
vy: [-0.25, 0.25] m/s
wz: [-0.60, 0.60] rad/s
```

`State_move_base` 激活期间以 10Hz 发布 `/a1/controller_ready=True`，退出前发布 `False`；
`a1_cmd_mux` 的 guard 再施加系统硬上限、加速度限制、来源超时、全局超时和 ready 门控。
`/cmd_vel_guard` 是 `/cmd_vel` 的唯一发布者。

恢复行为只清空动态障碍层，不执行 `rotate_recovery`。旧旋转恢复会在 action 取消后继续输出角速度，
不满足“取消立即停止”和四足窄处安全要求。

## 本轮修复的底层问题

真实 Livox 将仿真实时倍率降到约 0.25～0.30，暴露了多项原有控制缺陷：

1. `MOVE_BASE` 在 CMake 中被硬编码关闭，现改为默认开启的构建选项。
2. Unitree 仿真控制循环原来只按墙钟运行；低实时倍率下会在一个仿真周期内重复控制。
   仿真路径现额外跟随 ROS `/clock`，真机调度不变。
3. `State_move_base` 的 `_vx/_vy/_wz` 原来未初始化。现于构造和每次进入状态时清零，
   先处理最新消息，拒绝非有限速度，并发布控制循环 ready 心跳。
4. A1 髋关节逆运动学对工作空间边界缺少根号定义域保护，瞬时越界会生成 `q=NaN`。
   现将负根号输入投影到最近可达边界。
5. Gazebo IO 在发布 12 路电机命令前增加非有限值阻断和阻尼回退，防止单帧坏值污染 ODE。
6. `/Odometry_gazebo` 的 twist 原来错误地使用 world/odom 分量；现按
   `child_frame_id=base` 转到 base frame。
7. Unitree 内部 estimator 的 `odom -> base` 默认关闭；正式定位和 dev 真值 TF 都保持唯一发布者。
8. Gazebo 瞬时 base twist 含摆腿高频角速度，曾使 DWA 的动态减速窗口采样出超过
   `max_vel_theta` 的命令。`nav_dev.launch` 单独增大 dev 真值链的角减速度窗口；
   正式导航参数和 guard 硬限制不变。

这些是共享 Unitree/仿真层改动，合入前应由 localization、仿真和导航负责人共同 review。

## 2026-07-27 验收结果

### 假障碍

- 假墙位于起点和目标之间，只在侧面留缺口；
- action `SUCCEEDED`，耗时约 10.3 秒墙钟；
- 轨迹横向偏移约 0.67 m 穿过缺口；
- 最终误差约 0.338 m；
- 到站 `/cmd_vel=0`。

记录：`/tmp/a1_navigation_fake_obstacle.bag`。

### 真实 Livox 楼内绕障

第一目标 `(0.0,-1.7)` 位于 Livox 代价地图的致命障碍中，`move_base` 正确拒绝规划，全程速度为零。

第二目标要求绕过右侧墙体。最终修复后回归结果：

| 指标 | 结果 |
|---|---|
| 起点 | `(0.005, -3.231)` |
| 目标 | `(2.800, -1.500)` |
| action | `SUCCEEDED` |
| 起终点位移 | `3.133 m` |
| 最终目标误差 | `0.295 m`，小于 `0.35 m` 容差 |
| `/cmd_vel_nav` 峰值 | `vx=0.40, vy=0.25, wz=0.60` |
| guard 后 `/cmd_vel` 峰值 | `vx=0.40, vy=0.25, wz=0.60` |
| 到站输出 | `/cmd_vel=(0,0,0)` |
| Livox 过滤点云 | 本轮约 10,695 点/帧，10 Hz |
| 控制器门控 | fixed stand 为 `not_ready`；mode 5 为 10Hz `True`；退出发布 `False` |
| `/cmd_vel` 发布者 | 仅 `/cmd_vel_guard` |

较早的完整记录 `/tmp/a1_navigation_livox_e2e.bag`（约 136.5 MB、101 秒仿真时间）
保存了真值轨迹、障碍云、路径、action 状态和三段速度链。该记录曾暴露
`/cmd_vel_nav` 的 `0.960 rad/s` 峰值，并用于定位上述 DWA 动态窗口问题；最终回归已将峰值
限制到 `0.600 rad/s`。bag 是运行产物，不应提交 Git。

## 已知限制与下一步

- 当前成功仍依赖 Gazebo 真值 TF 和 twist，不能声称 FAST-LIO2 已与导航正式合体。
- `a1_localization/odom` 暂无可信 twist；这是 localization 与 navigation 的明确集成阻塞项。
- 当前无持久二维楼层地图，只能在滚动窗口内规划。
- 真实 Livox 全采样下实时倍率较低；演示应服务器本地录 bag，再离线回放 RViz，避免远程桌面卡顿
  干扰控制过程。
- 绕障过程中 DWA 偶发 “failed to produce path” 后恢复，最终能到达但轨迹还不够平滑，需要基于
  rosbag 优化 footprint、膨胀半径、采样数和评分权重。
- dev 真值 twist 的高频摆腿分量由 `nav_dev.launch` 专用参数规避；正式接入定位时仍要求
  `a1_localization/odom` 提供可信、适当滤波且符合 `child_frame_id` 语义的 twist。
- 通用导航到达精度约 0.35 m。进电梯、过门和精确停靠归 `a1_building_behavior`。
- 目前使用标准 `MoveBaseAction` 反馈，没有额外自定义导航状态；确需新增时必须进入
  `a1_navigation_interfaces`，不能在业务包私建共享协议。

## 依赖

容器镜像应预装：

```bash
apt-get update
apt-get install -y ros-noetic-navigation ros-noetic-twist-mux
```

依赖需要烤进团队镜像，不能只依赖某个成员容器里的临时 `apt install`。
