# a1_navigation

单楼层二维导航模块。封装 `move_base`，接收导航目标 → 全局规划 + 局部避障 → 速度输出到 `/cmd_vel_nav`。

> **当前状态：里程碑1 基线版**。move_base 整条链路已跑通并实测通过，但**尚未接入任何真实传感器**，
> 也**未在楼栋内部测试过**。参数是针对合成障碍在室外空地调出来的，接真实传感器后需要重调。
> 详见文末"已知限制"。

## 接口

### 输入

| 话题 / 资源 | 类型 | 说明 |
|---|---|---|
| TF `odom` → `base` | tf2 | 定位。dev 期由仿真自带 `state_from_gazebo` 提供（`ENABLE_REFEREE_ODOM=1` 默认开，派生自 Gazebo 真值），**正式版须换成 `a1_localization` 的 FAST-LIO2** |
| `/Odometry_gazebo` | `nav_msgs/Odometry` | DWA 读当前速度用。可用 launch 参数 `odom_topic` 改 |
| `/a1_nav/obstacle_cloud` | `sensor_msgs/PointCloud2` | 障碍点云。本模块只认这一个规范输入，由 launch 的 `obstacle_cloud_topic` 决定实际接谁 |
| `/move_base_simple/goal` | `geometry_msgs/PoseStamped` | 单个目标点（RViz 的 2D Nav Goal 按钮） |
| `/move_base` action | `move_base_msgs/MoveBaseAction` | 程序化调用入口，带成功/失败状态，供 `a1_mission_manager` 用 |

### 输出

| 话题 | 类型 | 说明 |
|---|---|---|
| `/cmd_vel_nav` | `geometry_msgs/Twist` | 10 Hz。**本模块不发布 `/cmd_vel`**，最终速度只能由 `a1_cmd_mux` 仲裁后发出 |
| `/move_base/GlobalPlanner/plan` | `nav_msgs/Path` | 全局路径（可视化/调试） |
| `/move_base/{global,local}_costmap/costmap` | `nav_msgs/OccupancyGrid` | 代价地图（可视化/调试） |

## 启动

```bash
roslaunch a1_navigation navigation.launch
```

主要参数：

| 参数 | 默认 | 说明 |
|---|---|---|
| `obstacle_cloud_topic` | `/a1_nav/obstacle_cloud` | 障碍源；指向别的话题时自动起 relay 转进来 |
| `cmd_vel_topic` | `/cmd_vel_nav` | 速度输出 |
| `odom_topic` | `/Odometry_gazebo` | 里程计 |

联调（含假障碍 + RViz + dev 期的 `/cmd_vel_nav`→`/cmd_vel` 桥接）见 `a1_navigation_tests`：

```bash
roslaunch a1_navigation_tests nav_dev.launch
```

## 运行依赖

⚠️ **`simenv:noetic-cu128` 镜像里没有 navigation 相关包**，需要先装（长期应烤进 Dockerfile）：

```bash
apt-get update && apt-get install -y ros-noetic-navigation
```

## 设计要点

**无地图滚动窗口。** 两个 costmap 都是 `global_frame: odom` + `static_map: false` + `rolling_window: true`，
即不加载静态地图，代价地图是跟着机器人滚动的窗口，只由实时传感器填充。好处是不依赖 `a1_floor_mapping`
就能单独工作。后期接楼层地图时改成 `map` 帧 + `static_map: true` 并加 `static_layer`。

配套地，`GlobalPlanner` 必须 `allow_unknown: true` —— 滚动窗口之外全是"未知"，不允许穿越未知的话
一步都规划不出去。

**障碍输入单一入口。** 只订阅 `/a1_nav/obstacle_cloud`，换传感器后端（假障碍 / RealSense / Livox /
未来 `a1_floor_mapping` 的投影结果）只改 launch 参数，配置文件不动。

**速度上限按 A1 的 RL 策略范围设定。** `vx≤0.40 vy≤0.25 wz≤0.70`。策略训练用的键盘 scale 是
`vx 0.6 / vy 0.35 / wz 0.9`（见 `State_RL_test.h`），超出范围策略没见过会步态发散。
官方 `unitree_move_base` 的配置是 Go1 + MPC 步态的（`max_vel_x 0.5 / vy 0.5 / vtheta 1.2`，
且 `min_vel_x: 0.3` 禁止慢速），**不能照抄**。

## 三个踩过的坑（改配置前务必读）

1. **`DWAPlannerROS/odom_topic` 必须显式设**。默认值是 `odom`，本仿真没有这个话题。
   漏了极难查：DWA 靠它读当前速度算可达速度窗口，读不到就永远以为当前速度=0，
   可达窗口只剩 `±acc_lim×dt`，`/cmd_vel_nav` 被永久钉在 0.1 m/s、狗原地不动，
   接着被判振荡去做恢复旋转，看起来像"规划器坏了"。

2. **A1 的 RL 策略有速度死区**。实测 0.25 m/s 正常走，0.18 m/s 及以下原地踏步、净位移≈0。
   DWA 临近目标会自然降速，一旦掉进死区就卡死。对策：`min_vel_trans: 0.24`（抬到死区之上，
   变成"要么以能走动的速度走、要么停"）+ `xy_goal_tolerance: 0.45`（容差必须大于最后一步的残差，
   否则永远判不了到达）。另外 `acc_lim_*` 不能按轮式车求平滑给小值，给 3.0；
   RL 本身是速度跟踪器，吃阶跃指令没问题。

3. **`RotateRecovery` 的速度参数从 `~/TrajectoryPlannerROS` 读，不是从 `~/rotate_recovery` 读**
   （源码硬编码，为向后兼容）。所以即使用 DWA 做局部规划，也必须写一份 `TrajectoryPlannerROS`
   的角速度参数，否则 `max_vel_theta` 取默认值 1.0，超过 A1 的上限 0.9。

另外：用了 `plugins:` 列表后，传感器参数必须放在 `<costmap>/obstacle_layer` 命名空间，
放在 `<costmap>/` 下会被**静默忽略**，表现为"传感器在发但 costmap 一片空白"。

## 实测结果（2026-07-25，仿真，假障碍）

场景：出生点外空地，一道假墙横在狗和目标之间，墙上留缺口，狗必须绕行穿过。

| 缺口宽度 | 结果 |
|---|---|
| 0.9 m | 通过，到达 |
| 0.8 m | 通过，到达（轨迹东偏 1.3 m 找缺口再折回） |
| 0.6 m | 通过，到达（残差 0.45 m） |

全程 `max\|vx\|=0.40 max\|vy\|=0.25 max\|wz\|=0.70`，均在 A1 限内；到达后 `/cmd_vel_nav` 恒零，
无残留指令；无恢复行为触发。参考：楼内最窄的门是电梯门 1.4 m，主入口 2.0 m。

## 已知限制 / 后续工作

- **未接真实传感器**。目前只用过 `a1_navigation_tests` 的假障碍发布器。Livox 插件当前会崩 gzserver，
  RealSense 因容器缺 GPU 渲染能力（软渲染）会拖垮实时因子导致狗摔倒 —— 两个都是环境侧问题，
  已单独报告给团队。
- **未在楼栋内部测试**。只在室外空地验证过。楼内走廊窄、有家具、且 45° 斜装的 Livox 会打大量地面点，
  高度过滤参数（`min_obstacle_height`）需要用真实点云重新验证。
- **到达精度 0.45 m**，只够"走到房间某处去看一眼"这类探索型目标。进电梯/过门这类需要精确对位的动作
  归 `a1_building_behavior`。
- **依赖真值 odom**（dev-only），须换 FAST-LIO2 后重测。
- **无模块级状态输出**。目前只有 `move_base` 原生的 action 反馈，还没有 `a1_navigation_interfaces`
  定义的导航状态话题。
- **偶发** `sensor origin out of map bounds` 警告，清除功能会短暂失效，未定位。
- 恢复行为里的原地旋转对四足在窄处有刮碰风险，接真实传感器后应重新评估是否保留。
