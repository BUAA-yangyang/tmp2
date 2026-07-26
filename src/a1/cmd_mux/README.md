# a1_cmd_mux

`a1_cmd_mux` 是 A1 上层速度命令到四足底层的唯一出口。它不生成导航或
特殊行为，只负责仲裁与安全保护：

```text
/cmd_vel_emergency (100) ─┐
/cmd_vel_teleop      (90) ─┤
/cmd_vel_behavior    (50) ─┼─ twist_mux ─ /cmd_vel_muxed ─ cmd_vel_guard ─ /cmd_vel
/cmd_vel_nav         (10) ─┘
```

这对应 `MODULES.md` 中的模块边界：导航只能发布 `/cmd_vel_nav`，
特殊行为只能发布 `/cmd_vel_behavior`，其他上层模块不得直接发布 `/cmd_vel`。

## 为什么是两级结构

`twist_mux` 使用固定优先级和每个输入独立的超时完成多源仲裁。所有源都沉默时，
它不会持续发布零速度；它也不负责 A1 的速度和加速度约束。

`cmd_vel_guard` 因此负责：

- 将 `vx / vy / wz` 限制到 `0.50 / 0.30 / 0.80`；
- 将对应加速度限制到 `3.0 / 3.0 / 4.0`；
- 上游断流后减速到零，并以 50Hz 持续发布零；
- 拒绝 NaN 和 Inf；
- 急停或 safety lock 时绕过减速度限制，下一发布周期立即归零；
- 可选的控制器 ready 心跳门控；
- 仿真 `/clock` 回拨时清空旧指令并立即归零。
- 正常 Ctrl-C/节点关闭前连续发送零速度，单节点异常退出时由 roslaunch 重启。

普通输入的 `twist_mux` 超时为 0.5s，guard 总输入超时为 0.7s。后者必须稍长，
否则高优先级源退出时，guard 会先误判断流，来不及让 mux 平滑回落到仍在发布的
低优先级源。当前最大线速度下，全部输入断开后最坏约 0.87s 归零。

## 话题契约

| 话题 | 类型 | 方向 | 语义 |
|---|---|---|---|
| `/cmd_vel_nav` | `geometry_msgs/Twist` | 输入 | 单楼层导航，最低优先级 |
| `/cmd_vel_behavior` | `geometry_msgs/Twist` | 输入 | 门、电梯、楼梯等特殊行为 |
| `/cmd_vel_teleop` | `geometry_msgs/Twist` | 输入 | 人工接管 |
| `/cmd_vel_emergency` | `geometry_msgs/Twist` | 输入 | 急停心跳；消息内容忽略，持续发布即保持急停 |
| `/a1_cmd_mux/safety_lock` | `std_msgs/Bool` | 输入 | `True` 立即锁零，明确收到 `False` 才释放 |
| `/a1/controller_ready` | `std_msgs/Bool` | 可选输入 | 控制器 ready 心跳 |
| `/a1/cmd_mux/status` | `a1_navigation_interfaces/CmdMuxStatus` | 输出 | 当前控制源、急停、输出使能、实际速度和源年龄 |
| `/cmd_vel` | `geometry_msgs/Twist` | 输出 | 唯一底层速度出口 |

急停发布频率必须高于 2Hz；停发 0.5s 后自动释放。即使急停发布者误发了非零
Twist，guard 仍然只会输出零。`safety_lock=True` 则不会因发布者退出自动释放，
避免故障进程死亡后意外恢复运动。

ready 门控接口已经实现，但当前 `unitree_guide` 没有提供可靠的 FSM 状态话题，
默认 `require_ready=false`。不能用“存在 `/cmd_vel` 订阅者”代替 ready，因为
`State_RL` 对象在未进入 RL `/cmd_vel` 模式时也可能已经建立订阅。未来应由底层
或 `mission_manager/bringup` 提供真实、持续的 Bool 心跳，再启用：

```bash
roslaunch a1_cmd_mux cmd_mux.launch \
  require_ready:=true \
  ready_topic:=/实际的控制器就绪话题
```

状态话题复用团队共享的 `CmdMuxStatus.msg`，不在本包重复定义接口。普通激活状态
会报告 `SOURCE_NAVIGATION / BEHAVIOR / TELEOP`，急停报告 `SOURCE_ESTOP`；
锁定、未 ready、断流或时钟回拨时报告 `SOURCE_NONE`、`output_enabled=false`，
且 `active_source_age_s=-1.0`。

## 安装与启动

当前使用 Ubuntu 20.04 / ROS Noetic：

```bash
sudo apt install ros-noetic-twist-mux
catkin_make --pkg a1_cmd_mux
source devel/setup.bash
roslaunch a1_cmd_mux cmd_mux.launch
```

`ros-noetic-twist-mux` 必须写入团队镜像的 Dockerfile 或依赖安装脚本，不能只在
个人容器里手工安装。

## 验收

以下测试会主动发布速度，必须在独立 ROS master 中运行，或确认机器人不在
RL `/cmd_vel` 模式：

```bash
roslaunch a1_navigation_tests cmd_mux_acceptance.launch
```

测试覆盖 `MODULES.md` 的五项标准，并额外检查 ready 门控、急停延迟、
safety lock、NaN/Inf、高低优先级切换瞬态和 guard 自动重启。系统联调时还应执行：

```bash
rostopic info /cmd_vel
```

确认 `Publishers` 中只有 `/cmd_vel_guard`。该检查是运行时系统级约束，单个包
无法阻止其他团队以后错误地新增 `/cmd_vel` 发布者。

有序关闭时 guard 会尽力在 ROS 连接断开前连续发布五次零速度；默认 launch 还会
在单个 `twist_mux` 或 guard 进程异常后自动重启。但 `SIGKILL`、容器崩溃或整机
掉电无法由同一用户态节点自救，最终仍应在 Unitree 底层增加独立 watchdog。
