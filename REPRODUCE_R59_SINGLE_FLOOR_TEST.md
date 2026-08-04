# 第 59 轮单层探索测试复现手册

## 1. 目标与预期结果

该流程复现 2026-07-31 完成的第 59 轮 GUI 验收：

- A1 从室外出生点出发；
- 通过入口台阶和大门；
- 跳过入口左右分支；
- 探索主走廊两组共四个房间；
- 从走廊深处返回室内入口锚点；
- 在最后“入口锚点 → 室外出生点”阶段复现当前返航缺陷。

复现必须使用固定场景种子：

```text
100276380
```

若不固定 `SEED`，建筑会重新随机生成，不能视为同一轮复现。

## 2. 前置条件

- 服务器仓库：`/home/xiaoyi-dev/simenv/SimEnv`
- 仿真容器：`a55579836396`
- 容器内工作区：`/workspace/SimEnv`
- ROS Noetic。
- Gazebo GUI 使用服务器显示 `DISPLAY=:1`；开发成员需先确认 VNC/远程桌面中的
  X server 可用。
- 不要同时运行其他 Gazebo、手动遥控或 `junior_ctrl`。
- 当前版本需先提交或由交接者提供对应分支；仅拉取当时的 `main`
  `b704ff2d...` 不包含本阶段修改。

以下命令均在服务器执行。

## 3. 获取代码并构建

```bash
cd /home/xiaoyi-dev/simenv/SimEnv
git fetch origin
git checkout <包含本阶段修改的分支或提交>
git status --short
```

进入容器并构建：

```bash
docker exec -it a55579836396 bash
cd /workspace/SimEnv
source /opt/ros/noetic/setup.bash
catkin_make
source /workspace/SimEnv/devel/setup.bash
```

建议先执行回归测试：

```bash
cd /workspace/SimEnv
source /opt/ros/noetic/setup.bash
source /workspace/SimEnv/devel/setup.bash
catkin_make run_tests
catkin_test_results
```

## 4. 启动 GUI 仿真

打开终端 A：

```bash
docker exec -it a55579836396 bash
cd /workspace/SimEnv

export DISPLAY=:1
export GUI=true
export SEED=100276380

export ACCEPTANCE_MANAGED_PHYSICS=1
export AUTO_UNPAUSE=0
export ROBOT_Z=0.34

export ENABLE_SENSOR_DATA=false
export ENABLE_LIVOX=true
export ENABLE_LIVOX_IMU=false
export ENABLE_REALSENSE=false
export ENABLE_FRONT_CAMERA=false

# 这些真值话题只供独立验收 oracle 使用。
# 严禁发布真值 TF 或把真值里程计接入定位/导航。
export ENABLE_REFEREE_ODOM=true
export PUBLISH_REFEREE_TF=false
export ENABLE_GROUND_TRUTH=true
export POINTCLOUD_USE_GROUND_TRUTH_ODOM=false

export ENABLE_FOOT_CONTACT_SENSOR=true
export ENABLE_FOOT_FORCE_VISUAL=false
export ENABLE_JOY_NODE=false
export ENABLE_POINTCLOUD_CONVERTER=false

mkdir -p /workspace/SimEnv/results/single_floor/r59_reproduction
./auto.sh \
  > /workspace/SimEnv/results/single_floor/r59_reproduction/sim.log 2>&1
```

`auto.sh` 在前台运行是正常行为。不要手工解暂停 Gazebo；验收程序负责在
rosbag、控制器和传感器检查通过后进行唯一一次解暂停。

如果 GUI 启动即退出，先检查：

```bash
echo "$DISPLAY"
xdpyinfo -display :1 >/dev/null
tail -n 100 /workspace/SimEnv/results/single_floor/r59_reproduction/sim.log
```

## 5. 启动第 59 轮验收

确认 Gazebo GUI 已出现后，打开终端 B：

```bash
docker exec -it a55579836396 bash
source /opt/ros/noetic/setup.bash
source /workspace/SimEnv/devel/setup.bash

mkdir -p /workspace/SimEnv/results/single_floor/r59_reproduction

roslaunch a1_navigation_tests single_floor_gazebo_acceptance.launch \
  run_id:=r59_reproduction \
  roi_depth:=35.5 \
  roi_half_width:=9.5 \
  minimum_frontier_successes:=2 \
  action_timeout_sim:=300 \
  action_wall_timeout:=1200 \
  output:=/workspace/SimEnv/results/single_floor/r59_reproduction/bounded.json \
  bag_path:=/workspace/SimEnv/results/single_floor/r59_reproduction/bounded.bag \
  2>&1 | tee /workspace/SimEnv/results/single_floor/r59_reproduction/acceptance.log
```

必须全程保持终端 A、终端 B 和 GUI 打开。当前机器的实时倍率可能约为
`0.1–0.3`，因此 `300 s` 仿真时间使用 `1200 s` 墙钟验收窗口。

## 6. GUI 观察重点

预期依次出现：

1. 机器人落地并在约 4 秒仿真时间内稳定站立；
2. 主入口打开；
3. 机器人通过室外台阶和大门；
4. 经过入口左右大型侧向开口但不进入；
5. 到达第一组房门，分别完成两个房间；
6. 沿主走廊继续深入；
7. 到达第二组房门，分别完成两个房间；
8. 继续向走廊末端推进并宣布探索完成；
9. 掉头，沿主走廊返回室内入口锚点；
10. 开始最终出门时错误地向前偏离，随后 DWA 停止输出有效速度。

GUI/RViz 建议同时显示：

- 第一人称相机（如当前传感器配置启用）；
- Gazebo 第三人称跟随；
- `/a1/floor_mapping/map`；
- `/a1/exploration/trajectory`；
- `/a1/exploration/frontiers`；
- `/a1/exploration/selected_target`；
- `/move_base/GlobalPlanner/plan`；
- `/move_base/DWAPlannerROS/local_plan`；
- 探索状态和日志。

## 7. 第 59 轮判据与关键日志

成功探索的关键日志应包含：

```text
room branch scanned, exited, and marked complete
no eligible frontier after completed rooms; advancing through map-verified main-corridor free space
EXPLORATION_DONE
RETURNING: returning to indoor entry anchor
Goal reached
RETURNING: returning to outdoor start pose
```

参考时间线：

```text
sim 153.27  EXPLORATION_DONE
sim 153.27  开始返回室内入口锚点
sim 194.78  室内入口锚点 Goal reached
sim 194.82  开始返回室外出生点
sim 197.52  DWA planner failed to produce path
sim 207.32  move_base 因找不到有效控制而 ABORTED
```

参考指标：

```text
coverage ratio        0.7609
trajectory distance   约 96.92 m
maximum roll          约 0.132 rad
maximum pitch         约 0.257 rad
```

## 8. 复现后收集信息

```bash
cd /workspace/SimEnv/results/single_floor/r59_reproduction
grep -E \
  "EXPLORATION_DONE|RETURNING|Goal reached|DWA planner failed|bounded backout|FAILED" \
  acceptance.log

source /opt/ros/noetic/setup.bash
rosbag info bounded.bag
```

重点比较以下话题在 `sim=194–208 s` 的数据：

```text
/a1/localization/odom
/cmd_vel_nav
/cmd_vel_muxed
/cmd_vel
/move_base/goal
/move_base/result
/move_base/DWAPlannerROS/local_plan
/move_base/GlobalPlanner/plan
/rosout_agg
```

## 9. 当前返航问题的代码入口

主要代码：

```text
src/a1/exploration/scripts/frontier_explorer_node.py
  execute_return()
  apply_entry_speed_limit()
  restore_entry_speed_limit()
  navigate()
  bounded_backout()

src/a1/exploration/config/exploration.yaml
  entry.speed_limit
  return
  navigation.backout
```

当前问题不是 FAST-LIO 丢失、State_RL 摔倒或全局规划中断。室内入口锚点到达后，
`execute_return()` 对室外阶段调用 `apply_entry_speed_limit()`，把 DWA 限制为：

```text
min_vel_x       1.25 m/s
max_vel_x       1.40 m/s
max_vel_theta   0.01 rad/s
```

机器人此时需要大角度掉头，因此该参数组合没有可行局部控制，并把机器人推向错误方向。
修复时不要改变已经验证通过的房间探索主流程；应把最终出门拆成“普通参数掉头对准”
和“对准后的短距离门槛直行”两个阶段。

## 10. 正常结束与清理

先在终端 B 使用 `Ctrl-C` 结束验收，再在终端 A 使用 `Ctrl-C` 结束仿真。
确认没有残留：

```bash
docker exec a55579836396 \
  ps -eo pid,ppid,stat,etime,cmd
```

不要把 `results/`、`logs/`、`generated_building/` 或 rosbag 提交到 Git。
