# A1 单层探索 WIP 协作交接

> 本文记录可复现的开发状态，不代表最终赛事验收通过。

## 当前能力

- 生产运动链保持为：
  `ExploreFloor -> MoveBaseAction -> /cmd_vel_nav -> cmd_mux/guard -> /cmd_vel`。
- 已实现公开主入口开门、Livox OccupancyGrid 通道确认、入口与
  `RECORD_START` 分离、室内 ROI frontier、失败冷却、自动返航、
  final-zero 和全足支撑安全停机。
- 算法不读取 world、Gazebo model/link state、layout metadata 或固定场景坐标。
- 当前定位仍使用明确隔离的 dev-only 真值 TF；尚未宣称 FAST-LIO2 正式接入。

## 已验证但未完成的 Gazebo 结果

`indoor_start_dev_only_20260729_fixed_03` 从室内平地启动，用于把探索能力与
入口 8 cm 平台运动问题解耦：

- 运行约 367 仿真秒；
- 发送 9 个 frontier 目标；
- 前两个目标真实成功，后续失败按 1/2 重试与 cooldown 处理，没有被误判为完成；
- 覆盖率诊断约 51.2%；
- `MoveBase -> /cmd_vel_nav -> cmd_mux -> /cmd_vel` 链路正常；
- 第 9 个目标发送后 roll 超过 10°，验收器取消 action；
- 随后获得四足支撑并切回 fixed-stand，但 final verifier 因
  `/cmd_vel_nav` 样本过期而保持 fail-closed。

因此本次运行证明了室内多 frontier 导航的部分能力，但没有完成
“ROI 耗尽 -> 返航 -> final-zero”的完整验收。

同服务器工件目录：

```text
/home/xiaoyi-dev/simenv/indoor_start_milestone_20260729/results/indoor_start_dev_only_20260729_fixed_03
```

## 两个独立阻塞

1. **官方入口平台运动**
   - 生成器固定创建 4.5 m x 2.4 m x 0.08 m 的 `entrance_apron`。
   - v7-v11 的失稳集中在上下平台的落足冲击。
   - v12 将导航步态周期改为 0.90 s 后，在到达平台前仅运动约 0.129 m
     就触发 roll 10.89°，属于疑似控制回归。
   - 当前 Unitree 提交保留这些实验供审查和二分，不应视为已完成修复。

2. **室内长时间导航姿态超限**
   - `fixed_03` 在长时间稳定运动后，于第 9 个目标附近首次超过 10°。
   - 需要从 sealed bag 对齐 IMU、四足力、12 关节命令/状态、恢复行为及
     `/cmd_vel_nav`，区分底层周期性失稳与局部规划转向触发。

## 复现入口

室内启动验收仅供开发诊断，不得作为官方起点验收：

```bash
rosrun a1_navigation_tests run_indoor_start_once.sh \
  /workspace/SimEnv/results/indoor_start_YYYYMMDD_HHMMSS
```

运行器要求全新目录，并在解暂停、发送 ExploreFloor/MoveBase 目标之前开始录包。
生产 launch 默认不跳过开门或入口阶段。

## 建议协作拆分

- 一名同学只审查 exploration/frontier/FSM、MoveBase 失败与终点朝向；
- 一名同学只审查 Unitree `State_move_base`、步态时基、落足冲击和安全停机；
- 一名同学复核 floor_mapping 与最新 main 的 stable wall/doorway 改动合并；
- 不要通过放宽 10° 姿态门限、gyro 门限或直接发布 `/cmd_vel` 获得假通过。
