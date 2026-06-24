# 算法接入接口

本文说明参赛算法允许使用的控制输入、公开场景信息和传感器接口。

## 控制器状态切换

`auto.sh` 默认以前台方式启动 `junior_ctrl`。该控制器仍遵循 Unitree 原有交互流程：

- 键盘输入 `2`：站立。
- 键盘输入 `6`：切换到 RL 模式。
- RL 模式下订阅 `/cmd_vel`。

如果 `CONTROLLER_FOREGROUND=0`，`junior_ctrl` 会在后台运行，键盘状态切换通常不可用，日志写入 `logs/junior_ctrl.log`。

## 最小控制接口

| 接口 | 类型 | 说明 |
|------|------|------|
| `/cmd_vel` | `geometry_msgs/Twist` | 机器人速度指令输入 |

`/cmd_vel` 在 RL 模式下生效。

## 公开场景信息

参赛算法可以读取：

```text
generated_building/team_scene_info.json
```

该文件只包含机器人起点、公开门/电梯 ID、允许话题、允许服务和结果文件路径。`generated_building/layout_metadata.json`、`generated_building/building_config.json`、`generated_building/scene_manifest.json`、`generated_building/danger_truth.json` 和 `results/danger_truth.json` 属于裁判/调试信息，不作为正式算法输入。

## 允许状态与传感器接口

| 接口 | 类型 | 说明 |
|------|------|------|
| `/scan` | `sensor_msgs/PointCloud2` | Livox Mid-360 点云数据 |
| `/livox/Pointcloud2` | `sensor_msgs/PointCloud2` | 点云转换节点输出，开启时可用 |
| `/livox/lidar2` | `unitree_guide/CustomMsg` | Livox 风格点云消息，开启时可用 |
| `/livox/imu` | `sensor_msgs/Imu` | Livox 内置 IMU |
| `/trunk_imu` | `sensor_msgs/Imu` | 机体 IMU |
| `/real_sense/rgb/image_raw` | `sensor_msgs/Image` | RealSense RGB 图像 |
| `/real_sense/rgb/camera_info` | `sensor_msgs/CameraInfo` | RealSense RGB 相机标定 |
| `/real_sense/depth/image_raw` | `sensor_msgs/Image` | RealSense 深度图像 |
| `/real_sense/depth/camera_info` | `sensor_msgs/CameraInfo` | RealSense 深度相机标定 |
| `/real_sense/depth/points` | `sensor_msgs/PointCloud2` | 深度相机点云 |

传感器安装位姿、完整话题和坐标系见 [传感器与 ROS 话题](sensors-and-topics.md)。

## 门与电梯交互

| 接口 | 类型 | 说明 |
|------|------|------|
| `/set_door_state` | `building_generator_interfaces/SetDoorState` | 设置动态门开关状态 |
| `/call_elevator` | `building_generator_interfaces/CallElevator` | 呼叫电梯到目标楼层 |

## 裁判与调试专用接口

`/Odometry_gazebo`、`/ground_truth/base_w`、`/ground_truth/base_trunk` 和 `/ground_truth/*_foot` 是 Gazebo 真值通道，不作为正式比赛算法输入。即使本地调试时能够看到这些话题，参赛算法也不得订阅或读取。

## 结果输出约束

参赛算法应输出 `results/detected_danger.json`。不应读取 `results/danger_truth.json`。结果格式和评分方法见 [结果格式与评估方法](evaluation.md)。

## 控制周期

`junior_ctrl` 默认控制周期为 `0.002 s`，即 500 Hz：

```bash
UNITREE_CTRL_DT=0.002
```

键盘 `1` 为 passive/down，`2` 为站立，`6` 为 RL 模式。按 `2` 站立时，控制器会等待 Gazebo 12 个关节状态反馈就绪，再从当前姿态平滑过渡到站立姿态。默认站立过渡时长为 3 秒。

如机器性能较弱、GUI 下动作明显卡顿，可优先降低传感器数据负载或 Gazebo 物理负载；确需降低控制频率时可显式设置：

```bash
UNITREE_CTRL_DT=0.004 ./auto.sh
```
