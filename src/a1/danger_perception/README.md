# a1_danger_perception 使用说明

本包是危险源识别与定位的第一版实现，使用 RealSense RGB + 深度图和 OpenCV 检测红色球体。

## 编译

在工作空间根目录执行：

```bash
cd /workspace/SimEnv
source /opt/ros/noetic/setup.bash
catkin_make -j
source devel/setup.bash
```

## 启动仿真

建议先只开 RealSense，方便调试：

```bash
SEED=1 DANGER_COUNT=1 DISTRACTOR_COUNT=0 ENABLE_SENSOR_DATA=0 ENABLE_REALSENSE=1 GUI=true ./auto.sh
```

## 启动识别节点

另开终端：

```bash
source /opt/ros/noetic/setup.bash
source /workspace/SimEnv/devel/setup.bash
roslaunch a1_danger_perception danger_perception.launch
```

如果暂时没有定位/TF，可先关闭 TF 转换，只看相机坐标下的检测：

```bash
roslaunch a1_danger_perception danger_perception.launch enable_tf:=false
```

## 查看图像和结果

看 RealSense 原始图：

```bash
rosrun image_view image_view image:=/real_sense/rgb/image_raw
```

看算法画框后的图：

```bash
rosrun image_view image_view image:=/danger_perception/debug/detections_image
```

看红色 mask：

```bash
rosrun image_view image_view image:=/danger_perception/debug/mask_red
```

看结构化检测结果：

```bash
rostopic echo /danger_perception/detections
```

## 说明

默认 `target_frame` 是 `map`。当前仿真中 `map` 通常与 Gazebo `world` 对齐；如果后续定位模块直接发布 `world` 坐标系，把启动参数改成：

```bash
roslaunch a1_danger_perception danger_perception.launch target_frame:=world
```

`/Odometry_gazebo` 和 `/ground_truth/*` 只能用于本地调试，不应作为正式算法输入。
