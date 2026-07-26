#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Livox /scan -> 导航可用的障碍点云。

为什么必须有这一层：
  Gazebo 的 Livox 插件把"没有回波"的射线也照发不误，坐标写成 (0,0,0)。
  实测一帧 24000 个点里有 11463 个(47.8%)是这种 (0,0,0) 的空回波。
  它们在传感器坐标系原点，也就是紧贴机器人身上 —— 直接喂给 costmap 会在
  机器人脚下糊出一坨致命障碍，而且这坨障碍跟着机器人走，导航永远在"我被围住了"
  的状态里出不来（实测现象：狗原地不动、一直发 vx=-0.15 想后退、
  最后触发 rotate recovery 并报 "potential collision, Cost: -1.00"）。

  官方那个 pointcloud2livox.py 里的 laser_blind 参数就是干这个的，但它只作用在
  /livox/Pointcloud2 那条链路上；而那条链路用了 Gazebo 真值做坐标变换，比赛不能用。
  所以走原始 /scan 的话，这层盲区过滤得自己做。

顺带把 sensor_msgs/PointCloud 转成 PointCloud2，下游 costmap 统一按 PointCloud2 配置。

注：这属于传感器预处理，长期更适合放在 a1_floor_mapping（它负责三维到二维投影）。
    在那个模块可用之前，先由导航自己兜底，保证 a1_navigation 能独立跑起来。
"""

import numpy as np
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud, PointCloud2
from std_msgs.msg import Header


class ScanToObstacleCloud(object):
    def __init__(self):
        self.min_range = rospy.get_param("~min_range", 0.6)
        self.max_range = rospy.get_param("~max_range", 20.0)
        out_topic = rospy.get_param("~output_topic", "/a1_nav/obstacle_cloud")
        in_topic = rospy.get_param("~input_topic", "/scan")

        self.pub = rospy.Publisher(out_topic, PointCloud2, queue_size=1)
        self.sub = rospy.Subscriber(in_topic, PointCloud, self.cb, queue_size=1)

        self.n_in = 0
        self.n_out = 0
        self.frames = 0
        rospy.Timer(rospy.Duration(10.0), self.report)

        rospy.loginfo("scan_to_obstacle_cloud: %s -> %s, 盲区 %.2fm, 量程 %.1fm",
                      in_topic, out_topic, self.min_range, self.max_range)

    def cb(self, msg):
        n = len(msg.points)
        if n == 0:
            return
        # 用 fromiter 比逐点 append 快很多；24000 点在 10Hz 下毫无压力
        x = np.fromiter((p.x for p in msg.points), dtype=np.float32, count=n)
        y = np.fromiter((p.y for p in msg.points), dtype=np.float32, count=n)
        z = np.fromiter((p.z for p in msg.points), dtype=np.float32, count=n)

        d2 = x * x + y * y + z * z
        keep = (d2 >= self.min_range ** 2) & (d2 <= self.max_range ** 2)
        keep &= np.isfinite(d2)

        pts = np.column_stack((x[keep], y[keep], z[keep]))

        header = Header()
        # 时间戳沿用原消息，不要用 now()：costmap 要拿它去查 TF，
        # 用 now() 在 RTF<1 的仿真里会查到"未来"的变换而丢帧
        header.stamp = msg.header.stamp
        header.frame_id = msg.header.frame_id
        self.pub.publish(pc2.create_cloud_xyz32(header, pts.tolist()))

        self.n_in += n
        self.n_out += int(keep.sum())
        self.frames += 1

    def report(self, _):
        if self.frames == 0:
            rospy.logwarn_throttle(30.0, "scan_to_obstacle_cloud: 还没收到 /scan")
            return
        rospy.loginfo("scan_to_obstacle_cloud: %d 帧, 平均每帧 %d -> %d 点 (滤掉 %.1f%%)",
                      self.frames, self.n_in // self.frames, self.n_out // self.frames,
                      100.0 * (1.0 - float(self.n_out) / max(self.n_in, 1)))
        self.n_in = self.n_out = self.frames = 0


if __name__ == "__main__":
    rospy.init_node("scan_to_obstacle_cloud")
    ScanToObstacleCloud()
    rospy.spin()
