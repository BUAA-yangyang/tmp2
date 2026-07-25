#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
假障碍发布器 —— 在没有可用真实传感器时，验证 move_base 整条链路。

为什么需要它：
  仿真里 Livox 插件会崩 gzserver，深度相机因容器只有软渲染(llvmpipe)会拖垮实时因子
  把狗搞趴窝。这两件事都不属于 a1_navigation 模块。为了不被卡住，先用一个"完全可控的
  障碍源"把 costmap → 全局规划 → 局部规划 → /cmd_vel_nav 这条链路跑通并调好参数，
  等真实传感器修好后直接把 obstacle_cloud_topic 指过去即可，导航配置一行都不用改。

发布内容：
  odom 坐标系下一道带缺口的墙。狗必须绕开墙、从缺口穿过去才能到达目标，
  因此能真正验证"避障"，而不只是"撞停"。

  frame_id 直接用 odom，省掉传感器外参这一层变量；真实传感器则会用自己的
  光学坐标系，由 costmap_2d 通过 TF 转换。
"""

import math

import rospy
import sensor_msgs.point_cloud2 as pc2
import tf2_ros
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header


def make_wall(x0, y0, x1, y1, z_min, z_max, spacing):
    """在 (x0,y0)-(x1,y1) 之间生成一片竖直墙面的点。"""
    points = []
    length = math.hypot(x1 - x0, y1 - y0)
    if length < 1e-6:
        return points
    n_along = max(int(length / spacing), 1)
    n_up = max(int((z_max - z_min) / spacing), 1)
    for i in range(n_along + 1):
        t = float(i) / n_along
        x = x0 + (x1 - x0) * t
        y = y0 + (y1 - y0) * t
        for j in range(n_up + 1):
            z = z_min + (z_max - z_min) * float(j) / n_up
            points.append((x, y, z))
    return points


def build_scene(wall_y, half_width, gap_center, gap_width, z_min, z_max, spacing):
    """一道 y=wall_y 的横墙，在 x=gap_center 处留一个宽 gap_width 的缺口。"""
    points = []
    gap_lo = gap_center - gap_width / 2.0
    gap_hi = gap_center + gap_width / 2.0
    # 缺口左段
    if gap_lo > -half_width:
        points += make_wall(-half_width, wall_y, gap_lo, wall_y, z_min, z_max, spacing)
    # 缺口右段
    if gap_hi < half_width:
        points += make_wall(gap_hi, wall_y, half_width, wall_y, z_min, z_max, spacing)
    return points


def main():
    rospy.init_node("fake_obstacle_publisher")

    topic = rospy.get_param("~topic", "/a1_nav/obstacle_cloud")
    # 墙用世界坐标(odom)定义，但发布时转到 sensor_frame。
    # 为什么不直接发 odom 帧：costmap_2d 把点云 header 的 frame 原点当成"传感器位置"，
    # 用它做射线清除(raytrace clearing)的起点。如果发 odom 帧，起点就是 odom 原点(0,0)，
    # 通常落在滚动窗口外，于是报 "sensor origin out of map bounds"、清除功能失效
    # （只能标记不能清除，地图会越跑越脏）。转到机身坐标系后起点=机器人位置，行为才和真实传感器一致。
    world_frame = rospy.get_param("~world_frame", "odom")
    sensor_frame = rospy.get_param("~sensor_frame", "base")
    max_range = rospy.get_param("~max_range", 6.0)   # 模拟传感器量程
    rate_hz = rospy.get_param("~rate", 5.0)

    # 默认参数针对 auto.sh 的出生点：x=0, y=-3.2, yaw=90°（朝 +y）。
    # 墙放在 y=-1.5，即狗正前方约 1.7m 处。
    wall_y = rospy.get_param("~wall_y", -1.5)
    half_width = rospy.get_param("~half_width", 2.5)
    gap_center = rospy.get_param("~gap_center", 1.0)
    gap_width = rospy.get_param("~gap_width", 0.9)
    z_min = rospy.get_param("~z_min", 0.15)
    z_max = rospy.get_param("~z_max", 1.00)
    spacing = rospy.get_param("~spacing", 0.05)

    points = build_scene(wall_y, half_width, gap_center, gap_width,
                         z_min, z_max, spacing)

    pub = rospy.Publisher(topic, PointCloud2, queue_size=1)
    tf_buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(tf_buf)

    rospy.loginfo("fake_obstacle_publisher: %d points -> %s (%s 定义, 发布为 %s, %.1f Hz)",
                  len(points), topic, world_frame, sensor_frame, rate_hz)
    rospy.loginfo("  wall at y=%.2f, x in [%.2f, %.2f], gap at x=%.2f width %.2f",
                  wall_y, -half_width, half_width, gap_center, gap_width)

    rate = rospy.Rate(rate_hz)
    while not rospy.is_shutdown():
        # 查 sensor_frame ← world_frame 的变换，把世界坐标下的墙转到机身坐标系
        try:
            tr = tf_buf.lookup_transform(sensor_frame, world_frame,
                                         rospy.Time(0), rospy.Duration(0.5))
        except Exception as exc:
            rospy.logwarn_throttle(5.0, "等待 TF %s <- %s: %s",
                                   sensor_frame, world_frame, exc)
            rate.sleep()
            continue

        t = tr.transform.translation
        q = tr.transform.rotation
        # 四元数转旋转矩阵
        xx, yy, zz, ww = q.x, q.y, q.z, q.w
        r00 = 1 - 2 * (yy * yy + zz * zz)
        r01 = 2 * (xx * yy - zz * ww)
        r02 = 2 * (xx * zz + yy * ww)
        r10 = 2 * (xx * yy + zz * ww)
        r11 = 1 - 2 * (xx * xx + zz * zz)
        r12 = 2 * (yy * zz - xx * ww)
        r20 = 2 * (xx * zz - yy * ww)
        r21 = 2 * (yy * zz + xx * ww)
        r22 = 1 - 2 * (xx * xx + yy * yy)

        local = []
        for (wx, wy, wz) in points:
            lx = r00 * wx + r01 * wy + r02 * wz + t.x
            ly = r10 * wx + r11 * wy + r12 * wz + t.y
            lz = r20 * wx + r21 * wy + r22 * wz + t.z
            if lx * lx + ly * ly + lz * lz <= max_range * max_range:
                local.append((lx, ly, lz))

        header = Header()
        # 用 rospy.Time.now()；仿真下 use_sim_time=true 时它就是 /clock 的时间，
        # 和 TF 时间戳一致，否则 costmap 会因 "message too old" 丢掉整帧
        header.stamp = rospy.Time.now()
        header.frame_id = sensor_frame
        pub.publish(pc2.create_cloud_xyz32(header, local))
        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
