#!/usr/bin/env python3
import math
import time
import unittest

import rospy
import rostest
import tf2_ros
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


class CostmapTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rospy.init_node("floor_mapping_costmap_test")
        cls.cloud_pub = rospy.Publisher("/test/costmap/cloud", PointCloud2, queue_size=2)
        cls.odom_pub = rospy.Publisher("/test/costmap/odom", Odometry, queue_size=2)
        cls.loc_pub = rospy.Publisher("/test/costmap/localization", DiagnosticStatus, queue_size=2, latch=True)
        cls.sup_pub = rospy.Publisher("/test/costmap/supervisor", DiagnosticStatus, queue_size=2, latch=True)
        cls.costmaps = []
        cls.mapping_clouds = []
        rospy.Subscriber("/test_costmap/costmap/costmap", OccupancyGrid, cls.costmaps.append)
        rospy.Subscriber("/test/costmap/mapping_cloud", PointCloud2, cls.mapping_clouds.append)
        cls.tf = tf2_ros.TransformBroadcaster()
        time.sleep(1.0)

    def publish_health(self):
        status = DiagnosticStatus(values=[KeyValue("state", "TRACKING"), KeyValue("results_valid", "true")])
        supervisor = DiagnosticStatus(values=[KeyValue("generation", "1")])
        self.loc_pub.publish(status); self.sup_pub.publish(supervisor)

    def publish_frame(self, obstacle):
        stamp = rospy.Time.now()
        for parent, child, z in (("odom", "base", 0.0), ("base", "laser_livox", 0.5)):
            transform = TransformStamped(); transform.header.stamp = stamp; transform.header.frame_id = parent; transform.child_frame_id = child
            transform.transform.translation.z = z; transform.transform.rotation.w = 1.0; self.tf.sendTransform(transform)
        odom = Odometry(); odom.header = Header(stamp=stamp, frame_id="odom"); odom.child_frame_id = "base"; odom.pose.pose.position.z = 0.5; odom.pose.pose.orientation.w = 1.0; self.odom_pub.publish(odom)
        time.sleep(0.03)
        fields = [PointField("x",0,PointField.FLOAT32,1),PointField("y",4,PointField.FLOAT32,1),PointField("z",8,PointField.FLOAT32,1),PointField("intensity",12,PointField.FLOAT32,1)]
        points = [(ix*0.12, iy*0.12, -0.5, 1.0) for ix in range(-20,31) for iy in range(-15,16) if math.hypot(ix*0.12,iy*0.12)>0.5]
        obstacle_y=[i*0.04 for i in range(-5,6)]
        if obstacle: points += [(2.0, y, 0.0, 2.0) for y in obstacle_y]
        else: points += [(3.0, y, -0.5, 1.0) for y in obstacle_y]
        self.cloud_pub.publish(point_cloud2.create_cloud(Header(stamp=stamp, frame_id="laser_livox"), fields, points))

    def cell(self, grid, x, y):
        mx=int((x-grid.info.origin.position.x)/grid.info.resolution);my=int((y-grid.info.origin.position.y)/grid.info.resolution)
        return grid.data[my*grid.info.width+mx]

    def region_max(self, grid, x, y, radius=0.25):
        values=[]
        steps=int(radius/grid.info.resolution)
        cx=int((x-grid.info.origin.position.x)/grid.info.resolution);cy=int((y-grid.info.origin.position.y)/grid.info.resolution)
        for dx in range(-steps,steps+1):
            for dy in range(-steps,steps+1):
                values.append(grid.data[(cy+dy)*grid.info.width+cx+dx])
        return max(values)

    def wait_value(self, expected, obstacle, timeout):
        end=time.time()+timeout
        while time.time()<end and not rospy.is_shutdown():
            self.publish_frame(obstacle);time.sleep(0.1)
            if self.costmaps and ((expected==100 and self.region_max(self.costmaps[-1],2.0,0.0)==100) or (expected==0 and self.cell(self.costmaps[-1],2.0,0.0)<100)):return
        max_z=max((p[2] for p in point_cloud2.read_points(self.mapping_clouds[-1],field_names=("x","y","z"),skip_nans=True)),default=-99) if self.mapping_clouds else -99
        self.fail("costmap region did not become %s; last=%s global_max=%s mapping_clouds=%s max_sensor_z=%s"%(expected,self.region_max(self.costmaps[-1],2.0,0.0) if self.costmaps else None,max(self.costmaps[-1].data) if self.costmaps else None,len(self.mapping_clouds),max_z))

    def test_marking_clearing_and_sensor_origin(self):
        self.publish_health()
        self.wait_value(100, True, 8.0)
        self.wait_value(0, False, 6.0)
        self.assertTrue(self.costmaps)


if __name__ == "__main__":
    rostest.rosrun("a1_floor_mapping", "floor_mapping_costmap", CostmapTest)
