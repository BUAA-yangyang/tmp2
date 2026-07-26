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
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
from std_msgs.msg import Header

class RuntimeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rospy.init_node("floor_mapping_runtime_test")
        cls.clouds=[]; cls.maps=[]; cls.statuses=[]
        cls.cloud_pub=rospy.Publisher("/test/mapping/cloud",PointCloud2,queue_size=2)
        cls.odom_pub=rospy.Publisher("/test/mapping/odom",Odometry,queue_size=2)
        cls.loc_pub=rospy.Publisher("/test/mapping/localization",DiagnosticStatus,queue_size=2,latch=True)
        cls.sup_pub=rospy.Publisher("/test/mapping/supervisor",DiagnosticStatus,queue_size=2,latch=True)
        rospy.Subscriber("/test/mapping/output_cloud",PointCloud2,cls.clouds.append)
        rospy.Subscriber("/test/mapping/map",OccupancyGrid,cls.maps.append)
        rospy.Subscriber("/test/mapping/status",DiagnosticStatus,cls.statuses.append)
        cls.tf=tf2_ros.TransformBroadcaster(); time.sleep(1)

    def status(self, tracking=True):
        msg=DiagnosticStatus(); msg.values=[KeyValue("state","TRACKING" if tracking else "LOST"),KeyValue("results_valid","true" if tracking else "false")]; self.loc_pub.publish(msg)
    def supervisor(self,g):
        msg=DiagnosticStatus(); msg.values=[KeyValue("generation",str(g))]; self.sup_pub.publish(msg)
    def frame(self, stamp=None):
        stamp=stamp or rospy.Time.now(); t=TransformStamped();t.header.stamp=stamp;t.header.frame_id="odom";t.child_frame_id="laser_livox";t.transform.translation.z=0.5;t.transform.rotation.w=1;self.tf.sendTransform(t)
        od=Odometry();od.header.stamp=stamp;od.header.frame_id="odom";od.child_frame_id="base";od.pose.pose.position.z=0.5;od.pose.pose.orientation.w=1;self.odom_pub.publish(od)
        fields=[PointField("x",0,PointField.FLOAT32,1),PointField("y",4,PointField.FLOAT32,1),PointField("z",8,PointField.FLOAT32,1),PointField("intensity",12,PointField.FLOAT32,1)]
        pts=[]
        for ix in range(-10,11):
            for iy in range(-10,11):
                x=ix*0.12;y=iy*0.12
                if math.hypot(x,y)>0.5: pts.append((x,y,-0.5,1.0))
        pts += [(2.0,y,0.0,2.0) for y in (-.2,-.1,0,.1,.2)]
        msg=point_cloud2.create_cloud(Header(stamp=stamp,frame_id="laser_livox"),fields,pts);self.cloud_pub.publish(msg);return stamp
    def wait_state(self,name,timeout=8):
        end=time.time()+timeout
        while time.time()<end and not rospy.is_shutdown():
            if self.statuses and self.statuses[-1].message==name:return self.statuses[-1]
            time.sleep(.05)
        self.fail("state %s not reached; last=%s"%(name,self.statuses[-1].message if self.statuses else None))
    def test_mapping_and_lifecycle(self):
        self.status(False);self.supervisor(1);self.frame();time.sleep(.3);self.assertFalse(self.clouds)
        self.status(True)
        for _ in range(6): self.frame();time.sleep(.12)
        status=self.wait_state("MAPPING");self.assertEqual(dict((v.key,v.value) for v in status.values)["map_valid"],"true")
        self.assertTrue(self.clouds);self.assertEqual(self.clouds[-1].header.frame_id,"laser_livox")
        end=time.time()+3
        while time.time()<end and not self.maps:time.sleep(.05)
        self.assertTrue(self.maps);self.assertEqual(self.maps[-1].header.frame_id,"odom");self.assertEqual(len(self.maps[-1].data),self.maps[-1].info.width*self.maps[-1].info.height);self.assertIn(100,self.maps[-1].data);self.assertIn(0,self.maps[-1].data)
        old_count=len(self.clouds);self.status(False);time.sleep(.25);self.frame();time.sleep(.2);self.assertEqual(len(self.clouds),old_count)
        self.status(True);self.supervisor(2);time.sleep(.2);s=self.wait_state("RESETTING");self.assertEqual(dict((v.key,v.value) for v in s.values)["map_valid"],"false")

if __name__ == "__main__": rostest.rosrun("a1_floor_mapping","floor_mapping_runtime",RuntimeTest)
