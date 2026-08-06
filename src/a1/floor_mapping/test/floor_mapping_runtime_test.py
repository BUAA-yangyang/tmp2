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
from std_srvs.srv import Trigger

class RuntimeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rospy.init_node("floor_mapping_runtime_test")
        cls.clouds=[]; cls.clearing_clouds=[]; cls.maps=[]; cls.statuses=[]
        cls.cloud_pub=rospy.Publisher("/test/mapping/cloud",PointCloud2,queue_size=2)
        cls.odom_pub=rospy.Publisher("/test/mapping/odom",Odometry,queue_size=2)
        cls.loc_pub=rospy.Publisher("/test/mapping/localization",DiagnosticStatus,queue_size=2,latch=True)
        cls.sup_pub=rospy.Publisher("/test/mapping/supervisor",DiagnosticStatus,queue_size=2,latch=True)
        rospy.Subscriber("/test/mapping/output_cloud",PointCloud2,cls.clouds.append)
        rospy.Subscriber("/test/mapping/clearing_cloud",PointCloud2,cls.clearing_clouds.append)
        rospy.Subscriber("/test/mapping/map",OccupancyGrid,cls.maps.append)
        rospy.Subscriber("/test/mapping/status",DiagnosticStatus,cls.statuses.append)
        cls.tf=tf2_ros.TransformBroadcaster(); time.sleep(1)

    def status(self, tracking=True):
        msg=DiagnosticStatus(); msg.values=[KeyValue("state","TRACKING" if tracking else "LOST"),KeyValue("results_valid","true" if tracking else "false")]; self.loc_pub.publish(msg)
    def supervisor(self,g):
        msg=DiagnosticStatus(); msg.values=[KeyValue("generation",str(g))]; self.sup_pub.publish(msg)
    def send_tf(self, stamp):
        sensor=TransformStamped();sensor.header.stamp=stamp;sensor.header.frame_id="odom";sensor.child_frame_id="laser_livox";sensor.transform.translation.z=0.5;sensor.transform.rotation.w=1
        base=TransformStamped();base.header.stamp=stamp;base.header.frame_id="odom";base.child_frame_id="base";base.transform.translation.z=0.5;base.transform.rotation.w=1
        self.tf.sendTransform([sensor,base])
    def frame(self, stamp=None, floor_sensor_z=-0.5, publish_tf=True,
              include_obstacle=True, extra_points=()):
        stamp=stamp or rospy.Time.now()
        if publish_tf:self.send_tf(stamp)
        od=Odometry();od.header.stamp=stamp;od.header.frame_id="odom";od.child_frame_id="base";od.pose.pose.position.z=0.5;od.pose.pose.orientation.w=1;self.odom_pub.publish(od)
        fields=[PointField("x",0,PointField.FLOAT32,1),PointField("y",4,PointField.FLOAT32,1),PointField("z",8,PointField.FLOAT32,1),PointField("intensity",12,PointField.FLOAT32,1)]
        pts=[]
        for ix in range(-10,11):
            for iy in range(-10,11):
                x=ix*0.12;y=iy*0.12
                if math.hypot(x,y)>0.5: pts.append((x,y,floor_sensor_z,1.0))
        if include_obstacle:
            pts += [(2.0,y,0.0,2.0) for y in (-.2,-.1,0,.1,.2)]
        pts.extend(extra_points)
        msg=point_cloud2.create_cloud(Header(stamp=stamp,frame_id="laser_livox"),fields,pts);self.cloud_pub.publish(msg);return stamp
    @staticmethod
    def map_value(message, x, y):
        col=int(math.floor((x-message.info.origin.position.x)/message.info.resolution))
        row=int(math.floor((y-message.info.origin.position.y)/message.info.resolution))
        if row<0 or col<0 or row>=message.info.height or col>=message.info.width:return None
        return message.data[row*message.info.width+col]
    def wait_new_map(self, previous_count, timeout=3):
        end=time.time()+timeout
        while time.time()<end and not rospy.is_shutdown():
            if len(self.maps)>previous_count:return self.maps[-1]
            time.sleep(.03)
        self.fail("fresh map not published")
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
        first_session=int(dict((v.key,v.value) for v in status.values)["floor_session_id"])
        self.assertTrue(self.clouds);self.assertEqual(self.clouds[-1].header.frame_id,"laser_livox")
        self.assertTrue(self.clearing_clouds);self.assertEqual(self.clearing_clouds[-1].header.frame_id,"laser_livox")
        obstacles=list(point_cloud2.read_points(self.clouds[-1],field_names=("x","y","z"),skip_nans=True))
        clearing=list(point_cloud2.read_points(self.clearing_clouds[-1],field_names=("x","y","z"),skip_nans=True))
        self.assertGreater(len(clearing),len(obstacles))
        self.assertGreater(len(obstacles),0)
        self.assertTrue(all(point[2] > -0.25 for point in obstacles))
        end=time.time()+3
        while time.time()<end and (not self.maps or 100 not in self.maps[-1].data or 0 not in self.maps[-1].data):
            self.frame();time.sleep(.12)
        self.assertTrue(self.maps);self.assertEqual(self.maps[-1].header.frame_id,"odom");self.assertEqual(len(self.maps[-1].data),self.maps[-1].info.width*self.maps[-1].info.height);self.assertIn(100,self.maps[-1].data);self.assertIn(0,self.maps[-1].data)
        self.assertEqual(self.map_value(self.maps[-1],2.0,0.0),100)

        # A 3-D ray can pass above a low obstacle while crossing the same 2-D
        # cell. Twenty fresh frames of such ray traversal must not erase a
        # formally confirmed obstacle column.
        before=len(self.maps)
        for _ in range(20):
            self.frame(
                include_obstacle=False,
                extra_points=((3.0,0.0,-0.5,3.0),),
            );time.sleep(.04)
        persisted=self.wait_new_map(before)
        self.assertEqual(self.map_value(persisted,2.0,0.0),100)

        # The cell may still clear dynamically, but only after the configured
        # number of actual near-floor endpoints land in that exact cell.
        before=len(self.maps)
        for _ in range(3):
            self.frame(
                include_obstacle=False,
                extra_points=((2.0,0.0,-0.5,3.0),),
            );time.sleep(.08)
        cleared=self.wait_new_map(before)
        self.assertEqual(self.map_value(cleared,2.0,0.0),0)
        # The real chain publishes the raw cloud before FAST-LIO can publish
        # odometry/TF for the same stamp. A short inversion of arrival order
        # must be queued and processed with the exact transform, not rejected.
        baseline_status=dict((v.key,v.value) for v in self.statuses[-1].values);baseline_failures=int(baseline_status["tf_failure_count"])
        before=len(self.clouds);stamp=self.frame(publish_tf=False);time.sleep(.15);self.send_tf(stamp)
        end=time.time()+2
        while time.time()<end and len(self.clouds)==before:time.sleep(.02)
        self.assertGreater(len(self.clouds),before)
        time.sleep(.25);delayed=self.wait_state("MAPPING");delayed_values=dict((v.key,v.value) for v in delayed.values)
        self.assertEqual(int(delayed_values["tf_failure_count"]),baseline_failures);self.assertEqual(delayed_values["tf_pending_clouds"],"0")

        # A transform that never arrives still expires under the independent
        # wall bound and invalidates both products. Subsequent exact-stamp
        # frames recover through the normal recovery gate.
        self.frame(publish_tf=False);expired=self.wait_state("WAITING_FOR_TF",2);expired_values=dict((v.key,v.value) for v in expired.values)
        self.assertEqual(expired_values["map_valid"],"false");self.assertGreaterEqual(int(expired_values["tf_failure_count"]),1)
        for _ in range(4):self.frame();time.sleep(.12)
        self.wait_state("MAPPING")
        old_count=len(self.clouds);self.status(False);time.sleep(.25);self.frame();time.sleep(.2);self.assertEqual(len(self.clouds),old_count)
        self.status(True);time.sleep(.15);self.frame(publish_tf=False);time.sleep(.05);self.supervisor(2);time.sleep(.2);s=self.wait_state("RESETTING");reset_values=dict((v.key,v.value) for v in s.values)
        self.assertEqual(reset_values["map_valid"],"false");self.assertEqual(reset_values["tf_pending_clouds"],"0")
        for _ in range(6): self.frame();time.sleep(.12)
        next_status=self.wait_state("MAPPING")
        self.assertGreater(int(dict((v.key,v.value) for v in next_status.values)["floor_session_id"]),first_session)
        regression=rospy.Time.now()-rospy.Duration(5.0);self.frame(regression);lost=self.wait_state("LOST")
        self.assertEqual(dict((v.key,v.value) for v in lost.values)["map_valid"],"false")
        self.status(False);time.sleep(.25);self.assertEqual(self.statuses[-1].message,"LOST")
        rospy.wait_for_service("/a1/floor_mapping/reset",3);reset=rospy.ServiceProxy("/a1/floor_mapping/reset",Trigger);self.assertTrue(reset().success)
        self.status(True)
        for _ in range(6): self.frame();time.sleep(.12)
        reset_status=self.wait_state("MAPPING")
        self.assertGreater(int(dict((v.key,v.value) for v in reset_status.values)["floor_session_id"]),int(dict((v.key,v.value) for v in next_status.values)["floor_session_id"]))
        for _ in range(3): self.frame(floor_sensor_z=0.0);time.sleep(.12)
        changed=self.wait_state("FLOOR_CHANGE_UNSUPPORTED");values=dict((v.key,v.value) for v in changed.values)
        self.assertEqual(values["obstacle_cloud_valid"],"false");self.assertGreaterEqual(int(values["floor_change_count"]),3)

if __name__ == "__main__": rostest.rosrun("a1_floor_mapping","floor_mapping_runtime",RuntimeTest)
