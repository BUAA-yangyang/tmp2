#!/usr/bin/env python3
import math
import sys
import rospy
import rosgraph
from sensor_msgs.msg import Imu, PointCloud, PointCloud2, PointField

def fail(message):
    rospy.logerr(message)
    raise RuntimeError(message)

def main():
    rospy.init_node("a1_localization_preflight", anonymous=True)
    raw = rospy.wait_for_message("/scan", PointCloud, timeout=30)
    cloud = rospy.wait_for_message("/a1_localization/livox_pointcloud", PointCloud2, timeout=30)
    imu = rospy.wait_for_message("/trunk_imu", Imu, timeout=30)
    if raw.header.frame_id != "laser_livox" or cloud.header.frame_id != "laser_livox":
        fail("unexpected lidar frame")
    fields = [(f.name, f.offset, f.datatype, f.count) for f in cloud.fields]
    expected = [("x", 0, PointField.FLOAT32, 1), ("y", 4, PointField.FLOAT32, 1),
                ("z", 8, PointField.FLOAT32, 1), ("intensity", 12, PointField.FLOAT32, 1)]
    if fields != expected or cloud.point_step != 16 or cloud.row_step != cloud.width * 16:
        fail("invalid PointCloud2 XYZI layout")
    if len(cloud.data) != cloud.row_step * cloud.height or cloud.is_bigendian:
        fail("invalid PointCloud2 storage")
    values = list(imu.angular_velocity.__getstate__()) + list(imu.linear_acceleration.__getstate__())
    if not all(math.isfinite(v) for v in values):
        fail("IMU contains non-finite values")
    master = rosgraph.Master(rospy.get_name())
    publishers, subscribers, _ = master.getSystemState()
    pubs = dict(publishers).get("/a1_localization/livox_pointcloud", [])
    if len(pubs) != 1:
        fail("localization pointcloud must have exactly one publisher")
    forbidden = {"/Odometry_gazebo"}
    forbidden.update(t for t, _ in subscribers if t.startswith("/ground_truth/"))
    for topic, nodes in subscribers:
        if topic in forbidden and any(n.startswith("/a1_localization") for n in nodes):
            fail("localization subscribes forbidden truth topic " + topic)
    rospy.loginfo("a1 localization preflight passed")
    return 0

if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as exc:
        rospy.logerr(str(exc))
        sys.exit(1)
