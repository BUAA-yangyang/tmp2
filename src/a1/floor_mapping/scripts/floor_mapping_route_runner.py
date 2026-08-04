#!/usr/bin/env python3
"""Readiness-gated, watchdog-safe route runner for repeatable mapping validation."""
import argparse
import csv
import hashlib
import json
import os
import platform
import signal
import time

import rospy
import yaml
from diagnostic_msgs.msg import DiagnosticStatus
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import PointCloud2


def load_route(path):
    with open(path) as stream:
        document = yaml.safe_load(stream)
    segments = document.get("segments", [])
    if not segments:
        raise ValueError("route must contain at least one segment")
    limits = document.get("limits", {})
    max_linear, max_angular = float(limits.get("linear", 0.25)), float(limits.get("angular", 0.5))
    normalized = []
    for index, segment in enumerate(segments):
        duration, linear, angular = float(segment["duration"]), float(segment.get("linear", 0)), float(segment.get("angular", 0))
        if duration <= 0 or abs(linear) > max_linear or abs(angular) > max_angular:
            raise ValueError("unsafe route segment %d" % index)
        normalized.append({"name": segment.get("name", "segment_%d" % index), "duration": duration, "linear": linear, "angular": angular})
    return normalized, {"linear": max_linear, "angular": max_angular}


class Runner:
    def __init__(self, output):
        self.output = output; self.status = None; self.status_wall = 0.; self.cloud_wall = 0.; self.rows = []; self.odom_rows = []; self.truth_rows = []; self.grid = None; self.aborted_reason = None
        self.wall_heartbeat_timeout=float(rospy.get_param("~wall_heartbeat_timeout",5.0));self.segment_wall_factor=float(rospy.get_param("~segment_wall_timeout_factor",20.0))
        self.pub = rospy.Publisher(rospy.get_param("~cmd_vel_topic", "/cmd_vel"), Twist, queue_size=1)
        rospy.Subscriber(rospy.get_param("~mapping_status_topic", "/a1/floor_mapping/status"), DiagnosticStatus, self.on_status, queue_size=1)
        rospy.Subscriber(rospy.get_param("~obstacle_cloud_topic", "/a1/floor_mapping/obstacle_cloud"), PointCloud2, self.on_cloud, queue_size=1)
        rospy.Subscriber(rospy.get_param("~map_topic", "/a1/floor_mapping/map"), OccupancyGrid, self.on_map, queue_size=1)
        rospy.Subscriber(rospy.get_param("~odom_topic", "/a1/localization/odom"), Odometry, lambda m:self.on_pose(m,self.odom_rows), queue_size=5)
        truth_topic=rospy.get_param("~validation_truth_topic", "/ground_truth/base_w")
        if truth_topic: rospy.Subscriber(truth_topic, Odometry, lambda m:self.on_pose(m,self.truth_rows), queue_size=5)

    def on_status(self, message):
        values = {item.key: item.value for item in message.values}; values.update(wall_time=time.time(), ros_time=rospy.Time.now().to_sec(), state=message.message)
        self.status, self.status_wall = values, time.monotonic(); self.rows.append(values)

    def on_map(self, message): self.grid = message

    def on_cloud(self, _message): self.cloud_wall = time.monotonic()

    def on_pose(self, message, rows):
        p=message.pose.pose.position;q=message.pose.pose.orientation
        rows.append({"stamp":message.header.stamp.to_sec(),"x":p.x,"y":p.y,"z":p.z,"qx":q.x,"qy":q.y,"qz":q.z,"qw":q.w})

    def ready(self):
        # Mapping owns freshness in ROS/simulation time. These wall bounds only
        # detect a frozen process or simulator.
        now = time.monotonic()
        return self.status is not None and self.status.get("state") == "MAPPING" and self.status.get("map_valid") == "true" and self.status.get("obstacle_cloud_valid") == "true" and now-self.status_wall < self.wall_heartbeat_timeout and now-self.cloud_wall < self.wall_heartbeat_timeout

    def stop(self, count=5):
        for _ in range(count): self.pub.publish(Twist()); time.sleep(0.04)

    def execute(self, segments, readiness_timeout):
        deadline = time.monotonic()+readiness_timeout
        while not rospy.is_shutdown() and not self.ready() and time.monotonic()<deadline: self.stop(1); rospy.sleep(0.05)
        if not self.ready(): raise RuntimeError("readiness_timeout")
        rate = rospy.Rate(20)
        for segment in segments:
            end=rospy.Time.now()+rospy.Duration(segment["duration"]);wall_deadline=time.monotonic()+max(30.0,segment["duration"]*self.segment_wall_factor)
            while not rospy.is_shutdown() and rospy.Time.now()<end:
                if time.monotonic()>=wall_deadline:self.aborted_reason="simulation_wall_timeout";raise RuntimeError(self.aborted_reason)
                if not self.ready(): self.aborted_reason="mapping_health_lost"; raise RuntimeError(self.aborted_reason)
                command=Twist();command.linear.x=segment["linear"];command.angular.z=segment["angular"];self.pub.publish(command);rate.sleep()
            self.stop()

    def write_artifacts(self, route_path, limits, result):
        os.makedirs(self.output, exist_ok=True)
        keys=sorted({key for row in self.rows for key in row})
        with open(os.path.join(self.output,"mapping_status.csv"),"w",newline="") as stream:
            writer=csv.DictWriter(stream,fieldnames=keys);writer.writeheader();writer.writerows(self.rows)
        def write_trajectory(name, rows):
            if rows:
                with open(os.path.join(self.output,name),"w",newline="") as stream:
                    writer=csv.DictWriter(stream,fieldnames=rows[0].keys());writer.writeheader();writer.writerows(rows)
            length=sum(((b["x"]-a["x"])**2+(b["y"]-a["y"])**2)**.5 for a,b in zip(rows,rows[1:]))
            maximum=max((((row["x"]-rows[0]["x"])**2+(row["y"]-rows[0]["y"])**2)**.5 for row in rows),default=0.)
            return {"samples":len(rows),"path_length_m":length,"maximum_displacement_m":maximum}
        odom_metrics=write_trajectory("odom_trajectory.csv",self.odom_rows)
        truth_metrics=write_trajectory("truth_trajectory.csv",self.truth_rows)
        grid_summary=None
        if self.grid:
            grid_summary={"frame":self.grid.header.frame_id,"stamp":self.grid.header.stamp.to_sec(),"resolution":self.grid.info.resolution,"width":self.grid.info.width,"height":self.grid.info.height,"unknown":self.grid.data.count(-1),"free":self.grid.data.count(0),"occupied":self.grid.data.count(100)}
            with open(os.path.join(self.output,"map.pgm"),"wb") as stream:
                stream.write(("P5\n%d %d\n255\n"%(self.grid.info.width,self.grid.info.height)).encode("ascii"))
                for y in range(self.grid.info.height-1,-1,-1):
                    start=y*self.grid.info.width
                    stream.write(bytes(0 if value==100 else 254 if value==0 else 205 for value in self.grid.data[start:start+self.grid.info.width]))
            with open(os.path.join(self.output,"map.yaml"),"w") as stream:
                yaml.safe_dump({"image":"map.pgm","resolution":self.grid.info.resolution,"origin":[self.grid.info.origin.position.x,self.grid.info.origin.position.y,0.0],"frame_id":self.grid.header.frame_id,"generation":self.status.get("localization_generation") if self.status else None,"floor_session_id":self.status.get("floor_session_id") if self.status else None,"negate":0,"occupied_thresh":0.65,"free_thresh":0.196},stream,default_flow_style=False)
        with open(route_path,"rb") as stream: route_sha=hashlib.sha256(stream.read()).hexdigest()
        manifest={"result":result,"reason":self.aborted_reason,"route":os.path.abspath(route_path),"route_sha256":route_sha,"limits":limits,"grid":grid_summary,"odom_trajectory":odom_metrics,"validation_truth_trajectory":truth_metrics,"host":platform.node(),"ros_distro":os.environ.get("ROS_DISTRO"),"finished_at":time.time()}
        with open(os.path.join(self.output,"manifest.json"),"w") as stream: json.dump(manifest,stream,indent=2,sort_keys=True)


def main():
    parser=argparse.ArgumentParser();parser.add_argument("--route",required=True);parser.add_argument("--output",required=True);parser.add_argument("--readiness-timeout",type=float,default=30.);args=parser.parse_args()
    segments,limits=load_route(args.route);rospy.init_node("floor_mapping_route_runner");runner=Runner(args.output)
    def shutdown(*_): runner.stop()
    signal.signal(signal.SIGINT,shutdown);signal.signal(signal.SIGTERM,shutdown);result="failed"
    try: runner.execute(segments,args.readiness_timeout);result="passed"
    except Exception as error: runner.aborted_reason=runner.aborted_reason or str(error);rospy.logerr("route aborted: %s",error)
    finally: runner.stop();runner.write_artifacts(args.route,limits,result)
    if result != "passed": raise SystemExit(2)


if __name__ == "__main__": main()
