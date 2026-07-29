#!/usr/bin/env python3
import argparse
import csv
import json
import os
import time

import rospy
from a1_navigation_interfaces.msg import DoorwayArray, WallSegmentArray
from diagnostic_msgs.msg import DiagnosticStatus
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2


class Recorder:
    def __init__(self):
        self.rows=[];self.input_times=[];self.cloud_times=[];self.map_times=[];self.latest_status=None;self.latest_structure={};self.doorway_rows=[];self.wall_rows=[]
        rospy.Subscriber("/a1/floor_mapping/status",DiagnosticStatus,self.status)
        rospy.Subscriber("/a1/floor_mapping/structure_status",DiagnosticStatus,self.structure_status)
        rospy.Subscriber("/a1/floor_mapping/doorways",DoorwayArray,self.doorways)
        rospy.Subscriber("/a1/floor_mapping/walls",WallSegmentArray,self.walls)
        rospy.Subscriber("/a1_localization/livox_pointcloud",PointCloud2,lambda message:self.input_times.append(message.header.stamp))
        rospy.Subscriber("/a1/floor_mapping/obstacle_cloud",PointCloud2,lambda message:self.cloud_times.append(message.header.stamp))
        rospy.Subscriber("/a1/floor_mapping/map",OccupancyGrid,lambda message:self.map_times.append(message.header.stamp))

    def status(self,message):
        values={item.key:item.value for item in message.values};values["state"]=message.message;values["wall_time"]=time.time();self.latest_status=values

    def structure_status(self,message):
        values={"structure_"+item.key:item.value for item in message.values};values["structure_message"]=message.message;self.latest_structure=values

    def doorways(self,message):
        for doorway in message.doorways:
            self.doorway_rows.append({"stamp":message.header.stamp.to_sec(),"frame_id":message.header.frame_id,"detection_id":doorway.detection_id,"localization_generation":doorway.localization_generation,"floor_session_id":doorway.floor_session_id,"state":doorway.state,"width":doorway.width,"usable_width":doorway.usable_width,"confidence":doorway.confidence,"observation_count":doorway.observation_count,"stable":doorway.stable,"traversable":doorway.traversable,"control_id_matched":doorway.control_id_matched,"control_door_id":doorway.control_door_id,"door_kind":doorway.door_kind})

    def walls(self,message):
        for wall in message.walls:
            self.wall_rows.append({"stamp":message.header.stamp.to_sec(),"frame_id":message.header.frame_id,"detection_id":wall.detection_id,"localization_generation":wall.localization_generation,"floor_session_id":wall.floor_session_id,"length":wall.length,"fit_error":wall.fit_error,"height_support":wall.height_support,"confidence":wall.confidence,"observation_count":wall.observation_count,"stable":wall.stable})

    def sample(self):
        if self.latest_status:
            row=dict(self.latest_status);row.update(self.latest_structure);self.rows.append(row)


def percentile(values, fraction):
    if not values:return None
    ordered=sorted(values);return ordered[min(len(ordered)-1,int((len(ordered)-1)*fraction))]


def wait_for_sim_time(timeout_sec=10.0):
    """Wait until /clock is available before defining the recording interval.

    Gazebo starts clients before it publishes its first clock message.  Capturing
    ``rospy.Time.now()`` during that small window yields zero, and the first
    non-zero clock tick then makes a recorder appear to have run for thousands
    of seconds.  Use wall time only for this bootstrap and retain simulation
    time for all measurements in the report.
    """
    deadline=time.monotonic()+timeout_sec
    while not rospy.is_shutdown() and time.monotonic()<deadline:
        now=rospy.Time.now()
        if now.to_sec()>0.0:
            return now
        rospy.rostime.wallsleep(0.02)
    raise RuntimeError("did not receive a non-zero ROS simulation clock")


def main():
    parser=argparse.ArgumentParser();parser.add_argument("--duration",type=float,default=600);parser.add_argument("--output",required=True);parser.add_argument("--wall-timeout-factor",type=float,default=20.0);args=parser.parse_args()
    rospy.init_node("floor_mapping_validation_recorder");os.makedirs(args.output,exist_ok=True);recorder=Recorder();wall_started=time.monotonic();started=wait_for_sim_time();rate=rospy.Rate(5)
    while not rospy.is_shutdown() and (rospy.Time.now()-started).to_sec()<args.duration and time.monotonic()-wall_started<max(60.0,args.duration*args.wall_timeout_factor):recorder.sample();rate.sleep()
    keys=sorted({key for row in recorder.rows for key in row})
    with open(os.path.join(args.output,"mapping_status.csv"),"w",newline="") as stream:
        writer=csv.DictWriter(stream,fieldnames=keys);writer.writeheader();writer.writerows(recorder.rows)
    for name,rows in (("doorways.csv",recorder.doorway_rows),("walls.csv",recorder.wall_rows)):
        if rows:
            with open(os.path.join(args.output,name),"w",newline="") as stream:
                writer=csv.DictWriter(stream,fieldnames=rows[0].keys());writer.writeheader();writer.writerows(rows)
    numeric=lambda key:[float(row[key]) for row in recorder.rows if key in row and row[key] not in ("unknown","")]
    processing=numeric("processing_time_ms");floor=numeric("floor_z");margins=numeric("minimum_boundary_margin_m")
    elapsed=max(1e-6,(rospy.Time.now()-started).to_sec())
    input_hz=len(recorder.input_times)/elapsed;output_hz=len(recorder.cloud_times)/elapsed
    doorway_states={str(state):sum(1 for row in recorder.doorway_rows if row["state"]==state) for state in sorted({row["state"] for row in recorder.doorway_rows})}
    summary={"duration_sim_sec":elapsed,"duration_wall_sec":time.monotonic()-wall_started,"samples":len(recorder.rows),"states":{state:sum(1 for row in recorder.rows if row.get("state")==state) for state in sorted({row.get("state") for row in recorder.rows})},"input_cloud_hz_sim":input_hz,"obstacle_cloud_hz_sim":output_hz,"output_input_ratio":output_hz/input_hz if input_hz else None,"map_hz_sim":len(recorder.map_times)/elapsed,"processing_ms_p50":percentile(processing,.5),"processing_ms_p95":percentile(processing,.95),"floor_z_peak_to_peak":max(floor)-min(floor) if floor else None,"minimum_boundary_margin_m":min(margins) if margins else None,"tf_failure_count":max(numeric("tf_failure_count")) if numeric("tf_failure_count") else None,"occupied_cells_final":numeric("occupied_cells")[-1] if numeric("occupied_cells") else None,"free_cells_final":numeric("free_cells")[-1] if numeric("free_cells") else None,"doorway_observations":len(recorder.doorway_rows),"wall_observations":len(recorder.wall_rows),"doorway_state_observations":doorway_states,"structure_results_valid_final":recorder.rows[-1].get("structure_results_valid") if recorder.rows else None}
    with open(os.path.join(args.output,"summary.json"),"w") as stream:json.dump(summary,stream,indent=2,sort_keys=True)
    print(json.dumps(summary,indent=2,sort_keys=True))


if __name__=="__main__":main()
