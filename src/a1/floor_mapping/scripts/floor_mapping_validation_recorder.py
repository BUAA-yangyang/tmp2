#!/usr/bin/env python3
import argparse
import csv
import json
import os
import time

import rospy
from diagnostic_msgs.msg import DiagnosticStatus
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2


class Recorder:
    def __init__(self):
        self.rows=[];self.input_times=[];self.cloud_times=[];self.map_times=[];self.latest_status=None
        rospy.Subscriber("/a1/floor_mapping/status",DiagnosticStatus,self.status)
        rospy.Subscriber("/a1_localization/livox_pointcloud",PointCloud2,lambda message:self.input_times.append(message.header.stamp))
        rospy.Subscriber("/a1/floor_mapping/obstacle_cloud",PointCloud2,lambda message:self.cloud_times.append(message.header.stamp))
        rospy.Subscriber("/a1/floor_mapping/map",OccupancyGrid,lambda message:self.map_times.append(message.header.stamp))

    def status(self,message):
        values={item.key:item.value for item in message.values};values["state"]=message.message;values["wall_time"]=time.time();self.latest_status=values

    def sample(self):
        if self.latest_status:self.rows.append(dict(self.latest_status))


def percentile(values, fraction):
    if not values:return None
    ordered=sorted(values);return ordered[min(len(ordered)-1,int((len(ordered)-1)*fraction))]


def main():
    parser=argparse.ArgumentParser();parser.add_argument("--duration",type=float,default=600);parser.add_argument("--output",required=True);parser.add_argument("--wall-timeout-factor",type=float,default=20.0);args=parser.parse_args()
    rospy.init_node("floor_mapping_validation_recorder");os.makedirs(args.output,exist_ok=True);recorder=Recorder();started=rospy.Time.now();wall_started=time.monotonic();rate=rospy.Rate(5)
    while not rospy.is_shutdown() and (rospy.Time.now()-started).to_sec()<args.duration and time.monotonic()-wall_started<max(60.0,args.duration*args.wall_timeout_factor):recorder.sample();rate.sleep()
    keys=sorted({key for row in recorder.rows for key in row})
    with open(os.path.join(args.output,"mapping_status.csv"),"w",newline="") as stream:
        writer=csv.DictWriter(stream,fieldnames=keys);writer.writeheader();writer.writerows(recorder.rows)
    numeric=lambda key:[float(row[key]) for row in recorder.rows if key in row and row[key] not in ("unknown","")]
    processing=numeric("processing_time_ms");floor=numeric("floor_z");margins=numeric("minimum_boundary_margin_m")
    elapsed=max(1e-6,(rospy.Time.now()-started).to_sec())
    input_hz=len(recorder.input_times)/elapsed;output_hz=len(recorder.cloud_times)/elapsed
    summary={"duration_sim_sec":elapsed,"duration_wall_sec":time.monotonic()-wall_started,"samples":len(recorder.rows),"states":{state:sum(1 for row in recorder.rows if row.get("state")==state) for state in sorted({row.get("state") for row in recorder.rows})},"input_cloud_hz_sim":input_hz,"obstacle_cloud_hz_sim":output_hz,"output_input_ratio":output_hz/input_hz if input_hz else None,"map_hz_sim":len(recorder.map_times)/elapsed,"processing_ms_p50":percentile(processing,.5),"processing_ms_p95":percentile(processing,.95),"floor_z_peak_to_peak":max(floor)-min(floor) if floor else None,"minimum_boundary_margin_m":min(margins) if margins else None,"tf_failure_count":max(numeric("tf_failure_count")) if numeric("tf_failure_count") else None,"occupied_cells_final":numeric("occupied_cells")[-1] if numeric("occupied_cells") else None,"free_cells_final":numeric("free_cells")[-1] if numeric("free_cells") else None}
    with open(os.path.join(args.output,"summary.json"),"w") as stream:json.dump(summary,stream,indent=2,sort_keys=True)
    print(json.dumps(summary,indent=2,sort_keys=True))


if __name__=="__main__":main()
