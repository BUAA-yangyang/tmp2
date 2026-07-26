#!/usr/bin/env python3
"""Acceptance-only trajectory recorder; truth is never fed into localization."""

import argparse
import csv
import math
import os
import sys
import time

import rospy
from diagnostic_msgs.msg import DiagnosticStatus
from nav_msgs.msg import Odometry


def yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def pose_row(msg):
    p = msg.pose.pose.position
    return (msg.header.stamp.to_sec(), p.x, p.y, p.z, yaw(msg.pose.pose.orientation))


def write_csv(path, header, rows):
    with open(path, "w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(header)
        writer.writerows(rows)


def align_and_compare(localization, truth, max_dt):
    if not localization or not truth:
        return [], {}
    loc0, truth0 = localization[0], truth[0]
    heading_offset = truth0[4] - loc0[4]
    c, s = math.cos(heading_offset), math.sin(heading_offset)
    compared = []
    truth_index = 0
    for loc in localization:
        while (truth_index + 1 < len(truth) and
               abs(truth[truth_index + 1][0] - loc[0]) <=
               abs(truth[truth_index][0] - loc[0])):
            truth_index += 1
        gt = truth[truth_index]
        if abs(gt[0] - loc[0]) > max_dt:
            continue
        dx, dy = loc[1] - loc0[1], loc[2] - loc0[2]
        ax = truth0[1] + c * dx - s * dy
        ay = truth0[2] + s * dx + c * dy
        az = truth0[3] + loc[3] - loc0[3]
        ayaw = wrap(loc[4] + heading_offset)
        planar_error = math.hypot(ax - gt[1], ay - gt[2])
        yaw_error = abs(wrap(ayaw - gt[4]))
        compared.append((loc[0], ax, ay, az, ayaw, gt[1], gt[2], gt[3], gt[4],
                         planar_error, abs(az - gt[3]), yaw_error))
    if not compared:
        return [], {}
    planar = [row[9] for row in compared]
    vertical = [row[10] for row in compared]
    angular = [row[11] for row in compared]
    metrics = {
        "matched_samples": len(compared),
        "planar_rmse_m": math.sqrt(sum(value * value for value in planar) / len(planar)),
        "planar_mean_m": sum(planar) / len(planar),
        "planar_max_m": max(planar),
        "vertical_rmse_m": math.sqrt(sum(value * value for value in vertical) / len(vertical)),
        "yaw_rmse_deg": math.degrees(math.sqrt(sum(value * value for value in angular) / len(angular))),
        "endpoint_planar_error_m": planar[-1],
        "endpoint_yaw_error_deg": math.degrees(angular[-1]),
        "ground_truth_path_length_m": sum(
            math.hypot(truth[i][1] - truth[i - 1][1], truth[i][2] - truth[i - 1][2])
            for i in range(1, len(truth))),
    }
    return compared, metrics


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--duration", type=float, default=45.0)
    parser.add_argument("--localization-topic", default="/a1/localization/odom")
    parser.add_argument("--truth-topic", default="/ground_truth/base_w")
    parser.add_argument("--status-topic", default="/a1/localization/status")
    parser.add_argument("--max-time-difference", type=float, default=0.10)
    args = parser.parse_args(rospy.myargv()[1:])

    os.makedirs(args.output_dir, exist_ok=True)
    rospy.init_node("localization_validation_recorder", anonymous=True)
    localization, truth, statuses = [], [], []

    rospy.Subscriber(args.localization_topic, Odometry,
                     lambda message: localization.append(pose_row(message)), queue_size=100)
    rospy.Subscriber(args.truth_topic, Odometry,
                     lambda message: truth.append(pose_row(message)), queue_size=500)

    def status_callback(message):
        values = {item.key: item.value for item in message.values}
        statuses.append((rospy.Time.now().to_sec(), message.message,
                         values.get("reason", ""), values.get("results_valid", "")))

    rospy.Subscriber(args.status_topic, DiagnosticStatus, status_callback, queue_size=20)
    deadline = time.monotonic() + args.duration
    rate = rospy.Rate(20)
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        rate.sleep()

    write_csv(os.path.join(args.output_dir, "localization_trajectory.csv"),
              ("stamp", "x", "y", "z", "yaw_rad"), localization)
    write_csv(os.path.join(args.output_dir, "ground_truth_trajectory.csv"),
              ("stamp", "x", "y", "z", "yaw_rad"), truth)
    write_csv(os.path.join(args.output_dir, "status.csv"),
              ("stamp", "state", "reason", "results_valid"), statuses)
    compared, metrics = align_and_compare(localization, truth, args.max_time_difference)
    write_csv(os.path.join(args.output_dir, "aligned_trajectory.csv"),
              ("stamp", "localization_x", "localization_y", "localization_z",
               "localization_yaw", "truth_x", "truth_y", "truth_z", "truth_yaw",
               "planar_error_m", "vertical_error_m", "yaw_error_rad"), compared)
    metrics.update({
        "localization_samples": len(localization),
        "ground_truth_samples": len(truth),
        "status_samples": len(statuses),
        "duration_wall_sec": args.duration,
        "alignment": "initial planar pose (SE2)",
        "truth_usage": "acceptance-only; never republished or fed into localization",
    })
    with open(os.path.join(args.output_dir, "metrics.yaml"), "w") as stream:
        for key, value in metrics.items():
            stream.write("{}: {}\n".format(key, value))
    if not compared:
        rospy.logerr("no timestamp-matched localization/truth samples were recorded")
        return 2
    rospy.loginfo("validation artifacts written to %s", args.output_dir)
    return 0


if __name__ == "__main__":
    sys.exit(main())
