#!/usr/bin/env python3
"""Summarize one DEV-ONLY constant-velocity stability bag as JSON."""

import argparse
import json
import math
import os
import tempfile

import rosbag


JOINTS = (
    "FR_hip",
    "FR_thigh",
    "FR_calf",
    "FL_hip",
    "FL_thigh",
    "FL_calf",
    "RR_hip",
    "RR_thigh",
    "RR_calf",
    "RL_hip",
    "RL_thigh",
    "RL_calf",
)
FEET = ("FR", "FL", "RR", "RL")
TILT_THRESHOLDS_DEG = (5.0, 10.0, 20.0, 35.0, 45.0)


def quaternion_to_rpy(quaternion):
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w
    roll = math.atan2(
        2.0 * (w * x + y * z),
        1.0 - 2.0 * (x * x + y * y),
    )
    sin_pitch = 2.0 * (w * y - z * x)
    pitch = (
        math.copysign(math.pi / 2.0, sin_pitch)
        if abs(sin_pitch) >= 1.0
        else math.asin(sin_pitch)
    )
    yaw = math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )
    return roll, pitch, yaw


def update_extrema(metrics, message, command):
    if command:
        values = {
            "q": message.q,
            "dq": message.dq,
            "tau": message.tau,
            "kp": message.Kp,
            "kd": message.Kd,
        }
    else:
        values = {
            "q": message.q,
            "dq": message.dq,
            "ddq": message.ddq,
            "tau_est": message.tauEst,
        }
    metrics["samples"] += 1
    for name, value in values.items():
        metrics["min"][name] = min(metrics["min"].get(name, value), value)
        metrics["max"][name] = max(metrics["max"].get(name, value), value)
        metrics["max_abs"][name] = max(metrics["max_abs"].get(name, 0.0), abs(value))


def blank_joint_metrics():
    return {"samples": 0, "min": {}, "max": {}, "max_abs": {}}


def foot_summary(samples, crossing_time, contact_threshold):
    if not samples:
        return {"samples": 0}
    contact_samples = 0
    longest_no_contact = 0.0
    no_contact_start = None
    force_norm_max = 0.0
    lateral_max = 0.0
    z_min = samples[0][3]
    z_max = samples[0][3]
    pre_crossing = []
    for stamp, fx, fy, fz in samples:
        force_norm = math.sqrt(fx * fx + fy * fy + fz * fz)
        lateral = math.hypot(fx, fy)
        force_norm_max = max(force_norm_max, force_norm)
        lateral_max = max(lateral_max, lateral)
        z_min = min(z_min, fz)
        z_max = max(z_max, fz)
        if force_norm >= contact_threshold:
            contact_samples += 1
            if no_contact_start is not None:
                longest_no_contact = max(longest_no_contact, stamp - no_contact_start)
                no_contact_start = None
        elif no_contact_start is None:
            no_contact_start = stamp
        if crossing_time is not None and crossing_time - 0.2 <= stamp <= crossing_time:
            pre_crossing.append((fx, fy, fz))
    if no_contact_start is not None:
        longest_no_contact = max(longest_no_contact, samples[-1][0] - no_contact_start)
    pre_crossing_mean = None
    if pre_crossing:
        pre_crossing_mean = {
            "fx": sum(item[0] for item in pre_crossing) / len(pre_crossing),
            "fy": sum(item[1] for item in pre_crossing) / len(pre_crossing),
            "fz": sum(item[2] for item in pre_crossing) / len(pre_crossing),
        }
    return {
        "samples": len(samples),
        "force_norm_max": force_norm_max,
        "lateral_force_max": lateral_max,
        "force_z_min": z_min,
        "force_z_max": z_max,
        "contact_fraction": float(contact_samples) / len(samples),
        "longest_below_threshold_s": longest_no_contact,
        "mean_force_last_0_2s_before_35deg": pre_crossing_mean,
        "last_force": {
            "fx": samples[-1][1],
            "fy": samples[-1][2],
            "fz": samples[-1][3],
        },
    }


def analyze(path, contact_threshold):
    command_start = None
    seen_unlock = False
    relock_time = None
    start_pose = None
    last_pose = None
    crossings = {}
    max_abs_roll = 0.0
    max_abs_pitch = 0.0
    max_output_cmd = [0.0, 0.0, 0.0]
    final_output_cmd = None
    rtf_values = []
    foot_samples = {foot: [] for foot in FEET}
    command_metrics = {joint: blank_joint_metrics() for joint in JOINTS}
    state_metrics = {joint: blank_joint_metrics() for joint in JOINTS}

    foot_topics = {
        "/visual/{}_foot_contact/the_force".format(foot): foot for foot in FEET
    }
    command_topics = {
        "/a1_gazebo/{}_controller/command".format(joint): joint for joint in JOINTS
    }
    state_topics = {
        "/a1_gazebo/{}_controller/state".format(joint): joint for joint in JOINTS
    }
    topics = set(foot_topics)
    topics.update(command_topics)
    topics.update(state_topics)
    topics.update(
        (
            "/Odometry_gazebo",
            "/cmd_vel",
            "/a1_cmd_mux/safety_lock",
            "/a1/navigation_tests/stability_diagnostic",
        )
    )

    with rosbag.Bag(path, "r") as bag:
        bag_start = bag.get_start_time()
        bag_end = bag.get_end_time()
        for topic, message, bag_stamp in bag.read_messages(topics=topics):
            stamp = bag_stamp.to_sec()
            if topic == "/cmd_vel":
                vector = (
                    message.linear.x,
                    message.linear.y,
                    message.angular.z,
                )
                final_output_cmd = vector
                for index, value in enumerate(vector):
                    max_output_cmd[index] = max(max_output_cmd[index], abs(value))
                if command_start is None and any(abs(value) > 1e-3 for value in vector):
                    command_start = stamp
                continue
            if topic == "/a1_cmd_mux/safety_lock":
                if not message.data:
                    seen_unlock = True
                elif seen_unlock and relock_time is None:
                    relock_time = stamp
                continue
            if topic == "/a1/navigation_tests/stability_diagnostic":
                for status in message.status:
                    for value in status.values:
                        if value.key == "rtf":
                            try:
                                rtf = float(value.value)
                                if rtf > 0.0:
                                    rtf_values.append(rtf)
                            except ValueError:
                                pass
                continue
            if topic == "/Odometry_gazebo":
                roll, pitch, yaw = quaternion_to_rpy(message.pose.pose.orientation)
                pose = {
                    "t": stamp,
                    "x": message.pose.pose.position.x,
                    "y": message.pose.pose.position.y,
                    "z": message.pose.pose.position.z,
                    "roll_deg": math.degrees(roll),
                    "pitch_deg": math.degrees(pitch),
                    "yaw_deg": math.degrees(yaw),
                }
                last_pose = pose
                max_abs_roll = max(max_abs_roll, abs(roll))
                max_abs_pitch = max(max_abs_pitch, abs(pitch))
                if command_start is not None:
                    if start_pose is None:
                        start_pose = pose
                    tilt_deg = math.degrees(max(abs(roll), abs(pitch)))
                    for threshold in TILT_THRESHOLDS_DEG:
                        key = str(int(threshold))
                        if key not in crossings and tilt_deg >= threshold:
                            crossings[key] = {
                                "dt_from_command_s": stamp - command_start,
                                "pose": pose,
                            }
                continue
            if command_start is None or "35" in crossings:
                continue
            if topic in foot_topics:
                wrench = message.wrench
                foot_samples[foot_topics[topic]].append(
                    (
                        stamp,
                        wrench.force.x,
                        wrench.force.y,
                        wrench.force.z,
                    )
                )
            elif topic in command_topics:
                update_extrema(command_metrics[command_topics[topic]], message, True)
            elif topic in state_topics:
                update_extrema(state_metrics[state_topics[topic]], message, False)

    crossing_time = None
    if command_start is not None and "35" in crossings:
        crossing_time = command_start + crossings["35"]["dt_from_command_s"]
    return {
        "schema_version": 1,
        "dev_only": True,
        "bag": os.path.abspath(path),
        "bag_duration_s": bag_end - bag_start,
        "command_start_s": command_start,
        "relock_dt_from_command_s": None
        if command_start is None or relock_time is None
        else relock_time - command_start,
        "start_pose": start_pose,
        "last_pose": last_pose,
        "tilt_crossings_deg": crossings,
        "max_abs_roll_deg": math.degrees(max_abs_roll),
        "max_abs_pitch_deg": math.degrees(max_abs_pitch),
        "max_abs_cmd_vel": {
            "vx": max_output_cmd[0],
            "vy": max_output_cmd[1],
            "wz": max_output_cmd[2],
        },
        "final_cmd_vel": final_output_cmd,
        "diagnostic_rtf_min": min(rtf_values) if rtf_values else None,
        "diagnostic_rtf_max": max(rtf_values) if rtf_values else None,
        "foot_force_contact_threshold_n": contact_threshold,
        "foot_force_until_35deg": {
            foot: foot_summary(samples, crossing_time, contact_threshold)
            for foot, samples in foot_samples.items()
        },
        "joint_command_until_35deg": command_metrics,
        "joint_state_until_35deg": state_metrics,
    }


def write_json(path, result):
    directory = os.path.dirname(os.path.abspath(path))
    os.makedirs(directory, exist_ok=True)
    with tempfile.NamedTemporaryFile(
        mode="w", dir=directory, delete=False, prefix=".bag_analysis_", suffix=".json"
    ) as temporary:
        json.dump(result, temporary, indent=2, sort_keys=True)
        temporary.write("\n")
        temporary_path = temporary.name
    os.replace(temporary_path, path)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    parser.add_argument("--output")
    parser.add_argument("--contact-threshold-n", type=float, default=5.0)
    arguments = parser.parse_args()
    result = analyze(arguments.bag, arguments.contact_threshold_n)
    if arguments.output:
        write_json(arguments.output, result)
    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
