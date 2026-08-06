#!/usr/bin/env python3
"""Passive MP4 recorder for the A1 exploration demo dashboard."""
from __future__ import annotations

import json
import math
import os
import time
from collections import defaultdict, deque
from types import SimpleNamespace

import cv2
import numpy as np
import rospy
from a1_navigation_interfaces.msg import (
    DangerDetectionArray,
    ExplorationStatus,
    MissionStatus,
)
from cv_bridge import CvBridge, CvBridgeError
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import String

try:
    from PIL import Image as PILImage
    from PIL import ImageDraw, ImageFont
except Exception:
    PILImage = None
    ImageDraw = None
    ImageFont = None


BGR_BG = (11, 16, 24)
BGR_PANEL = (18, 25, 36)
BGR_BORDER = (57, 76, 98)
BGR_ACCENT = (82, 210, 232)
BGR_ORANGE = (0, 177, 255)
BGR_RED = (54, 72, 245)
RGB_WHITE = (232, 238, 246)
RGB_MUTED = (148, 162, 179)
RGB_ACCENT = (100, 224, 242)
RGB_RED = (255, 96, 96)


EXP_STATES = {
    ExplorationStatus.IDLE: "Idle",
    ExplorationStatus.SELECTING_TARGET: "Selecting Target",
    ExplorationStatus.NAVIGATING: "Navigating",
    ExplorationStatus.COMPLETED: "Exploration Complete",
    ExplorationStatus.FAILED: "Failed",
    ExplorationStatus.PAUSED: "Paused",
    ExplorationStatus.RECORD_START: "Record Start",
    ExplorationStatus.UPDATE_COVERAGE: "Update Coverage",
    ExplorationStatus.EXPLORATION_DONE: "No Reachable Frontier",
    ExplorationStatus.RETURNING: "Returning",
    ExplorationStatus.RETURNED: "Returned",
    ExplorationStatus.CANCELLED: "Cancelled",
    ExplorationStatus.REQUEST_ENTRY_DOOR_OPEN: "Request Door Open",
    ExplorationStatus.TRANSIT_TO_ENTRY: "Transit To Entry",
    ExplorationStatus.ENTERED_FLOOR: "Entered Floor",
}
MISSION_STATES = {
    MissionStatus.INIT: "Init",
    MissionStatus.WAIT_LOCALIZATION: "Wait Localization",
    MissionStatus.WAIT_MAP: "Wait Map",
    MissionStatus.EXPLORING: "Exploring",
    MissionStatus.APPROACHING_DANGER: "Approaching Danger",
    MissionStatus.CROSSING_FLOOR: "Crossing Floor",
    MissionStatus.RECOVERING: "Recovering",
    MissionStatus.PAUSED: "Paused",
    MissionStatus.FINISHED: "Finished",
    MissionStatus.ERROR: "Error",
}
class Text:
    def __init__(self):
        self.cache = {}
        self.paths = [
            "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
            "/usr/share/fonts/opentype/noto/NotoSansCJKsc-Regular.otf",
            "/usr/share/fonts/truetype/noto/NotoSansCJK-Regular.ttc",
            "/usr/share/fonts/truetype/wqy/wqy-microhei.ttc",
            "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc",
            "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        ]

    def font(self, size):
        if ImageFont is None:
            return None
        if size in self.cache:
            return self.cache[size]
        for path in self.paths:
            if os.path.exists(path):
                try:
                    self.cache[size] = ImageFont.truetype(path, size)
                    return self.cache[size]
                except Exception:
                    pass
        self.cache[size] = ImageFont.load_default()
        return self.cache[size]

    def draw(self, image, items):
        if not items:
            return image
        if PILImage is None or ImageDraw is None:
            for text, x, y, size, rgb in items:
                ascii_text = text.encode("ascii", "ignore").decode("ascii")
                cv2.putText(image, ascii_text, (x, y + size),
                            cv2.FONT_HERSHEY_SIMPLEX, max(0.45, size / 28.0),
                            (rgb[2], rgb[1], rgb[0]), 1, cv2.LINE_AA)
            return image
        pil = PILImage.fromarray(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
        draw = ImageDraw.Draw(pil)
        for text, x, y, size, rgb in items:
            draw.text((x, y), text, font=self.font(size), fill=rgb)
        return cv2.cvtColor(np.asarray(pil), cv2.COLOR_RGB2BGR)


class ImageSlot:
    def __init__(self, bridge, name):
        self.bridge = bridge
        self.name = name
        self.frame = None
        self.wall_stamp = 0.0
        self.warned = False

    def callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            if not self.warned:
                rospy.logwarn("%s conversion failed: %s", self.name, exc)
                self.warned = True
            return
        if frame is None:
            return
        if frame.ndim == 2:
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        elif frame.shape[2] == 4:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
        self.frame = np.ascontiguousarray(frame)
        self.wall_stamp = time.time()

    def recent(self, max_age):
        return self.frame is not None and time.time() - self.wall_stamp <= max_age


class MissionVideoRecorder:
    def __init__(self):
        self.width = int(rospy.get_param("~width", 1280))
        self.height = int(rospy.get_param("~height", 720))
        self.fps = float(rospy.get_param("~fps", 8.0))
        self.output = str(rospy.get_param(
            "~output", "/workspace/SimEnv/results/mission_video/mission_video.mp4"))
        self.duration_s = float(rospy.get_param("~duration_s", 0.0))
        self.mission_timeout_s = float(rospy.get_param("~mission_timeout_s", 600.0))
        self.image_max_age = float(rospy.get_param("~image_max_age_s", 2.0))
        self.danger_merge_m = float(rospy.get_param("~danger_merge_m", 0.65))

        os.makedirs(os.path.dirname(self.output), exist_ok=True)
        self.writer = cv2.VideoWriter(
            self.output, cv2.VideoWriter_fourcc(*"mp4v"), self.fps,
            (self.width, self.height))
        if not self.writer.isOpened():
            raise rospy.ROSException("failed to open video writer: %s" % self.output)

        self.bridge = CvBridge()
        self.text = Text()
        self.first = ImageSlot(self.bridge, "first-person")
        self.first_fallback = ImageSlot(self.bridge, "first-person fallback")
        self.third = ImageSlot(self.bridge, "third-person")
        self.maps = {}
        self.paths = defaultdict(lambda: deque(maxlen=2200))
        self.pose_by_floor = {}
        self.target_by_floor = {}
        self.dangers = {}
        self.exploration = None
        self.mission = None
        self.ros_start = None
        self.wall_start = time.time()
        self.frames = 0

        rospy.Subscriber(rospy.get_param("~first_person_topic",
                                          "/danger_perception/debug/detections_image"),
                         Image, self.first.callback, queue_size=1, buff_size=2 ** 24)
        rospy.Subscriber(rospy.get_param("~first_person_fallback_topic",
                                          "/real_sense/rgb/image_raw"),
                         Image, self.first_fallback.callback, queue_size=1,
                         buff_size=2 ** 24)
        rospy.Subscriber(rospy.get_param("~third_person_topic",
                                          "/a1/third_person/image_raw"),
                         Image, self.third.callback, queue_size=1, buff_size=2 ** 24)
        rospy.Subscriber(rospy.get_param("~map_topic", "/a1/floor_mapping/map"),
                         OccupancyGrid, self.map_callback, queue_size=1)
        rospy.Subscriber(rospy.get_param("~odom_topic", "/a1/localization/odom"),
                         Odometry, self.odom_callback, queue_size=20)
        rospy.Subscriber(rospy.get_param("~exploration_status_topic",
                                          "/a1/exploration/status"),
                         ExplorationStatus, self.exploration_callback, queue_size=10)
        rospy.Subscriber(rospy.get_param("~mission_status_topic",
                                          "/a1/mission_manager/status"),
                         String, self.mission_callback, queue_size=10)
        rospy.Subscriber(rospy.get_param("~danger_detections_topic",
                                          "/danger_perception/detections"),
                         DangerDetectionArray, self.danger_callback, queue_size=10)
        rospy.on_shutdown(self.close)
        rospy.Timer(rospy.Duration(1.0 / max(0.1, self.fps)), self.tick)
        rospy.loginfo("mission_video_recorder writing %dx%d@%.1f to %s",
                      self.width, self.height, self.fps, self.output)

    def active_floor(self):
        if self.mission is not None and self.mission.current_floor_id >= 0:
            return int(self.mission.current_floor_id)
        if self.exploration is not None and self.exploration.floor_id >= 0:
            return int(self.exploration.floor_id)
        return 0

    def elapsed(self):
        now = rospy.Time.now()
        if not now.is_zero():
            if self.ros_start is None:
                self.ros_start = now.to_sec()
            return max(0.0, now.to_sec() - self.ros_start)
        return time.time() - self.wall_start

    def map_callback(self, msg):
        w = int(msg.info.width)
        h = int(msg.info.height)
        if w <= 0 or h <= 0 or len(msg.data) != w * h:
            return
        self.maps[self.active_floor()] = {
            "data": np.asarray(msg.data, dtype=np.int16).reshape((h, w)),
            "res": float(msg.info.resolution),
            "ox": float(msg.info.origin.position.x),
            "oy": float(msg.info.origin.position.y),
        }

    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        floor = self.active_floor()
        self.pose_by_floor[floor] = (x, y, yaw)
        path = self.paths[floor]
        if not path or math.hypot(path[-1][0] - x, path[-1][1] - y) > 0.04:
            path.append((x, y, yaw))

    def exploration_callback(self, msg):
        self.exploration = msg
        target = msg.current_target.pose.position
        if math.isfinite(target.x) and math.isfinite(target.y):
            self.target_by_floor[self.active_floor()] = (float(target.x), float(target.y))

    def mission_callback(self, msg):
        try:
            payload = json.loads(msg.data)
        except (TypeError, ValueError):
            self.mission = SimpleNamespace(
                state=str(msg.data),
                current_floor_id=self.active_floor(),
                active_goal="",
                message=str(msg.data),
            )
            return
        self.mission = SimpleNamespace(
            state=payload.get("state", ""),
            current_floor_id=int(payload.get("floor", self.active_floor())),
            active_goal=str(payload.get("active_goal", "")),
            message=str(payload.get("detail", payload.get("message", ""))),
        )

    def danger_callback(self, msg):
        floor = self.active_floor()
        for det in msg.detections:
            if not det.is_valid or det.confidence < 0.45:
                continue
            if det.class_name == "sphere_candidate":
                continue
            if det.class_name and "danger" not in det.class_name:
                continue
            x = float(det.position.x)
            y = float(det.position.y)
            if not (math.isfinite(x) and math.isfinite(y)):
                continue
            key = ("track", int(det.track_id)) if det.track_id else (
                "cell", round(x / self.danger_merge_m), round(y / self.danger_merge_m), floor)
            self.dangers[key] = (x, y, floor, float(det.confidence))

    @staticmethod
    def cover(img, w, h):
        if img is None or img.size == 0:
            return np.full((h, w, 3), BGR_PANEL, np.uint8)
        ih, iw = img.shape[:2]
        scale = max(w / float(iw), h / float(ih))
        rw, rh = max(1, int(iw * scale)), max(1, int(ih * scale))
        resized = cv2.resize(img, (rw, rh), interpolation=cv2.INTER_AREA)
        x0, y0 = max(0, (rw - w) // 2), max(0, (rh - h) // 2)
        return resized[y0:y0 + h, x0:x0 + w].copy()

    @staticmethod
    def contain(img, w, h, bg=BGR_PANEL):
        canvas = np.full((h, w, 3), bg, np.uint8)
        if img is None or img.size == 0:
            return canvas
        ih, iw = img.shape[:2]
        scale = min(w / float(iw), h / float(ih))
        rw, rh = max(1, int(iw * scale)), max(1, int(ih * scale))
        resized = cv2.resize(img, (rw, rh), interpolation=cv2.INTER_NEAREST)
        x0, y0 = (w - rw) // 2, (h - rh) // 2
        canvas[y0:y0 + rh, x0:x0 + rw] = resized
        return canvas

    def world_to_cell(self, meta, x, y):
        return (int(math.floor((x - meta["ox"]) / meta["res"])),
                int(math.floor((y - meta["oy"]) / meta["res"])))

    def draw_robot(self, canvas, center, yaw, size):
        x, y = center
        theta = -yaw
        pts = np.asarray([
            (x + int(math.cos(theta) * size), y + int(math.sin(theta) * size)),
            (x + int(math.cos(theta + 2.45) * size * 0.72),
             y + int(math.sin(theta + 2.45) * size * 0.72)),
            (x + int(math.cos(theta - 2.45) * size * 0.72),
             y + int(math.sin(theta - 2.45) * size * 0.72)),
        ], dtype=np.int32)
        cv2.fillConvexPoly(canvas, pts, BGR_ORANGE, cv2.LINE_AA)
        cv2.circle(canvas, center, max(3, size // 4), (18, 23, 30), -1, cv2.LINE_AA)

    def map_image(self, floor, w, h, follow=False):
        canvas = np.full((h, w, 3), BGR_PANEL, np.uint8)
        meta = self.maps.get(floor)
        if meta is None:
            cv2.rectangle(canvas, (1, 1), (w - 2, h - 2), BGR_BORDER, 1)
            return canvas
        data = meta["data"]
        mh, mw = data.shape
        xs, ys = [], []
        known_y, known_x = np.where(data >= 0)
        if known_x.size:
            xs += [int(known_x.min()), int(known_x.max())]
            ys += [int(known_y.min()), int(known_y.max())]
        for x, y, _ in self.paths.get(floor, []):
            cx, cy = self.world_to_cell(meta, x, y)
            xs.append(cx)
            ys.append(cy)
        if floor in self.pose_by_floor:
            cx, cy = self.world_to_cell(meta, self.pose_by_floor[floor][0],
                                        self.pose_by_floor[floor][1])
            xs.append(cx)
            ys.append(cy)
        if floor in self.target_by_floor:
            cx, cy = self.world_to_cell(meta, self.target_by_floor[floor][0],
                                        self.target_by_floor[floor][1])
            xs.append(cx)
            ys.append(cy)
        for x, y, dfloor, _ in self.dangers.values():
            if dfloor == floor:
                cx, cy = self.world_to_cell(meta, x, y)
                xs.append(cx)
                ys.append(cy)
        if follow and floor in self.pose_by_floor:
            cx, cy = self.world_to_cell(meta, self.pose_by_floor[floor][0],
                                        self.pose_by_floor[floor][1])
            r = max(10, int(7.0 / meta["res"]))
            x0, x1, y0, y1 = cx - r, cx + r, cy - r, cy + r
        elif xs and ys:
            margin = max(10, int(1.2 / meta["res"]))
            x0, x1 = min(xs) - margin, max(xs) + margin
            y0, y1 = min(ys) - margin, max(ys) + margin
        else:
            x0, x1, y0, y1 = 0, mw - 1, 0, mh - 1
        x0, x1 = max(0, min(mw - 1, x0)), max(0, min(mw - 1, x1))
        y0, y1 = max(0, min(mh - 1, y0)), max(0, min(mh - 1, y1))
        if x1 <= x0 or y1 <= y0:
            x0, x1, y0, y1 = 0, mw - 1, 0, mh - 1
        crop = data[y0:y1 + 1, x0:x1 + 1]
        rgb = np.zeros((crop.shape[0], crop.shape[1], 3), dtype=np.uint8)
        rgb[:] = (19, 25, 34)
        rgb[(crop >= 0) & (crop < 50)] = (205, 211, 218)
        rgb[(crop >= 50) & (crop < 65)] = (126, 138, 150)
        rgb[crop >= 65] = (24, 28, 34)
        rgb = np.flipud(rgb)
        inner = self.contain(rgb, w - 10, h - 10, (14, 18, 25))
        canvas[5:h - 5, 5:w - 5] = inner
        scale = min((w - 10) / float(crop.shape[1]),
                    (h - 10) / float(crop.shape[0]))
        drawn_w, drawn_h = int(crop.shape[1] * scale), int(crop.shape[0] * scale)
        ox, oy = 5 + (w - 10 - drawn_w) // 2, 5 + (h - 10 - drawn_h) // 2

        def to_px(wx, wy):
            cx, cy = self.world_to_cell(meta, wx, wy)
            if cx < x0 or cx > x1 or cy < y0 or cy > y1:
                return None
            return (int(ox + (cx - x0) * scale), int(oy + (y1 - cy) * scale))

        points = [p for p in (to_px(x, y) for x, y, _ in self.paths.get(floor, []))
                  if p is not None]
        if len(points) > 1:
            cv2.polylines(canvas, [np.asarray(points, np.int32)], False,
                          (230, 190, 70), 2, cv2.LINE_AA)
        if floor in self.target_by_floor:
            point = to_px(*self.target_by_floor[floor])
            if point is not None:
                px, py = point
                cv2.drawMarker(canvas, (px, py), BGR_ACCENT,
                               cv2.MARKER_DIAMOND, 18, 2, cv2.LINE_AA)
        for x, y, dfloor, _ in self.dangers.values():
            if dfloor == floor:
                point = to_px(x, y)
                if point is not None:
                    cv2.circle(canvas, point, 8, BGR_RED, -1, cv2.LINE_AA)
                    cv2.circle(canvas, point, 13, BGR_RED, 2, cv2.LINE_AA)
        if floor in self.pose_by_floor:
            point = to_px(self.pose_by_floor[floor][0], self.pose_by_floor[floor][1])
            if point is not None:
                self.draw_robot(canvas, point, self.pose_by_floor[floor][2],
                                max(12, int(16 * min(1.4, scale))))
        cv2.rectangle(canvas, (1, 1), (w - 2, h - 2), BGR_BORDER, 1)
        return canvas

    def first_frame(self):
        if self.first.recent(self.image_max_age):
            return self.first.frame, "First-person: robot camera"
        if self.first_fallback.recent(self.image_max_age):
            return self.first_fallback.frame, "First-person: robot camera"
        return None, "First-person: waiting for camera"

    def third_frame(self, w, h):
        if self.third.recent(self.image_max_age):
            return self.cover(self.third.frame, w, h), "Third-person: follow camera"
        return self.map_image(self.active_floor(), w, h, follow=True), "Third-person: map fallback"

    def status_rows(self):
        floor = self.active_floor()
        coverage = 0.0
        stage = "Waiting"
        message = ""
        if self.exploration is not None:
            coverage = max(0.0, min(1.0, float(self.exploration.coverage_ratio)))
            stage = EXP_STATES.get(int(self.exploration.state),
                                   "Exploration State %d" % int(self.exploration.state))
            message = self.exploration.message
        if self.mission is not None:
            if isinstance(self.mission.state, str):
                stage = self.mission.state.replace("_", " ").title()
            else:
                stage = MISSION_STATES.get(
                    int(self.mission.state),
                    "Mission State %d" % int(self.mission.state))
            if self.mission.active_goal:
                stage += " / " + self.mission.active_goal
            if self.mission.message:
                message = self.mission.message
        return floor, coverage, stage, message

    def tick(self, _event):
        elapsed = self.elapsed()
        if self.duration_s > 0.0 and elapsed > self.duration_s:
            rospy.signal_shutdown("mission video duration reached")
            return
        frame = np.full((self.height, self.width, 3), BGR_BG, np.uint8)
        text = []
        top_h = int(self.height * 0.58)
        half_w = self.width // 2
        first, first_label = self.first_frame()
        left = self.cover(first, half_w - 8, top_h)
        right, third_label = self.third_frame(self.width - half_w - 8, top_h)
        frame[:top_h, :half_w - 8] = left
        frame[:top_h, half_w + 8:] = right
        cv2.line(frame, (half_w, 0), (half_w, top_h), (0, 0, 0), 3)
        cv2.line(frame, (0, top_h), (self.width, top_h), (0, 0, 0), 3)
        self.label_bg(frame, 16, 10, 245, 34)
        self.label_bg(frame, half_w + 24, 10, 250, 34)
        text.append((first_label, 24, 15, 20, RGB_WHITE))
        text.append((third_label, half_w + 32, 15, 20, RGB_WHITE))

        bottom_y = top_h + 8
        bottom_h = self.height - bottom_y - 14
        map_x, map_y = 24, bottom_y + 8
        map_w, map_h = 430, bottom_h - 8
        floors = [0, 1, 2]
        panel_gap = 8
        title_h = 28
        panel_w = (map_w - panel_gap * 2) // 3
        active_floor = self.active_floor()
        for i, floor in enumerate(floors):
            x = map_x + i * (panel_w + panel_gap)
            y = map_y + title_h
            panel = self.map_image(floor, panel_w, map_h - title_h)
            frame[y:y + panel.shape[0], x:x + panel.shape[1]] = panel
            text.append(("Floor %d" % (floor + 1), x + 6, map_y + 2, 17,
                         RGB_ACCENT if floor == active_floor else RGB_MUTED))
            if floor == active_floor:
                cv2.rectangle(frame, (x, y), (x + panel_w - 1, y + map_h - title_h - 1),
                              BGR_ACCENT, 2)

        floor, coverage, stage, message = self.status_rows()
        info_x = map_x + map_w + 48
        rows = [
            ("Mission Clock", "%4.1f s / %.0f s  (%d%%)" % (
                elapsed, self.mission_timeout_s,
                int(100.0 * min(1.0, elapsed / max(1.0, self.mission_timeout_s))))),
            ("Stage", stage),
            ("Current Floor", "Floor %d" % (floor + 1)),
            ("Map Coverage", "%d%%" % int(coverage * 100.0)),
            ("Confirmed Dangers", str(len(self.dangers))),
        ]
        for idx, (label, value) in enumerate(rows):
            yy = bottom_y + 34 + idx * 38
            text.append((label, info_x, yy, 20, RGB_MUTED))
            text.append((value[:44], info_x + 180, yy - 2,
                         24 if idx == 0 else 22,
                         RGB_RED if idx == 4 else RGB_WHITE))
        if message:
            text.append((message[:58], info_x + 180, bottom_y + 226, 17, RGB_MUTED))
        bar_x, bar_y = info_x, self.height - 72
        bar_w, bar_h = self.width - info_x - 36, 18
        cv2.rectangle(frame, (bar_x, bar_y), (bar_x + bar_w, bar_y + bar_h),
                      (49, 62, 80), -1)
        cv2.rectangle(frame, (bar_x, bar_y),
                      (bar_x + int(bar_w * coverage), bar_y + bar_h),
                      (54, 130, 95), -1)
        cv2.rectangle(frame, (bar_x, bar_y),
                      (bar_x + int(bar_w * min(1.0, elapsed / max(1.0, self.mission_timeout_s))),
                       bar_y + bar_h), BGR_ACCENT, 2)
        text.append(("Coverage fill / time outline", bar_x, bar_y + 24, 15, RGB_MUTED))
        self.writer.write(self.text.draw(frame, text))
        self.frames += 1

    @staticmethod
    def label_bg(frame, x, y, w, h):
        overlay = frame.copy()
        cv2.rectangle(overlay, (x, y), (x + w, y + h), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.42, frame, 0.58, 0.0, dst=frame)

    def close(self):
        if self.writer is not None:
            self.writer.release()
            rospy.loginfo("mission_video_recorder closed %s after %d frames",
                          self.output, self.frames)
            self.writer = None


def main():
    rospy.init_node("mission_video_recorder")
    MissionVideoRecorder()
    rospy.spin()


if __name__ == "__main__":
    main()
