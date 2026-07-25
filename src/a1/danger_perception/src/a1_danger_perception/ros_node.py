from __future__ import annotations

from typing import List, Optional, Tuple

import cv2
import message_filters
import numpy as np
import rospy
import tf2_geometry_msgs  # noqa: F401
import tf2_ros
from a1_navigation_interfaces.msg import DangerDetection, DangerDetectionArray
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import CameraInfo, Image

from .depth import DepthLocalizer, DepthParams
from .detector import Candidate, DetectorParams, OpenCVDangerDetector
from .tracker import DangerTracker, TrackerParams


class DangerPerceptionNode:
    def __init__(self) -> None:
        self.bridge = CvBridge()
        self.detector = OpenCVDangerDetector(self._load_detector_params())
        self.depth_localizer = DepthLocalizer(self._load_depth_params())
        self.use_tracker = bool(rospy.get_param("~use_tracker", True))
        self.tracker = DangerTracker(self._load_tracker_params())

        self.enable_tf = bool(rospy.get_param("~enable_tf", True))
        self.target_frame = str(rospy.get_param("~target_frame", "map"))
        self.tf_timeout = float(rospy.get_param("~tf_timeout", 0.10))
        self.publish_debug_images = bool(rospy.get_param("~publish_debug_images", True))
        self.draw_rejected = bool(rospy.get_param("~draw_rejected_candidates", True))
        self.log_throttle_s = float(rospy.get_param("~log_throttle_s", 2.0))

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.pub_detections = rospy.Publisher(
            str(rospy.get_param("~detections_topic", "/danger_perception/detections")),
            DangerDetectionArray,
            queue_size=5,
        )
        self.pub_debug_image = rospy.Publisher(
            str(rospy.get_param("~debug_image_topic", "/danger_perception/debug/detections_image")),
            Image,
            queue_size=2,
        )
        self.pub_red_mask = rospy.Publisher(
            str(rospy.get_param("~debug_red_mask_topic", "/danger_perception/debug/mask_red")),
            Image,
            queue_size=2,
        )
        self.pub_green_mask = rospy.Publisher(
            str(rospy.get_param("~debug_green_mask_topic", "/danger_perception/debug/mask_green")),
            Image,
            queue_size=2,
        )

        self.rgb_sub = message_filters.Subscriber(
            str(rospy.get_param("~rgb_topic", "/real_sense/rgb/image_raw")),
            Image,
        )
        self.depth_sub = message_filters.Subscriber(
            str(rospy.get_param("~depth_topic", "/real_sense/depth/image_raw")),
            Image,
        )
        self.depth_info_sub = message_filters.Subscriber(
            str(rospy.get_param("~depth_camera_info_topic", "/real_sense/depth/camera_info")),
            CameraInfo,
        )
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub, self.depth_info_sub],
            queue_size=int(rospy.get_param("~sync_queue_size", 8)),
            slop=float(rospy.get_param("~sync_slop", 0.08)),
        )
        self.sync.registerCallback(self._callback)

        rospy.loginfo(
            "a1_danger_perception started: rgb=%s depth=%s target_frame=%s enable_tf=%s",
            rospy.get_param("~rgb_topic", "/real_sense/rgb/image_raw"),
            rospy.get_param("~depth_topic", "/real_sense/depth/image_raw"),
            self.target_frame,
            self.enable_tf,
        )

    def _callback(self, rgb_msg: Image, depth_msg: Image, depth_info_msg: CameraInfo) -> None:
        try:
            bgr_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
            depth_m = self._depth_to_meters(depth_msg)
        except CvBridgeError as exc:
            rospy.logwarn_throttle(self.log_throttle_s, "cv_bridge conversion failed: %s", exc)
            return

        candidates = self.detector.detect(bgr_image)
        accepted_messages = []
        stamp = rgb_msg.header.stamp
        stamp_s = stamp.to_sec() if stamp else rospy.Time.now().to_sec()

        for candidate in candidates:
            if not candidate.is_danger:
                continue

            estimate = self.depth_localizer.estimate(candidate, depth_m, depth_info_msg, stamp)
            candidate.depth_median = estimate.depth_median
            candidate.depth_valid_ratio = estimate.depth_valid_ratio
            candidate.depth_std = estimate.depth_std
            if estimate.point is None:
                candidate.status = estimate.status
                continue

            point, status = self._transform_point(estimate.point)
            if point is None:
                candidate.status = status
                continue

            detection = self._candidate_to_msg(candidate, point, track_id=0, status=status)
            if self.use_tracker:
                track = self.tracker.update(
                    detection.class_name,
                    (detection.position.x, detection.position.y, detection.position.z),
                    detection.confidence,
                    stamp_s,
                )
                detection.track_id = track.track_id
            accepted_messages.append(detection)

        if self.use_tracker:
            accepted_messages = self._tracks_to_messages(stamp, stamp_s)

        self._publish_detections(stamp, accepted_messages)
        if self.publish_debug_images:
            self._publish_debug(rgb_msg.header, bgr_image, candidates)

        rospy.loginfo_throttle(
            self.log_throttle_s,
            "danger_perception candidates=%d published=%d",
            len(candidates),
            len(accepted_messages),
        )

    def _transform_point(self, camera_point: PointStamped) -> Tuple[Optional[PointStamped], str]:
        if not self.enable_tf:
            return camera_point, "ok_camera_frame"

        try:
            transformed = self.tf_buffer.transform(
                camera_point,
                self.target_frame,
                rospy.Duration(self.tf_timeout),
            )
            return transformed, "ok"
        except Exception as exc:
            rospy.logwarn_throttle(
                self.log_throttle_s,
                "TF transform %s -> %s failed: %s",
                camera_point.header.frame_id,
                self.target_frame,
                exc,
            )
            return None, "tf_unavailable"

    def _candidate_to_msg(
        self,
        candidate: Candidate,
        point: PointStamped,
        track_id: int,
        status: str,
    ) -> DangerDetection:
        msg = DangerDetection()
        msg.header = point.header
        msg.position = point.point
        msg.class_name = "danger_red_sphere"
        msg.confidence = float(candidate.confidence)
        msg.track_id = int(track_id)
        msg.is_valid = True
        msg.status = status
        return msg

    def _tracks_to_messages(self, stamp, stamp_s: float) -> List[DangerDetection]:
        messages: List[DangerDetection] = []
        for track in self.tracker.publishable_tracks(stamp_s):
            msg = DangerDetection()
            msg.header.stamp = stamp
            msg.header.frame_id = self.target_frame if self.enable_tf else "real_sense_optical_frame"
            msg.position.x = track.position[0]
            msg.position.y = track.position[1]
            msg.position.z = track.position[2]
            msg.class_name = "danger_red_sphere"
            msg.confidence = float(track.confidence)
            msg.track_id = int(track.track_id)
            msg.is_valid = True
            msg.status = "tracked"
            messages.append(msg)
        return messages

    def _publish_detections(self, stamp, detections: List[DangerDetection]) -> None:
        msg = DangerDetectionArray()
        msg.header.stamp = stamp
        msg.header.frame_id = detections[0].header.frame_id if detections else self.target_frame
        msg.detections = detections
        self.pub_detections.publish(msg)

    def _publish_debug(self, header, bgr_image: np.ndarray, candidates: List[Candidate]) -> None:
        debug = bgr_image.copy()
        for candidate in candidates:
            if candidate.is_danger:
                color = (0, 0, 255)
            elif self.draw_rejected:
                color = (0, 200, 255) if candidate.class_name == "red_box" else (0, 180, 0)
            else:
                continue

            x, y, w, h = candidate.bbox
            cv2.rectangle(debug, (x, y), (x + w, y + h), color, 2)
            label = "%s %.2f %s" % (candidate.class_name, candidate.confidence, candidate.status)
            cv2.putText(debug, label, (x, max(0, y - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)

        debug_msg = self.bridge.cv2_to_imgmsg(debug, encoding="bgr8")
        debug_msg.header = header
        self.pub_debug_image.publish(debug_msg)

        if self.detector.last_red_mask is not None:
            red_msg = self.bridge.cv2_to_imgmsg(self.detector.last_red_mask, encoding="mono8")
            red_msg.header = header
            self.pub_red_mask.publish(red_msg)
        if self.detector.last_green_mask is not None:
            green_msg = self.bridge.cv2_to_imgmsg(self.detector.last_green_mask, encoding="mono8")
            green_msg.header = header
            self.pub_green_mask.publish(green_msg)

    def _depth_to_meters(self, depth_msg: Image) -> np.ndarray:
        depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        depth_np = np.asarray(depth)
        if depth_np.dtype == np.uint16 or depth_msg.encoding in ("16UC1", "mono16"):
            return depth_np.astype(np.float32) * float(rospy.get_param("~depth_scale", 0.001))
        return depth_np.astype(np.float32)

    def _load_detector_params(self) -> DetectorParams:
        return DetectorParams(
            red_lower1=rospy.get_param("~red_hsv_lower1", [0, 80, 50]),
            red_upper1=rospy.get_param("~red_hsv_upper1", [10, 255, 255]),
            red_lower2=rospy.get_param("~red_hsv_lower2", [170, 80, 50]),
            red_upper2=rospy.get_param("~red_hsv_upper2", [179, 255, 255]),
            green_lower=rospy.get_param("~green_hsv_lower", [40, 60, 40]),
            green_upper=rospy.get_param("~green_hsv_upper", [85, 255, 255]),
            min_area=float(rospy.get_param("~min_area", 60.0)),
            max_area=float(rospy.get_param("~max_area", 200000.0)),
            min_aspect_ratio=float(rospy.get_param("~min_aspect_ratio", 0.55)),
            max_aspect_ratio=float(rospy.get_param("~max_aspect_ratio", 1.85)),
            min_circularity=float(rospy.get_param("~min_circularity", 0.65)),
            max_sphere_extent=float(rospy.get_param("~max_sphere_extent", 0.88)),
            morph_kernel_size=int(rospy.get_param("~morph_kernel_size", 5)),
            morph_iterations=int(rospy.get_param("~morph_iterations", 1)),
        )

    def _load_depth_params(self) -> DepthParams:
        return DepthParams(
            min_depth_m=float(rospy.get_param("~min_depth_m", 0.40)),
            max_depth_m=float(rospy.get_param("~max_depth_m", 8.0)),
            depth_percentile=float(rospy.get_param("~depth_percentile", 50.0)),
            min_depth_valid_ratio=float(rospy.get_param("~min_depth_valid_ratio", 0.35)),
            max_depth_std_m=float(rospy.get_param("~max_depth_std_m", 0.45)),
            estimate_sphere_center=bool(rospy.get_param("~estimate_sphere_center", True)),
            sphere_radius_m=float(rospy.get_param("~sphere_radius_m", 0.15)),
            depth_mask_erode=int(rospy.get_param("~depth_mask_erode", 1)),
            fallback_camera_frame=str(rospy.get_param("~fallback_camera_frame", "real_sense_optical_frame")),
        )

    def _load_tracker_params(self) -> TrackerParams:
        return TrackerParams(
            merge_distance_m=float(rospy.get_param("~merge_distance_m", 0.60)),
            position_alpha=float(rospy.get_param("~track_position_alpha", 0.35)),
            stale_after_s=float(rospy.get_param("~track_stale_after_s", 20.0)),
            min_observations=int(rospy.get_param("~min_track_observations", 1)),
            publish_min_confidence=float(rospy.get_param("~publish_min_confidence", 0.45)),
        )


def main() -> None:
    rospy.init_node("danger_perception")
    DangerPerceptionNode()
    rospy.spin()
