from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import cv2
import numpy as np


def _clamp(value: float, low: float = 0.0, high: float = 1.0) -> float:
    return max(low, min(high, value))


@dataclass
class DetectorParams:
    red_lower1: Sequence[int]
    red_upper1: Sequence[int]
    red_lower2: Sequence[int]
    red_upper2: Sequence[int]
    green_lower: Sequence[int]
    green_upper: Sequence[int]
    min_area: float
    max_area: float
    min_aspect_ratio: float
    max_aspect_ratio: float
    min_circularity: float
    min_enclosing_circle_fill: float
    max_sphere_extent: float
    morph_kernel_size: int
    morph_iterations: int


@dataclass
class Candidate:
    class_name: str
    is_danger: bool
    bbox: Tuple[int, int, int, int]
    center_uv: Tuple[float, float]
    contour: np.ndarray
    area: float
    perimeter: float
    circularity: float
    aspect_ratio: float
    extent: float
    enclosing_circle_fill: float
    confidence: float
    status: str
    depth_median: float = 0.0
    depth_valid_ratio: float = 0.0
    depth_std: float = 0.0


class OpenCVDangerDetector:
    """Detect red-sphere candidates and common distractors from RGB images."""

    def __init__(self, params: DetectorParams) -> None:
        self.params = params
        self.last_red_mask = None
        self.last_green_mask = None

    def detect(self, bgr_image: np.ndarray) -> List[Candidate]:
        hsv = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
        red_mask = self._red_mask(hsv)
        green_mask = self._single_mask(hsv, self.params.green_lower, self.params.green_upper)
        self.last_red_mask = red_mask
        self.last_green_mask = green_mask

        candidates: List[Candidate] = []
        candidates.extend(self._candidates_from_mask(red_mask, color_name="red"))
        candidates.extend(self._candidates_from_mask(green_mask, color_name="green"))
        return candidates

    def _red_mask(self, hsv: np.ndarray) -> np.ndarray:
        mask1 = self._single_mask(hsv, self.params.red_lower1, self.params.red_upper1)
        mask2 = self._single_mask(hsv, self.params.red_lower2, self.params.red_upper2)
        return cv2.bitwise_or(mask1, mask2)

    def _single_mask(self, hsv: np.ndarray, lower: Sequence[int], upper: Sequence[int]) -> np.ndarray:
        lower_np = np.array(lower, dtype=np.uint8)
        upper_np = np.array(upper, dtype=np.uint8)
        mask = cv2.inRange(hsv, lower_np, upper_np)

        kernel_size = max(1, int(self.params.morph_kernel_size))
        if kernel_size % 2 == 0:
            kernel_size += 1
        kernel = np.ones((kernel_size, kernel_size), np.uint8)
        iterations = max(0, int(self.params.morph_iterations))
        if iterations > 0:
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=iterations)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=iterations)
        return mask

    def _candidates_from_mask(self, mask: np.ndarray, color_name: str) -> List[Candidate]:
        contours_result = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = contours_result[0] if len(contours_result) == 2 else contours_result[1]

        candidates: List[Candidate] = []
        for contour in contours:
            candidate = self._candidate_from_contour(contour, color_name)
            if candidate is not None:
                candidates.append(candidate)
        return candidates

    def _candidate_from_contour(self, contour: np.ndarray, color_name: str) -> Optional[Candidate]:
        area = float(cv2.contourArea(contour))
        if area < self.params.min_area:
            return None

        x, y, w, h = cv2.boundingRect(contour)
        if w <= 0 or h <= 0:
            return None

        perimeter = float(cv2.arcLength(contour, True))
        circularity = 0.0 if perimeter <= 1e-6 else float(4.0 * np.pi * area / (perimeter * perimeter))
        aspect_ratio = float(w) / float(h)
        extent = area / float(w * h)
        (_, _), enclosing_radius = cv2.minEnclosingCircle(contour)
        enclosing_circle_area = float(np.pi * enclosing_radius * enclosing_radius)
        enclosing_circle_fill = 0.0 if enclosing_circle_area <= 1e-6 else area / enclosing_circle_area
        center_uv = self._contour_center(contour, x, y, w, h)

        status = "ok"
        class_name = "unknown"
        is_danger = False
        confidence = 0.0

        if area > self.params.max_area:
            status = "reject_area_large"
        elif not (self.params.min_aspect_ratio <= aspect_ratio <= self.params.max_aspect_ratio):
            status = "reject_aspect_ratio"
        elif color_name == "green":
            class_name = "green_sphere"
            status = "reject_green_distractor"
            confidence = 0.20
        elif color_name == "red":
            looks_round = circularity >= self.params.min_circularity
            fills_enclosing_circle = enclosing_circle_fill >= self.params.min_enclosing_circle_fill
            looks_not_boxy = extent <= self.params.max_sphere_extent
            if looks_round and fills_enclosing_circle and looks_not_boxy:
                class_name = "red_sphere"
                is_danger = True
                confidence = self._shape_confidence(area, circularity, aspect_ratio, extent, enclosing_circle_fill)
            else:
                class_name = "red_box"
                if not looks_round:
                    status = "reject_circularity"
                elif not fills_enclosing_circle:
                    status = "reject_enclosing_circle_fill"
                else:
                    status = "reject_red_box_or_non_round"
                confidence = 0.25

        return Candidate(
            class_name=class_name,
            is_danger=is_danger,
            bbox=(int(x), int(y), int(w), int(h)),
            center_uv=center_uv,
            contour=contour,
            area=area,
            perimeter=perimeter,
            circularity=circularity,
            aspect_ratio=aspect_ratio,
            extent=extent,
            enclosing_circle_fill=enclosing_circle_fill,
            confidence=confidence,
            status=status,
        )

    def _shape_confidence(
        self,
        area: float,
        circularity: float,
        aspect_ratio: float,
        extent: float,
        enclosing_circle_fill: float,
    ) -> float:
        circular_score = _clamp(
            (circularity - self.params.min_circularity)
            / max(1e-6, 1.0 - self.params.min_circularity)
        )
        circle_fill_score = _clamp(
            (enclosing_circle_fill - self.params.min_enclosing_circle_fill)
            / max(1e-6, 1.0 - self.params.min_enclosing_circle_fill)
        )
        aspect_score = _clamp(1.0 - abs(aspect_ratio - 1.0) / 0.75)
        extent_score = _clamp(1.0 - max(0.0, extent - 0.78) / max(1e-6, self.params.max_sphere_extent - 0.78))
        area_score = _clamp((area - self.params.min_area) / max(1.0, self.params.max_area - self.params.min_area))
        return _clamp(
            0.30
            + 0.25 * circular_score
            + 0.20 * circle_fill_score
            + 0.15 * aspect_score
            + 0.05 * extent_score
            + 0.05 * area_score
        )

    @staticmethod
    def _contour_center(
        contour: np.ndarray,
        x: int,
        y: int,
        w: int,
        h: int,
    ) -> Tuple[float, float]:
        moments = cv2.moments(contour)
        if abs(moments["m00"]) > 1e-6:
            return float(moments["m10"] / moments["m00"]), float(moments["m01"] / moments["m00"])
        return float(x + w * 0.5), float(y + h * 0.5)
