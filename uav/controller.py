# uav/controller.py
"""High level drone control wrapper used by ``main.py``.

This module exposes a :class:`DroneController` that bundles
image capture, optical flow processing and navigation decisions.
The implementation reuses the existing ``Navigator`` and
``OpticalFlowTracker`` helpers.
"""

from __future__ import annotations

import time
import os
import csv
from typing import Tuple

import cv2
import numpy as np
import airsim

from .perception import OpticalFlowTracker, FlowHistory
from .navigation import Navigator
from .utils import (
    get_drone_state,
    should_flat_wall_dodge,
    FLOW_STD_MAX,
    retain_recent_logs,
)
from .interface import exit_flag


class DroneController:
    """Coordinate frame capture, processing and navigation."""

    def __init__(self, client: airsim.MultirotorClient) -> None:
        self.client = client
        self.tracker: OpticalFlowTracker | None = None
        self.history = FlowHistory()
        self.navigator = Navigator(client)
        self.frame_count = 0
        self.param_refs = {
            "L": [0.0],
            "C": [0.0],
            "R": [0.0],
            "state": [""],
        }
        self.log_path = ""
        self._header_written = False

    def initialize(self, lk_params: dict, feature_params: dict) -> None:
        """Initialise the optical flow tracker."""
        self.tracker = OpticalFlowTracker(lk_params, feature_params)

    # ------------------------------------------------------------------
    def capture_frame(self) -> Tuple[np.ndarray, float, float]:
        """Return the next grayscale frame and timing info."""
        t0 = time.time()
        responses = self.client.simGetImages([
            airsim.ImageRequest("oakd_camera", airsim.ImageType.Scene, False, True)
        ])
        t_fetch_end = time.time()
        response = responses[0]
        img1d = np.frombuffer(response.image_data_uint8, dtype=np.uint8)
        img = cv2.imdecode(img1d, cv2.IMREAD_COLOR)
        t_decode_end = time.time()
        if img is None:
            raise RuntimeError("Failed to decode image")
        img = cv2.resize(img, (640, 480))
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        return gray, t_fetch_end - t0, t_decode_end - t_fetch_end

    # ------------------------------------------------------------------
    def process_frame(self, gray: np.ndarray) -> Tuple[np.ndarray, np.ndarray, float, float, float, float]:
        """Update optical flow and return smoothed magnitudes."""
        assert self.tracker is not None
        good_old, flow_vectors, flow_std = self.tracker.process_frame(gray, time.time())
        if flow_vectors.ndim < 2:
            magnitudes = np.array([])
        else:
            magnitudes = np.linalg.norm(flow_vectors, axis=1)
        h, w = gray.shape
        if len(good_old) == 0:
            self.history.update(0.0, 0.0, 0.0)
            return good_old, flow_vectors, flow_std, 0.0, 0.0, 0.0
        good_old = good_old.reshape(-1, 2)
        x_coords = good_old[:, 0]
        y_coords = good_old[:, 1]
        left_mag = np.mean(magnitudes[x_coords < w // 3]) if np.any(x_coords < w // 3) else 0.0
        center_band = (x_coords >= w // 3) & (x_coords < 2 * w // 3)
        right_mag = np.mean(magnitudes[x_coords >= 2 * w // 3]) if np.any(x_coords >= 2 * w // 3) else 0.0
        center_mag = np.mean(magnitudes[center_band]) if np.any(center_band) else 0.0
        self.history.update(left_mag, center_mag, right_mag)
        smooth_L, smooth_C, smooth_R = self.history.average()
        self.param_refs["L"][0] = smooth_L
        self.param_refs["C"][0] = smooth_C
        self.param_refs["R"][0] = smooth_R
        return good_old, flow_vectors, flow_std, smooth_L, smooth_C, smooth_R

    # ------------------------------------------------------------------
    def decide_action(
        self,
        points: np.ndarray,
        smooth_L: float,
        smooth_C: float,
        smooth_R: float,
        probe_mag: float = 0.0,
        probe_count: int = 0,
        flow_std: float = 0.0,
        time_now: float | None = None,
    ) -> str:
        """Return the navigator decision for the current frame."""
        time_now = time_now or time.time()
        if len(points) < 5:
            if smooth_L > 1.5 and smooth_R > 1.5 and smooth_C < 0.2:
                state = self.navigator.brake()
            else:
                state = self.navigator.blind_forward()
            self.param_refs["state"][0] = state
            return state

        pos, yaw, speed = get_drone_state(self.client)
        brake_thres = 20 + 10 * speed
        dodge_thres = 2 + 0.5 * speed

        center_high = smooth_C > dodge_thres or smooth_C > 2 * min(smooth_L, smooth_R)
        side_diff = abs(smooth_L - smooth_R)
        side_safe = side_diff > 0.3 * smooth_C and (smooth_L < 100 or smooth_R < 100)

        probe_reliable = probe_count > 5 and probe_mag > 0.05
        in_grace_period = time_now < self.navigator.grace_period_end_time

        state = "none"
        if smooth_C > (brake_thres * 1.5):
            state = self.navigator.brake()
            self.navigator.grace_period_end_time = time_now + 2.5
        elif not in_grace_period:
            if smooth_C > brake_thres:
                state = self.navigator.brake()
                self.navigator.grace_period_end_time = time_now + 2.5
            elif center_high and side_safe:
                state = self.navigator.dodge(smooth_L, smooth_C, smooth_R)
                self.navigator.grace_period_end_time = time_now + 2.5
            elif probe_mag < 0.5 and should_flat_wall_dodge(
                smooth_C, probe_mag, probe_count, 5, flow_std, FLOW_STD_MAX
            ):
                state = self.navigator.dodge(smooth_L, smooth_C, smooth_R)
                self.navigator.grace_period_end_time = time_now + 2.5

        if state == "none":
            if (
                (self.navigator.braked or self.navigator.dodging)
                and smooth_C < 10
                and smooth_L < 10
                and smooth_R < 10
            ):
                state = self.navigator.resume_forward()
            elif not self.navigator.braked and not self.navigator.dodging and time_now - self.navigator.last_movement_time > 2:
                state = self.navigator.reinforce()
            elif (
                (self.navigator.braked or self.navigator.dodging)
                and speed < 0.2
                and smooth_C < 5
                and smooth_L < 5
                and smooth_R < 5
            ):
                state = self.navigator.nudge()
            elif time_now - self.navigator.last_movement_time > 4:
                state = self.navigator.timeout_recover()

        self.param_refs["state"][0] = state
        return state

    # ------------------------------------------------------------------
    def log_frame(self, state: str, **info) -> None:
        """Append frame data to the CSV log file."""
        if not self.log_path:
            return

        info.setdefault("frame", self.frame_count)
        info.setdefault("state", state)

        if not self._header_written:
            self._log_fields = list(info.keys())
            with open(self.log_path, "w", newline="") as f:
                writer = csv.DictWriter(f, fieldnames=self._log_fields)
                writer.writeheader()
                writer.writerow(info)
            self._header_written = True
        else:
            with open(self.log_path, "a", newline="") as f:
                writer = csv.DictWriter(f, fieldnames=self._log_fields)
                writer.writerow(info)

    # ------------------------------------------------------------------
    def run(self) -> None:
        """Example run loop demonstrating method usage."""
        if self.tracker is None:
            raise RuntimeError("initialize() must be called before run()")
        os.makedirs("flow_logs", exist_ok=True)
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        self.log_path = os.path.join("flow_logs", f"full_log_{timestamp}.csv")
        self._header_written = False
        try:
            while not exit_flag.is_set():
                gray, *_ = self.capture_frame()
                pts, vecs, std, sL, sC, sR = self.process_frame(gray)
                state = self.decide_action(pts, sL, sC, sR)
                self.log_frame(state)
                self.frame_count += 1
        finally:
            retain_recent_logs("flow_logs")

