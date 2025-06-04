# uav/perception.py
"""Perception utilities for computing optical flow and tracking history."""
import cv2
import numpy as np
import time
from collections import deque

class FlowHistory:
    """Maintain a rolling window of recent flow magnitudes."""
    def __init__(self, size=5):
        """Create a buffer storing the last *size* flow measurements."""
        self.size = size
        self.window = deque(maxlen=size)

    def update(self, left, center, right):
        """Append a new triple of left/center/right values."""
        self.window.append(np.array([left, center, right]))

    def average(self):
        """Return the mean of the stored flow readings."""
        if not self.window:
            return 0.0, 0.0, 0.0
        arr = np.array(self.window)
        return tuple(arr.mean(axis=0))

class OpticalFlowTracker:
    """Track sparse optical flow features between frames."""
    def __init__(self, lk_params, feature_params):
        """Initialize tracker with Lucas-Kanade and feature parameters."""
        self.lk_params = lk_params
        self.feature_params = feature_params
        self.prev_gray = None
        self.prev_pts = None
        self.prev_time = time.time()

    def initialize(self, gray_frame):
        """Start tracking using the provided grayscale frame."""
        self.prev_gray = gray_frame
        self.prev_pts = cv2.goodFeaturesToTrack(gray_frame, mask=None, **self.feature_params)
        self.prev_time = time.time()

    def process_frame(self, gray, _unused_start_time):  # ignore external time
        """Track features in *gray* and return vectors with standard deviation."""
        if self.prev_gray is None or self.prev_pts is None:
            self.initialize(gray)
            return np.array([]), np.array([]), 0.0

        next_pts, status, err = cv2.calcOpticalFlowPyrLK(self.prev_gray, gray, self.prev_pts, None, **self.lk_params)

        if next_pts is None or status is None:
            self.initialize(gray)
            return np.array([]), np.array([]), 0.0

        good_old = self.prev_pts[status.flatten() == 1]
        good_new = next_pts[status.flatten() == 1]

        current_time = time.time()
        dt = max(current_time - self.prev_time, 1e-6)  # avoid div by zero
        self.prev_time = current_time

        self.prev_gray = gray
        self.prev_pts = cv2.goodFeaturesToTrack(gray, mask=None, **self.feature_params)

        if len(good_old) == 0:
            return np.array([]), np.array([]), 0.0

        flow_vectors = good_new - good_old
        magnitudes = np.linalg.norm(flow_vectors, axis=1) / dt  # pixels/sec
        flow_std = np.std(magnitudes)

        return good_old, flow_vectors, flow_std


