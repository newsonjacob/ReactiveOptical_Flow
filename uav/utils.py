# uav/utils.py
"""Utility helpers for AirSim drone state and image processing."""
import math
import cv2
import numpy as np
import airsim
import os

def apply_clahe(gray_image):
    """Improve contrast of a grayscale image using CLAHE."""
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    return clahe.apply(gray_image)

def get_yaw(orientation):
    """Return yaw angle in degrees from an AirSim quaternion."""
    return math.degrees(airsim.to_eularian_angles(orientation)[2])

def get_speed(velocity):
    """Compute the speed magnitude of a velocity vector."""
    return np.linalg.norm([velocity.x_val, velocity.y_val, velocity.z_val])

def get_drone_state(client):
    """Fetch position, yaw and speed from the AirSim client."""
    state = client.getMultirotorState()
    pos = state.kinematics_estimated.position
    ori = state.kinematics_estimated.orientation
    yaw = get_yaw(ori)
    vel = state.kinematics_estimated.linear_velocity
    speed = get_speed(vel)
    return pos, yaw, speed


def retain_recent_logs(log_dir: str, keep: int = 5) -> None:
    """Keep only the `keep` most recent log files in *log_dir*.

    Files are ordered by modification time. Older files beyond the
    desired count are silently removed.
    """

    try:
        logs = [
            os.path.join(log_dir, f)
            for f in os.listdir(log_dir)
            if f.endswith(".csv")
        ]
    except FileNotFoundError:
        return

    logs.sort(key=os.path.getmtime, reverse=True)

    for old_log in logs[keep:]:
        try:
            os.remove(old_log)
        except OSError:
            pass


# === Optical Flow Helpers ===

# Default fraction of the image height used for the "probe" region when
# evaluating optical flow.  This matches the previous behaviour of using
# the top third of the frame.
DEFAULT_PROBE_HEIGHT = 1 / 3


def compute_probe_mask(y_coords, height: int, fraction: float = DEFAULT_PROBE_HEIGHT):
    """Return a boolean mask selecting coordinates within the probe region.

    The mask spans ``fraction`` of the frame starting from the top.  ``y_coords``
    can be any iterable of y pixel values.  The function avoids numpy
    vectorisation so that tests work with the minimal numpy stub.
    """

    threshold = int(height * fraction + 0.5)
    return np.array([float(y) < threshold for y in y_coords])
