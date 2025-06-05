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


def retain_recent_views(analysis_dir: str, keep: int = 5) -> None:
    """Remove old flight view HTML files leaving only the newest ``keep``.

    Parameters
    ----------
    analysis_dir: str
        Directory containing ``flight_view_*.html`` files.
    keep: int, optional
        Number of most recent files to keep. Defaults to 5.
    """

    try:
        views = [
            os.path.join(analysis_dir, f)
            for f in os.listdir(analysis_dir)
            if f.startswith("flight_view_") and f.endswith(".html")
        ]
    except FileNotFoundError:
        return

    views.sort(key=os.path.getmtime, reverse=True)

    for old_view in views[keep:]:
        try:
            os.remove(old_view)
        except OSError:
            pass
