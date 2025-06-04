# uav/utils.py
import math
import cv2
import numpy as np
import airsim
import os

def apply_clahe(gray_image):
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    return clahe.apply(gray_image)

def get_yaw(orientation):
    return math.degrees(airsim.to_eularian_angles(orientation)[2])

def get_speed(velocity):
    return np.linalg.norm([velocity.x_val, velocity.y_val, velocity.z_val])

def get_drone_state(client):
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
