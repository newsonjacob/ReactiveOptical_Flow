# uav/utils.py
"""Utility helpers for AirSim drone state and image processing."""
import math
import cv2
import numpy as np
import airsim

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
