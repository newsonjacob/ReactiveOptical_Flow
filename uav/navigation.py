# uav/navigation.py
"""Navigation utilities for issuing motion commands to an AirSim drone."""
import time
import math
import airsim

class Navigator:
    """Issue high level movement commands and track state."""
    def __init__(self, client):
        """Store the AirSim client used for all commands."""
        self.client = client
        self.braked = False
        self.dodging = False
        self.last_movement_time = time.time()

    def get_state(self):
        """Return the drone position, yaw angle and speed."""
        state = self.client.getMultirotorState()
        pos = state.kinematics_estimated.position
        ori = state.kinematics_estimated.orientation
        yaw = math.degrees(airsim.to_eularian_angles(ori)[2])
        vel = state.kinematics_estimated.linear_velocity
        speed = math.sqrt(vel.x_val ** 2 + vel.y_val ** 2 + vel.z_val ** 2)
        return pos, yaw, speed

    def brake(self):
        """Stop the drone immediately."""
        print("🛑 Braking")
        self.client.moveByVelocityAsync(0, 0, 0, 1).join()
        self.braked = True
        return "brake"

    def dodge(self, smooth_L, smooth_C, smooth_R):
        """Perform a lateral dodge based on flow magnitudes."""
        if smooth_L < smooth_R - 10:
            direction = "left"
        elif smooth_R < smooth_L - 10:
            direction = "right"
        else:
            print("❌ Dodge ambiguous — skipping")
            return "no_dodge"

        lateral = 1.0 if direction == "right" else -1.0
        strength = 0.5 if max(smooth_L, smooth_R) > 100 else 1.0

        # Cut existing motion before dodge
        self.client.moveByVelocityBodyFrameAsync(0, 0, 0, 0.2).join()  # brief stop

        # Decide forward speed
        forward_speed = 0.0 if smooth_C > 1.0 else 0.3

        print(f"🔀 Dodging {direction} (strength {strength:.1f}, forward {forward_speed:.1f})")
        self.client.moveByVelocityBodyFrameAsync(
            forward_speed,
            lateral * strength,
            0,
            2.0
        ).join()

        self.dodging = True
        self.braked = False
        self.last_movement_time = time.time()
        return f"dodge_{direction}"


    def resume_forward(self):
        """Resume normal forward velocity."""
        print("✅ Resuming forward motion")
        self.client.moveByVelocityAsync(2, 0, 0, duration=3,
            drivetrain=airsim.DrivetrainType.ForwardOnly,
            yaw_mode=airsim.YawMode(False, 0))
        self.braked = False
        self.dodging = False
        self.last_movement_time = time.time()
        return "resume"

    def blind_forward(self):
        """Move forward when no features are detected."""
        print("⚠️ No features — continuing blind forward motion")
        self.client.moveByVelocityAsync(2, 0, 0, duration=2,
            drivetrain=airsim.DrivetrainType.ForwardOnly,
            yaw_mode=airsim.YawMode(False, 0))
        self.last_movement_time = time.time()
        return "blind_forward"

    def nudge(self):
        """Gently push the drone forward when stalled."""
        print("⚠️ Low flow + zero velocity — nudging forward")
        self.client.moveByVelocityAsync(0.5, 0, 0, 1).join()
        self.last_movement_time = time.time()
        return "nudge"

    def reinforce(self):
        """Reissue the forward command to reinforce motion."""
        print("🔁 Reinforcing forward motion")
        self.client.moveByVelocityAsync(2, 0, 0, duration=3,
            drivetrain=airsim.DrivetrainType.ForwardOnly,
            yaw_mode=airsim.YawMode(False, 0))
        self.last_movement_time = time.time()
        return "resume_reinforce"

    def timeout_recover(self):
        """Move slowly forward after a command timeout."""
        print("⏳ Timeout — forcing recovery motion")
        self.client.moveByVelocityAsync(0.5, 0, 0, 1).join()
        self.last_movement_time = time.time()
        return "timeout_nudge"

