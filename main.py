import airsim
import cv2
import time
import os
import argparse
import subprocess

# Default path to the Unreal Engine simulator used during development
DEFAULT_UE4_PATH = r"C:\Users\newso\Documents\AirSimExperiments\BlocksBuild\WindowsNoEditor\Blocks\Binaries\Win64\Blocks.exe"

# Allow overriding the path via environment variable
ENV_UE4_PATH = os.environ.get("UE4_PATH")
ue4_default = ENV_UE4_PATH if ENV_UE4_PATH else DEFAULT_UE4_PATH

def main():
    parser = argparse.ArgumentParser(description="Optical flow navigation script")
    parser.add_argument("--manual-nudge", action="store_true", help="Enable manual nudge at frame 5 for testing")
    parser.add_argument(
        "--ue4-path",
        default=ue4_default,
        help="Path to the Unreal Engine executable (or set UE4_PATH env variable)",
    )
    args = parser.parse_args()

    from uav import DroneController, exit_flag, start_gui

    # GUI parameter and status holders
    param_refs = {
        'L': [0.0],
        'C': [0.0],
        'R': [0.0],
        'state': [''],
        'reset_flag': [False]
    }

    start_gui(param_refs)

    # === LAUNCH UE4 SIMULATION ===
    ue4_exe = args.ue4_path
    try:
        sim_process = subprocess.Popen([ue4_exe, "-windowed", "-ResX=1280", "-ResY=720"])
        print("Launching Unreal Engine simulation...")
        time.sleep(5)
    except Exception as e:
        print("Failed to launch UE4:", e)

    client = airsim.MultirotorClient()
    client.confirmConnection()
    print("Connected!")
    client.enableApiControl(True)
    client.armDisarm(True)

    # After takeoff
    client.takeoffAsync().join()
    client.moveToPositionAsync(0, 0, -2, 2).join()

    feature_params = dict(maxCorners=75, qualityLevel=0.1, minDistance=5, blockSize=5)
    lk_params = dict(winSize=(15, 15), maxLevel=2,
                     criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

    controller = DroneController(client)
    controller.initialize(lk_params, feature_params)
    try:
        controller.run()
    except KeyboardInterrupt:
        print("Interrupted.")
    finally:
        try:
            client.landAsync().join()
            client.armDisarm(False)
            client.enableApiControl(False)
        except Exception:
            pass


if __name__ == '__main__':
    main()
