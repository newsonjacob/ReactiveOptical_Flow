import airsim
import cv2
import numpy as np
import time
from datetime import datetime
import os
import subprocess
import math
from airsim import ImageRequest, ImageType
import argparse
from queue import Queue
from threading import Thread

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

    from uav.interface import exit_flag, start_gui
    from uav.perception import OpticalFlowTracker, FlowHistory
    from uav.navigation import Navigator
    from uav.utils import get_drone_state, retain_recent_logs, should_flat_wall_dodge
    from analysis.utils import retain_recent_views

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

    tracker = OpticalFlowTracker(lk_params, feature_params)
    flow_history = FlowHistory()
    navigator = Navigator(client)
    from collections import deque
    state_history = deque(maxlen=3)
    pos_history = deque(maxlen=3)

    frame_count = 0
    start_time = time.time()
    MAX_SIM_DURATION = 60  # seconds
    GOAL_X = 29  # distance from start in AirSim coordinates
    GOAL_RADIUS = 1.0  # meters
    MIN_PROBE_FEATURES = 5
    FLOW_STD_MAX = 10.0
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    os.makedirs("flow_logs", exist_ok=True)
    log_file = open(f"flow_logs/full_log_{timestamp}.csv", 'w')
    log_file.write(
        "frame,time,features,flow_left,flow_center,flow_right,"
        "flow_std,pos_x,pos_y,pos_z,yaw,speed,state,collided,obstacle,side_safe,"
        "brake_thres,dodge_thres,probe_req,fps,simgetimage_s,decode_s,processing_s,loop_s\n"
    )
    retain_recent_logs("flow_logs")

    # Video writer setup
    fourcc = cv2.VideoWriter_fourcc(*'MJPG')
    out = cv2.VideoWriter('flow_output.avi', fourcc, 8.0, (640, 480))

    # Offload video writing to a background thread
    frame_queue: Queue = Queue(maxsize=20)

    def video_worker() -> None:
        while not exit_flag.is_set() or not frame_queue.empty():
            frame = frame_queue.get()
            if frame is None:
                break
            out.write(frame)
            frame_queue.task_done()

    video_thread = Thread(target=video_worker, daemon=True)
    video_thread.start()

    # Buffer log lines to throttle disk writes
    log_buffer = []
    LOG_INTERVAL = 5  # flush every 5 frames

    last_vis_img = np.zeros((480, 640, 3), dtype=np.uint8)  # Black frame as fallback

    target_fps = 20
    frame_duration = 1.0 / target_fps

    fps_list = []
    img = None  # Add this before your main loop

    try:
        loop_start = time.time()
        while not exit_flag.is_set():
            frame_count += 1
            time_now = time.time()  # <-- Add this line
            # Handle settle phase after dodge
            if navigator.settling:
                if time_now < navigator.settle_end_time:
                    ...
                    # Log settle state
                    pos, yaw, speed = get_drone_state(client)
                    collision = client.simGetCollisionInfo()
                    collided = int(getattr(collision, "has_collided", False))

                    log_buffer.append(
                        f"{frame_count},{time_now:.2f},0,0,0,0,0,"
                        f"{pos.x_val:.2f},{pos.y_val:.2f},{pos.z_val:.2f},{yaw:.2f},{speed:.2f},settling,{collided},0,"
                        f"0,0,0,0,0,0,0,{time.time() - loop_start:.3f}\n"
                    )

                    if frame_count % LOG_INTERVAL == 0:
                        log_file.writelines(log_buffer)
                        log_buffer.clear()

                    try:
                        frame_queue.put_nowait(vis_img)
                    except Exception:
                        pass

                    # Reset loop timing so loop_s reflects the settle delay only
                    loop_start = time.time()

                    continue

                else:
                    print("✅ Settle period over — resuming evaluation")
                    navigator.settling = False

            if time_now - start_time >= MAX_SIM_DURATION:
                print("⏱️ Time limit reached — landing and stopping.")
                break

            pos_goal, _, _ = get_drone_state(client)
            if pos_goal.x_val >= GOAL_X - GOAL_RADIUS:
                print("\U0001F3C1 Goal reached — landing.")
                break

            # --- Get image from AirSim ---
            t0 = time.time()
            responses = client.simGetImages([
                ImageRequest("oakd_camera", ImageType.Scene, False, True)  # compress=True for JPEG
            ])
            t_fetch_end = time.time()
            response = responses[0]
            if response.width == 0 or response.height == 0 or len(response.image_data_uint8) == 0:
                print("⚠️ Invalid image frame — skipping visual processing")
                pos, yaw, speed = get_drone_state(client)
                collision = client.simGetCollisionInfo()
                collided = int(getattr(collision, "has_collided", False))

                log_buffer.append(
                    f"{frame_count},{time_now:.2f},0,0,0,0,0,"
                    f"{pos.x_val:.2f},{pos.y_val:.2f},{pos.z_val:.2f},{yaw:.2f},{speed:.2f},no_image,{collided},0,"
                    f"0,0,0,0,0,0,0,{time.time() - loop_start:.3f}\n"
                )

                try:
                    frame_queue.put_nowait(last_vis_img)
                except Exception:
                    pass

                if frame_count % LOG_INTERVAL == 0:
                    log_file.writelines(log_buffer)
                    log_buffer.clear()

                continue

            img1d = np.frombuffer(response.image_data_uint8, dtype=np.uint8)
            img = cv2.imdecode(img1d, cv2.IMREAD_COLOR)
            t_decode_end = time.time()
            simgetimage_s = t_fetch_end - t0
            decode_s = t_decode_end - t_fetch_end
            if img is None:
                try:
                    frame_queue.put_nowait(last_vis_img)
                except Exception:
                    pass
                continue

            img = cv2.resize(img, (640, 480))
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            vis_img = img.copy()
            last_vis_img = vis_img  # Update after a valid frame

            if frame_count == 1:
                tracker.initialize(gray)
                print("Initialized optical flow tracker.")
                frame_queue.put(vis_img)
                continue

            if args.manual_nudge and frame_count == 5:
                print("🔧 Manual nudge forward for test")
                client.moveByVelocityAsync(2, 0, 0, 2)

            # --- Optical flow processing ---
            good_old, flow_vectors, flow_std = tracker.process_frame(gray, start_time)
            magnitudes = np.linalg.norm(flow_vectors, axis=1)
            h, w = gray.shape
            good_old = good_old.reshape(-1, 2)  # Ensure proper shape
            x_coords = good_old[:, 0]
            y_coords = good_old[:, 1]

            left_mag = np.mean(magnitudes[x_coords < w // 3]) if np.any(x_coords < w // 3) else 0
            center_band = (x_coords >= w // 3) & (x_coords < 2 * w // 3)
            right_mag = np.mean(magnitudes[x_coords >= 2 * w // 3]) if np.any(x_coords >= 2 * w // 3) else 0
            probe_band = y_coords < h // 3
            probe_mag = np.mean(magnitudes[center_band & probe_band]) if np.any(center_band & probe_band) else 0
            probe_count = int(np.sum(center_band & probe_band))
            center_mag = np.mean(magnitudes[center_band]) if np.any(center_band) else 0

            flow_history.update(left_mag, center_mag, right_mag)
            smooth_L, smooth_C, smooth_R = flow_history.average()
            param_refs['L'][0] = smooth_L
            param_refs['C'][0] = smooth_C
            param_refs['R'][0] = smooth_R

            # Draw flow arrows for visualization
            for i, (p1, vec) in enumerate(zip(good_old, flow_vectors)):
                if i > 50:
                    break
                x1, y1 = int(p1[0]), int(p1[1])
                vec = np.ravel(vec)  # Ensure vec is 1D
                if vec.shape[0] >= 2:
                    dx, dy = float(vec[0]), float(vec[1])
                else:
                    dx, dy = 0.0, 0.0
                x2, y2 = int(x1 + dx), int(y1 + dy)
                cv2.arrowedLine(vis_img, (x1, y1), (x2, y2), (0, 255, 0), 1, tipLength=0.3)

            # Overlay info
            pos, yaw, speed = get_drone_state(client)
            cv2.putText(vis_img, f"Frame: {frame_count}", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
            cv2.putText(vis_img, f"Speed: {speed:.2f}", (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
            cv2.putText(vis_img, f"State: {param_refs['state'][0]}", (10, 85), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
            cv2.putText(vis_img, f"Sim Time: {time_now-start_time:.2f}s", (10, 115), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)

            # === Navigation logic ===
            state_str = "none"
            brake_thres = 0.0
            dodge_thres = 0.0
            probe_req = 0.0
            side_safe = False

            if len(good_old) < 5: #If the number of "good" feature points tracked by the optical flow algorithm is less than 5, then...
                if smooth_L > 1.5 and smooth_R > 1.5 and smooth_C < 0.2:
                    state_str = navigator.brake()
                else:
                    state_str = navigator.blind_forward()
            else:
                pos, yaw, speed = get_drone_state(client)

                # Adaptive thresholds tuned for quicker reactions
                brake_thres = 20 + 10 * speed
                dodge_thres = 2 + 0.5 * speed

                center_high = smooth_C > dodge_thres or smooth_C > 2 * min(smooth_L, smooth_R)
                side_diff = abs(smooth_L - smooth_R)
                side_safe = side_diff > 0.3 * smooth_C and (smooth_L < 100 or smooth_R < 100)

                probe_reliable = probe_count > MIN_PROBE_FEATURES and probe_mag > 0.05
                in_grace_period = time_now < navigator.grace_period_end_time

                # === Priority 1: Severe brake override (ignores grace period)
                if smooth_C > (brake_thres * 1.5):
                    state_str = navigator.brake()
                    navigator.grace_period_end_time = time_now + 2.5

                elif not in_grace_period:
                    # === Brake Logic
                    if smooth_C > brake_thres:
                        state_str = navigator.brake()
                        navigator.grace_period_end_time = time_now + 2.5
                    # elif not probe_reliable and center_mag > 0.7:
                    #     print("⚠️ No probe flow but high center flow — possible wall ahead")
                    #     state_str = navigator.brake()
                    #     navigator.grace_period_end_time = time_now + 2.5

                    # === Dodge Logic
                    elif center_high and side_safe:
                        state_str = navigator.dodge(smooth_L, smooth_C, smooth_R)
                        navigator.grace_period_end_time = time_now + 2.5
                    elif probe_mag < 0.5 and center_mag > 0.7:
                        if should_flat_wall_dodge(center_mag, probe_mag, probe_count, MIN_PROBE_FEATURES, flow_std, FLOW_STD_MAX):
                            print("🟥 Flat wall detected — attempting fallback dodge")
                            state_str = navigator.dodge(smooth_L, smooth_C, smooth_R)
                            navigator.grace_period_end_time = time_now + 2.5
                        else:
                            print("🔬 Insufficient probe features — ignoring fallback")

                # === Recovery / Maintenance States (always allowed)
                if state_str == "none":
                    if (navigator.braked or navigator.dodging) and smooth_C < 10 and smooth_L < 10 and smooth_R < 10:
                        state_str = navigator.resume_forward()
                    elif not navigator.braked and not navigator.dodging and time_now - navigator.last_movement_time > 2:
                        state_str = navigator.reinforce()
                    elif (navigator.braked or navigator.dodging) and speed < 0.2 and smooth_C < 5 and smooth_L < 5 and smooth_R < 5:
                        state_str = navigator.nudge()
                    elif time_now - navigator.last_movement_time > 4:
                        state_str = navigator.timeout_recover()

            param_refs['state'][0] = state_str
            obstacle_detected = int('dodge' in state_str or state_str == 'brake')



            # === Detect repeated dodges with minimal progress ===
            pos_hist, _, _ = get_drone_state(client)
            state_history.append(state_str)
            pos_history.append((pos_hist.x_val, pos_hist.y_val))
            if len(state_history) == state_history.maxlen:
                if all(s == state_history[-1] for s in state_history) and state_history[-1].startswith("dodge"):
                    dx = pos_history[-1][0] - pos_history[0][0]
                    dy = pos_history[-1][1] - pos_history[0][1]
                    if abs(dx) < 0.5 and abs(dy) < 1.0:
                        print("♻️ Repeated dodges detected — extending dodge")
                        state_str = navigator.dodge(smooth_L, smooth_C, smooth_R, duration=3.0)
                        state_history[-1] = state_str
                        param_refs['state'][0] = state_str

            # === Reset logic from GUI ===
            if param_refs['reset_flag'][0]:
                print("🔄 Resetting simulation...")
                try:
                    client.landAsync().join()
                    client.reset()
                    client.enableApiControl(True)
                    client.armDisarm(True)
                    client.takeoffAsync().join()
                    client.moveToPositionAsync(0, 0, -2, 2).join()
                except Exception as e:
                    print("Reset error:", e)

                tracker.initialize(gray)
                flow_history = FlowHistory()
                navigator = Navigator(client)
                frame_count = 0
                param_refs['reset_flag'][0] = False

                # === Reset log file ===
                if log_buffer:
                    log_file.writelines(log_buffer)
                    log_buffer.clear()
                log_file.close()
                timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
                log_file = open(f"flow_logs/full_log_{timestamp}.csv", 'w')
                log_file.write(
                    "frame,time,features,flow_left,flow_center,flow_right,"
                    "flow_std,pos_x,pos_y,pos_z,yaw,speed,state,collided,obstacle,side_safe,"
                    "brake_thres,dodge_thres,probe_req,fps,simgetimage_s,decode_s,processing_s,loop_s\n"
                )
                retain_recent_logs("flow_logs")

                # === Reset video writer ===
                frame_queue.put(None)
                video_thread.join()
                out.release()
                out = cv2.VideoWriter('flow_output.avi', fourcc, 8.0, (640, 480))
                video_thread = Thread(target=video_worker, daemon=True)
                video_thread.start()
                continue

            # Queue frame for async video writing
            try:
                frame_queue.put_nowait(vis_img)
            except Exception:
                pass

            # Throttle loop to target FPS
            elapsed = time.time() - loop_start
            if elapsed < frame_duration:
                time.sleep(frame_duration - elapsed)
            loop_elapsed = time.time() - loop_start
            actual_fps = 1 / max(loop_elapsed, 1e-6)
            loop_start = time.time()

            fps_list.append(actual_fps)

            pos, yaw, speed = get_drone_state(client)
            collision = client.simGetCollisionInfo()
            collided = int(getattr(collision, "has_collided", False))

            log_buffer.append(
                f"{frame_count},{time_now:.2f},{len(good_old)},"
                f"{smooth_L:.3f},{smooth_C:.3f},{smooth_R:.3f},{flow_std:.3f},"
                f"{pos.x_val:.2f},{pos.y_val:.2f},{pos.z_val:.2f},{yaw:.2f},{speed:.2f},{state_str},{collided},{obstacle_detected},{int(side_safe)},"
                f"{brake_thres:.2f},{dodge_thres:.2f},{probe_req:.2f},{actual_fps:.2f},"
                f"{simgetimage_s:.3f},{decode_s:.3f},0.0,{loop_elapsed:.3f}\n"
            )
            if frame_count % LOG_INTERVAL == 0:
                log_file.writelines(log_buffer)
                log_buffer.clear()

            print(f"Actual FPS: {actual_fps:.2f}")
            print(f"Features detected: {len(good_old)}")

    except KeyboardInterrupt:
        print("Interrupted.")

    finally:
        print("Landing...")
        if log_buffer:
            log_file.writelines(log_buffer)
            log_buffer.clear()
        log_file.close()
        frame_queue.put(None)
        video_thread.join()
        out.release()
        try:
            client.landAsync().join()
            client.armDisarm(False)
            client.enableApiControl(False)
        except Exception as e:
            print("Landing error:", e)
        # === Auto-generate 3D flight visualisation ===
        try:
            html_output = f"analysis/flight_view_{timestamp}.html"
            subprocess.run([
                "python", "analysis/visualize_flight.py",
                "--log", f"flow_logs/full_log_{timestamp}.csv",
                "--obstacles", "analysis/obstacles.json",
                "--output", html_output,
                "--scale", "1.0"
            ])
            print(f"✅ 3D visualisation saved to {html_output}")
            retain_recent_views("analysis")
        except Exception as e:
            print(f"⚠️ Visualization failed: {e}")

        if sim_process:
            sim_process.terminate()
            print("UE4 simulation closed.")

        # Re-encode video at median FPS using OpenCV
        import statistics
        if len(fps_list) > 0:
            median_fps = statistics.median(fps_list)
            print(f"Median FPS: {median_fps:.2f}")

            input_video = 'flow_output.avi'
            output_video = 'flow_output_fixed.avi'

            cap = cv2.VideoCapture(input_video)
            fourcc = cv2.VideoWriter_fourcc(*'MJPG')
            out_fixed = cv2.VideoWriter(output_video, fourcc, median_fps, (640, 480))

            frame_count = 0
            while True:
                ret, frame = cap.read()
                if not ret:
                    break
                out_fixed.write(frame)
                frame_count += 1

            cap.release()
            out_fixed.release()
            print(f"Re-encoded {frame_count} frames at {median_fps:.2f} FPS to {output_video}")


if __name__ == '__main__':
    main()
