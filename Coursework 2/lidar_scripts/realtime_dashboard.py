"""
Real-Time SLAM Dashboard — Camera + LiDAR simultaneous recording & visualization.

Layout:
  ┌──────────────────────┬──────────────┐
  │                      │  Camera Feed │
  │   Occupancy Grid     │  (live RGB)  │
  │   + Trajectory       ├──────────────┤
  │   (LiDAR SLAM)       │   Status     │
  │                      │   Panel      │
  └──────────────────────┴──────────────┘

Usage: python realtime_dashboard.py <output_folder_name>
Example: python realtime_dashboard.py corridor_loop_01

Controls:
  SPACE  — start / pause recording
  Q/ESC  — stop and save everything
"""

import os
import sys
import math
import time
import json
import csv
import threading
import pygame
import numpy as np
from rplidar import RPLidar, RPLidarException
from datetime import datetime, timezone

# Camera backends (try RealSense first, fall back to OpenCV webcam)
CAM_BACKEND = None
try:
    import pyrealsense2 as rs
    CAM_BACKEND = "realsense"
except ImportError:
    pass

import cv2
if CAM_BACKEND is None:
    CAM_BACKEND = "opencv"

print(f"[INFO] Camera backend: {CAM_BACKEND}")

# === CONFIG ===
LIDAR_PORT = 'COM19'
BAUD_RATE = 256000

# Layout
MAP_DIM = 600           # occupancy grid square
CAM_PANEL_W = 424       # camera preview width (half of 848)
CAM_PANEL_H = 240       # camera preview height (half of 480)
STATUS_H = MAP_DIM - CAM_PANEL_H  # status panel height
WINDOW_W = MAP_DIM + CAM_PANEL_W
WINDOW_H = MAP_DIM

CELL_SIZE_MM = 25       # mm per grid cell

# Occupancy grid drawing
CONFIDENCE_FREE = (8, 8, 8)
CONFIDENCE_OCCUPIED = (40, 40, 40)

# Blind spot
CUT_ANGLE_MIN = 135.0
CUT_ANGLE_MAX = 225.0

# Camera settings
CAM_WIDTH = 848
CAM_HEIGHT = 480
CAM_FPS = 30


class PoseEstimator:
    """Monte-Carlo pose optimizer (Lab 9 style)."""
    def __init__(self, map_dim, cell_size_mm):
        self.map_w = map_dim
        self.map_h = map_dim
        self.cell_size = cell_size_mm
        self.reset()

    def reset(self):
        self.x = (self.map_w * self.cell_size) / 2
        self.y = (self.map_h * self.cell_size) / 2
        self.theta = 0.0

    def get_pose(self):
        return self.x, self.y, self.theta

    def optimize_pose(self, scan_points, grid_map, iterations=10):
        if len(scan_points) == 0:
            return
        scan_arr = np.array(scan_points)
        angles = scan_arr[:, 0]
        dists = scan_arr[:, 1]
        local_x = dists * np.cos(angles)
        local_y = dists * np.sin(angles)
        step_xy = 30
        step_th = np.radians(0.5)

        for _ in range(iterations):
            best_score = -float('inf')
            best_pose = (self.x, self.y, self.theta)
            found_better = False
            for dth in [-step_th, 0, step_th]:
                test_th = self.theta + dth
                cos_th = np.cos(test_th)
                sin_th = np.sin(test_th)
                for dx in [-step_xy, 0, step_xy]:
                    for dy in [-step_xy, 0, step_xy]:
                        test_x = self.x + dx
                        test_y = self.y + dy
                        global_x = (local_x * cos_th - local_y * sin_th) + test_x
                        global_y = (local_x * sin_th + local_y * cos_th) + test_y
                        grid_x = (global_x / self.cell_size).astype(int)
                        grid_y = (global_y / self.cell_size).astype(int)
                        valid_mask = (grid_x >= 0) & (grid_x < self.map_w) & \
                                     (grid_y >= 0) & (grid_y < self.map_h)
                        if np.sum(valid_mask) > 5:
                            valid_gx = grid_x[valid_mask]
                            valid_gy = grid_y[valid_mask]
                            score = np.sum(grid_map[valid_gx, valid_gy])
                            if score > best_score:
                                best_score = score
                                best_pose = (test_x, test_y, test_th)
                                found_better = True
            if found_better:
                self.x, self.y, self.theta = best_pose
            else:
                break


def numpy_to_pygame_surface(img_bgr, target_w, target_h):
    """Convert an OpenCV BGR image to a pygame Surface, resized."""
    img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
    img_resized = cv2.resize(img_rgb, (target_w, target_h))
    return pygame.surfarray.make_surface(img_resized.swapaxes(0, 1))


def main():
    # Output folder
    if len(sys.argv) > 1:
        out_name = sys.argv[1]
    else:
        out_name = f"sequence_{datetime.now().strftime('%Y%m%d_%H%M%S')}"

    out_dir = os.path.abspath(out_name)
    lidar_dir = os.path.join(out_dir, "lidar")
    camera_dir = os.path.join(out_dir, "camera")
    rgb_dir = os.path.join(camera_dir, "rgb")
    os.makedirs(lidar_dir, exist_ok=True)
    os.makedirs(rgb_dir, exist_ok=True)

    print(f"Output: {out_dir}")

    # === Init LiDAR ===
    print(f"Connecting LiDAR on {LIDAR_PORT}...")
    lidar = RPLidar(LIDAR_PORT, baudrate=BAUD_RATE, timeout=1)
    print("LiDAR connected.")

    # === Init Camera ===
    pipeline = None
    cap = None
    cam_name = "none"

    if CAM_BACKEND == "realsense":
        try:
            pipeline = rs.pipeline()
            config = rs.config()
            config.enable_stream(rs.stream.color, CAM_WIDTH, CAM_HEIGHT, rs.format.bgr8, CAM_FPS)
            profile = pipeline.start(config)

            color_stream = profile.get_stream(rs.stream.color).as_video_stream_profile()
            intrinsics = color_stream.get_intrinsics()
            cam_info = {
                "camera": "Intel RealSense D455",
                "width": CAM_WIDTH, "height": CAM_HEIGHT, "fps": CAM_FPS,
                "fx": intrinsics.fx, "fy": intrinsics.fy,
                "ppx": intrinsics.ppx, "ppy": intrinsics.ppy,
                "model": str(intrinsics.model),
                "coeffs": list(intrinsics.coeffs)
            }
            with open(os.path.join(camera_dir, "camera_info.json"), 'w') as f:
                json.dump(cam_info, f, indent=2)
            cam_name = "Intel RealSense D455"
            print("RealSense D455 connected.")
        except Exception as e:
            print(f"[WARNING] RealSense failed: {e}")
            pipeline = None

    if pipeline is None and CAM_BACKEND == "opencv":
        # Fall back to default webcam / USB camera via OpenCV
        cap = cv2.VideoCapture(0)
        if cap.isOpened():
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_WIDTH)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_HEIGHT)
            cap.set(cv2.CAP_PROP_FPS, CAM_FPS)
            cam_name = "OpenCV Webcam"
            print(f"OpenCV webcam opened (index 0).")
        else:
            cap = None
            print("[WARNING] No camera found — LiDAR only mode.")

    has_camera = (pipeline is not None) or (cap is not None)

    # === Init Pygame ===
    pygame.init()
    screen = pygame.display.set_mode((WINDOW_W, WINDOW_H))
    pygame.display.set_caption("SLAM Dashboard — SPACE=record, Q=stop")
    font = pygame.font.SysFont("Consolas", 15)
    font_big = pygame.font.SysFont("Consolas", 22, bold=True)
    clock = pygame.time.Clock()

    # Map surface
    view_surface = pygame.Surface((MAP_DIM, MAP_DIM))
    view_surface.fill((128, 128, 128))
    occupancy_grid = np.full((MAP_DIM, MAP_DIM), 0.5, dtype=np.float32)

    estimator = PoseEstimator(MAP_DIM, CELL_SIZE_MM)
    trajectory_points = []

    # Latest camera frame (shared between grab thread and render)
    latest_cam_frame = None  # BGR numpy array
    cam_lock = threading.Lock()

    # Camera grab thread (non-blocking so LiDAR loop isn't stalled)
    cam_running = True

    def camera_grab_loop():
        nonlocal latest_cam_frame, cam_running
        while cam_running:
            frame = None
            if pipeline:
                try:
                    frames = pipeline.wait_for_frames(timeout_ms=100)
                    color_frame = frames.get_color_frame()
                    if color_frame:
                        frame = np.asanyarray(color_frame.get_data())
                except:
                    pass
            elif cap:
                ret, img = cap.read()
                if ret:
                    frame = img
            if frame is not None:
                with cam_lock:
                    latest_cam_frame = frame
            else:
                time.sleep(0.01)

    if has_camera:
        cam_thread = threading.Thread(target=camera_grab_loop, daemon=True)
        cam_thread.start()

    # Recording state
    recording = False
    scan_count = 0
    frame_count = 0
    start_time = None

    # File handles
    scans_file = None
    scan_index_file = None
    scan_index_writer = None
    rgb_txt_file = None
    meta_file = None
    meta_writer = None

    def start_recording():
        nonlocal scans_file, scan_index_file, scan_index_writer
        nonlocal rgb_txt_file, meta_file, meta_writer
        nonlocal start_time, scan_count, frame_count

        scans_file = open(os.path.join(lidar_dir, "scans.jsonl"), 'w')
        scan_index_file = open(os.path.join(lidar_dir, "scan_index.csv"), 'w', newline='')
        scan_index_writer = csv.writer(scan_index_file)
        scan_index_writer.writerow(["scan_idx", "timestamp_unix_s", "num_points"])

        rgb_txt_file = open(os.path.join(camera_dir, "rgb.txt"), 'w')
        rgb_txt_file.write("# timestamp filename\n")

        meta_file = open(os.path.join(camera_dir, "metadata.csv"), 'w', newline='')
        meta_writer = csv.writer(meta_file)
        meta_writer.writerow(["frame_idx", "timestamp_unix_s", "filename"])

        scan_count = 0
        frame_count = 0
        start_time = time.time()

    print("\nPress SPACE to start recording. Press Q/ESC to stop.\n")

    try:
        for scan in lidar.iter_scans(max_buf_meas=10000):
            # === Events ===
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    raise KeyboardInterrupt
                if event.type == pygame.KEYDOWN:
                    if event.key in (pygame.K_q, pygame.K_ESCAPE):
                        raise KeyboardInterrupt
                    if event.key == pygame.K_SPACE:
                        if not recording:
                            recording = True
                            start_recording()
                            print("[REC] Recording started!")
                        else:
                            recording = False
                            print(f"[PAUSE] Paused. Scans: {scan_count}, Frames: {frame_count}")

            # === Filter LiDAR ===
            valid_points = []
            for point in scan:
                if len(point) == 3:
                    _, angle, distance = point
                elif len(point) == 4:
                    _, _, angle, distance = point
                else:
                    continue
                if CUT_ANGLE_MIN <= angle <= CUT_ANGLE_MAX:
                    continue
                if distance > 0:
                    valid_points.append((math.radians(angle), distance))

            if not valid_points:
                continue

            # === Pose Estimation ===
            estimator.optimize_pose(valid_points, occupancy_grid, iterations=10)
            curr_x, curr_y, curr_th = estimator.get_pose()
            rob_px = int(curr_x / CELL_SIZE_MM)
            rob_py = int(curr_y / CELL_SIZE_MM)
            trajectory_points.append((rob_px, rob_py))

            # === Update Occupancy Grid ===
            flash_surface = pygame.Surface((MAP_DIM, MAP_DIM))
            flash_surface.fill((0, 0, 0))
            hits_surface = pygame.Surface((MAP_DIM, MAP_DIM))
            hits_surface.fill((0, 0, 0))

            cos_th = math.cos(curr_th)
            sin_th = math.sin(curr_th)
            hit_xs, hit_ys = [], []

            for (angle_rad, dist) in valid_points:
                lx = dist * math.cos(angle_rad)
                ly = dist * math.sin(angle_rad)
                gx_mm = (lx * cos_th - ly * sin_th) + curr_x
                gy_mm = (lx * sin_th + ly * cos_th) + curr_y
                px = int(gx_mm / CELL_SIZE_MM)
                py = int(gy_mm / CELL_SIZE_MM)
                if 0 <= px < MAP_DIM and 0 <= py < MAP_DIM:
                    pygame.draw.line(flash_surface, CONFIDENCE_FREE, (rob_px, rob_py), (px, py), 2)
                    pygame.draw.circle(hits_surface, CONFIDENCE_OCCUPIED, (px, py), 2)
                    hit_xs.append(px)
                    hit_ys.append(py)

            view_surface.blit(flash_surface, (0, 0), special_flags=pygame.BLEND_ADD)
            view_surface.blit(hits_surface, (0, 0), special_flags=pygame.BLEND_SUB)

            if hit_xs:
                rows = np.array(hit_xs)
                cols = np.array(hit_ys)
                occupancy_grid[rows, cols] = np.minimum(1.0, occupancy_grid[rows, cols] + 0.1)

            # === Record Data ===
            if recording:
                ts = time.time()

                # LiDAR
                scan_data = {
                    "scan_idx": scan_count,
                    "timestamp_unix_s": ts,
                    "points": [list(p) for p in scan]
                }
                scans_file.write(json.dumps(scan_data) + "\n")
                scan_index_writer.writerow([scan_count, ts, len(scan)])
                scan_count += 1

                # Camera
                if has_camera:
                    with cam_lock:
                        cam_frame = latest_cam_frame.copy() if latest_cam_frame is not None else None
                    if cam_frame is not None:
                        fname = f"frame_{frame_count:06d}.jpg"
                        cv2.imwrite(os.path.join(rgb_dir, fname), cam_frame)
                        rgb_txt_file.write(f"{ts} rgb/{fname}\n")
                        meta_writer.writerow([frame_count, ts, f"rgb/{fname}"])
                        frame_count += 1

            # === RENDER ===
            screen.fill((20, 20, 20))

            # -- Left: Occupancy grid --
            screen.blit(view_surface, (0, 0))

            # Trajectory
            if len(trajectory_points) > 1:
                pygame.draw.lines(screen, (0, 191, 255), False, trajectory_points, 2)

            # Robot marker
            pygame.draw.circle(screen, (255, 0, 0), (rob_px, rob_py), 6)
            end_x = rob_px + 18 * math.cos(curr_th)
            end_y = rob_py + 18 * math.sin(curr_th)
            pygame.draw.line(screen, (255, 0, 0), (rob_px, rob_py), (int(end_x), int(end_y)), 3)

            # -- Right top: Camera feed --
            cam_x = MAP_DIM
            cam_y = 0
            with cam_lock:
                cam_img = latest_cam_frame
            if cam_img is not None:
                cam_surface = numpy_to_pygame_surface(cam_img, CAM_PANEL_W, CAM_PANEL_H)
                screen.blit(cam_surface, (cam_x, cam_y))
                # Camera label
                label = font.render("CAMERA (live)", True, (0, 255, 0))
                screen.blit(label, (cam_x + 5, cam_y + 5))
            else:
                # No camera - draw placeholder
                placeholder = pygame.Surface((CAM_PANEL_W, CAM_PANEL_H))
                placeholder.fill((40, 40, 40))
                no_cam = font_big.render("No Camera", True, (100, 100, 100))
                placeholder.blit(no_cam, (CAM_PANEL_W // 2 - 60, CAM_PANEL_H // 2 - 12))
                screen.blit(placeholder, (cam_x, cam_y))

            # Border between camera and status
            pygame.draw.line(screen, (80, 80, 80), (cam_x, CAM_PANEL_H), (WINDOW_W, CAM_PANEL_H), 1)
            pygame.draw.line(screen, (80, 80, 80), (cam_x, 0), (cam_x, WINDOW_H), 1)

            # -- Right bottom: Status panel --
            status_x = MAP_DIM + 10
            status_y = CAM_PANEL_H + 10
            elapsed = time.time() - start_time if start_time else 0

            # Recording indicator
            if recording:
                rec_label = font_big.render("[  REC  ]", True, (255, 50, 50))
            else:
                rec_label = font_big.render("[ READY ]", True, (100, 200, 100))
            screen.blit(rec_label, (status_x, status_y))
            status_y += 30

            info_lines = [
                f"Time:   {elapsed:.0f}s",
                f"Scans:  {scan_count}",
                f"Frames: {frame_count}",
                f"Points: {len(valid_points)}",
                f"",
                f"Pose X: {curr_x:.0f} mm",
                f"Pose Y: {curr_y:.0f} mm",
                f"Theta:  {math.degrees(curr_th):.1f} deg",
                f"",
                f"Camera: {cam_name[:20]}",
                f"LiDAR:  {LIDAR_PORT}",
                f"",
                f"SPACE = rec/pause",
                f"Q/ESC = stop+save",
            ]

            for line in info_lines:
                txt = font.render(line, True, (200, 200, 200))
                screen.blit(txt, (status_x, status_y))
                status_y += 20

            # Distance from start indicator (loop closure hint)
            if len(trajectory_points) > 10:
                sx, sy = trajectory_points[0]
                dist_from_start = math.sqrt((rob_px - sx)**2 + (rob_py - sy)**2) * CELL_SIZE_MM / 1000
                dist_color = (0, 255, 0) if dist_from_start < 0.5 else (255, 255, 0) if dist_from_start < 2.0 else (200, 200, 200)
                dist_txt = font.render(f"Dist->start: {dist_from_start:.2f}m", True, dist_color)
                screen.blit(dist_txt, (status_x, status_y))

            pygame.display.flip()
            clock.tick(30)

    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        cam_running = False

        # Session info
        session = {
            "started_utc": datetime.now(timezone.utc).isoformat(),
            "total_lidar_scans": scan_count,
            "total_camera_frames": frame_count,
            "duration_s": time.time() - start_time if start_time else 0,
            "lidar_port": LIDAR_PORT,
            "camera": cam_name,
            "cell_size_mm": CELL_SIZE_MM,
            "map_dim": MAP_DIM,
        }
        with open(os.path.join(out_dir, "session_info.json"), 'w') as f:
            json.dump(session, f, indent=2)

        # Save final map
        pygame.image.save(view_surface, os.path.join(out_dir, "occupancy_grid.png"))
        print(f"Saved occupancy grid to {out_dir}/occupancy_grid.png")

        # Close files
        for fh in [scans_file, scan_index_file, rgb_txt_file, meta_file]:
            if fh:
                fh.flush()
                fh.close()

        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()
        if pipeline:
            pipeline.stop()
        if cap:
            cap.release()
        pygame.quit()

        print(f"\n=== Recording Complete ===")
        print(f"Output: {out_dir}")
        print(f"LiDAR scans: {scan_count}")
        print(f"Camera frames: {frame_count}")
        print(f"Duration: {session['duration_s']:.1f}s")


if __name__ == '__main__':
    main()
