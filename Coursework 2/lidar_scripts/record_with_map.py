"""
Combined LiDAR + Camera recorder with LIVE occupancy grid visualization.
Records both sensors simultaneously while showing a real-time 2D map.

Usage: python record_with_map.py <output_folder_name>
Example: python record_with_map.py indoor_small

Press SPACE to start/pause recording
Press Q or ESC to stop and save
"""

import os
import sys
import math
import time
import json
import csv
import pygame
import numpy as np
from rplidar import RPLidar, RPLidarException
from datetime import datetime, timezone

# Try to import RealSense - optional
try:
    import pyrealsense2 as rs
    HAS_REALSENSE = True
except ImportError:
    HAS_REALSENSE = False
    print("[WARNING] pyrealsense2 not installed - recording LiDAR only")

# === CONFIG ===
LIDAR_PORT = 'COM19'
BAUD_RATE = 256000

# Map settings
WINDOW_WIDTH = 1000
MAP_DIM = 800
CELL_SIZE_MM = 20
SIDEBAR_WIDTH = 200

# Occupancy grid constants
CONFIDENCE_FREE = (10, 10, 10)
CONFIDENCE_OCCUPIED = (50, 50, 50)

# Blind spot
CUT_ANGLE_MIN = 135.0
CUT_ANGLE_MAX = 225.0

# Camera settings
CAM_WIDTH = 848
CAM_HEIGHT = 480
CAM_FPS = 30


class PoseEstimator:
    """Same pose estimator from Lab 9"""
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
    align = None
    if HAS_REALSENSE:
        try:
            pipeline = rs.pipeline()
            config = rs.config()
            config.enable_stream(rs.stream.color, CAM_WIDTH, CAM_HEIGHT, rs.format.bgr8, CAM_FPS)
            profile = pipeline.start(config)
            align = rs.align(rs.stream.color)

            # Get intrinsics
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
            print("RealSense connected.")
        except Exception as e:
            print(f"[WARNING] RealSense failed: {e}")
            pipeline = None

    # === Init Pygame ===
    pygame.init()
    screen = pygame.display.set_mode((WINDOW_WIDTH, MAP_DIM))
    pygame.display.set_caption("LiDAR SLAM Recorder - SPACE to start, Q to stop")
    font = pygame.font.SysFont("Arial", 16)
    clock = pygame.time.Clock()

    # Map surfaces
    view_surface = pygame.Surface((MAP_DIM, MAP_DIM))
    view_surface.fill((128, 128, 128))
    occupancy_grid = np.full((MAP_DIM, MAP_DIM), 0.5, dtype=np.float32)

    estimator = PoseEstimator(MAP_DIM, CELL_SIZE_MM)
    trajectory_points = []

    # Recording state
    recording = False
    scan_count = 0
    frame_count = 0
    start_time = None

    # File handles
    scans_file = None
    scan_index_file = None
    rgb_txt_file = None
    meta_file = None

    def start_recording():
        nonlocal scans_file, scan_index_file, rgb_txt_file, meta_file, start_time
        nonlocal scan_count, frame_count

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
        return scan_index_writer, meta_writer

    scan_index_writer = None
    meta_writer = None

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
                            scan_index_writer, meta_writer = start_recording()
                            print("[REC] Recording started!")
                        else:
                            recording = False
                            print(f"[PAUSE] Paused. Scans: {scan_count}, Frames: {frame_count}")

            # === Filter LiDAR ===
            valid_points = []
            raw_points = []
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
                    raw_points.append(point)

            if not valid_points:
                continue

            # === Pose Estimation ===
            estimator.optimize_pose(valid_points, occupancy_grid, iterations=10)
            curr_x, curr_y, curr_th = estimator.get_pose()
            rob_px = int(curr_x / CELL_SIZE_MM)
            rob_py = int(curr_y / CELL_SIZE_MM)
            trajectory_points.append((rob_px, rob_py))

            # === Update Map ===
            flash_surface = pygame.Surface((MAP_DIM, MAP_DIM))
            flash_surface.fill((0, 0, 0))
            hits_surface = pygame.Surface((MAP_DIM, MAP_DIM))
            hits_surface.fill((0, 0, 0))

            cos_th = math.cos(curr_th)
            sin_th = math.sin(curr_th)
            math_hits_x, math_hits_y = [], []

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
                    math_hits_x.append(px)
                    math_hits_y.append(py)

            view_surface.blit(flash_surface, (0, 0), special_flags=pygame.BLEND_ADD)
            view_surface.blit(hits_surface, (0, 0), special_flags=pygame.BLEND_SUB)

            if math_hits_x:
                rows = np.array(math_hits_x)
                cols = np.array(math_hits_y)
                occupancy_grid[rows, cols] = np.minimum(1.0, occupancy_grid[rows, cols] + 0.1)

            # === Record Data ===
            if recording:
                ts = time.time()

                # Save LiDAR scan
                scan_data = {
                    "scan_idx": scan_count,
                    "timestamp_unix_s": ts,
                    "points": [list(p) for p in scan]
                }
                scans_file.write(json.dumps(scan_data) + "\n")
                scan_index_writer.writerow([scan_count, ts, len(scan)])
                scan_count += 1

                # Save camera frame
                if pipeline:
                    try:
                        frames = pipeline.wait_for_frames(timeout_ms=50)
                        if frames:
                            color_frame = frames.get_color_frame()
                            if color_frame:
                                import cv2
                                color_image = np.asanyarray(color_frame.get_data())
                                fname = f"frame_{frame_count:06d}.jpg"
                                cv2.imwrite(os.path.join(rgb_dir, fname), color_image)
                                rgb_txt_file.write(f"{ts} rgb/{fname}\n")
                                meta_writer.writerow([frame_count, ts, f"rgb/{fname}"])
                                frame_count += 1
                    except Exception:
                        pass

            # === Render ===
            screen.fill((30, 30, 30))
            screen.blit(view_surface, (0, 0))

            # Trajectory
            if len(trajectory_points) > 1:
                pygame.draw.lines(screen, (0, 191, 255), False, trajectory_points, 2)

            # Robot
            pygame.draw.circle(screen, (255, 0, 0), (rob_px, rob_py), 8)
            end_x = rob_px + 20 * math.cos(curr_th)
            end_y = rob_py + 20 * math.sin(curr_th)
            pygame.draw.line(screen, (255, 0, 0), (rob_px, rob_py), (end_x, end_y), 3)

            # Sidebar
            sidebar_x = MAP_DIM + 10
            elapsed = time.time() - start_time if start_time else 0

            lines = [
                f"{'[REC]' if recording else '[PAUSED]'}",
                f"Time: {elapsed:.0f}s",
                f"LiDAR scans: {scan_count}",
                f"Cam frames: {frame_count}",
                f"Points: {len(valid_points)}",
                f"Pose: ({curr_x:.0f}, {curr_y:.0f})",
                f"Theta: {math.degrees(curr_th):.1f} deg",
                f"",
                f"SPACE: start/pause",
                f"Q/ESC: stop & save",
            ]
            for i, line in enumerate(lines):
                color = (255, 50, 50) if i == 0 and recording else (200, 200, 200)
                text = font.render(line, True, color)
                screen.blit(text, (sidebar_x, 10 + i * 22))

            pygame.display.flip()
            clock.tick(30)

    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        # Save session info
        session = {
            "started_utc": datetime.now(timezone.utc).isoformat(),
            "total_lidar_scans": scan_count,
            "total_camera_frames": frame_count,
            "duration_s": time.time() - start_time if start_time else 0,
            "lidar_port": LIDAR_PORT,
            "camera": "Intel RealSense D455" if pipeline else "none"
        }
        with open(os.path.join(out_dir, "session_info.json"), 'w') as f:
            json.dump(session, f, indent=2)

        # Save final map image
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
        pygame.quit()

        print(f"\n=== Recording Complete ===")
        print(f"Output: {out_dir}")
        print(f"LiDAR scans: {scan_count}")
        print(f"Camera frames: {frame_count}")
        print(f"Duration: {session['duration_s']:.1f}s")


if __name__ == '__main__':
    main()
