"""
Validate collected data by running Lab 8 (ICP) and Lab 9 (Occupancy Grid) algorithms.
Processes joint_sequence_01 lidar data in replay mode and generates:
  1. ICP point cloud map + trajectory
  2. Occupancy grid map
  3. Camera frame quality check
  4. Full diagnostic report

Usage: python validate_data.py [path_to_sequence_folder]
Default: ../joint_sequence_01
"""

import os
import sys
import json
import math
import csv
import time
import numpy as np
import matplotlib.pyplot as plt
from sklearn.neighbors import NearestNeighbors

# === PATH ===
if len(sys.argv) > 1:
    SEQ_DIR = sys.argv[1]
else:
    SEQ_DIR = os.path.join(os.path.dirname(__file__), "..", "..",
                           "joint_sequence_01")
    if not os.path.exists(SEQ_DIR):
        SEQ_DIR = r"C:\Users\mmaaz\University\Year 3\Simultaneous Localisation and Mapping\joint_sequence_01"

LIDAR_FILE = os.path.join(SEQ_DIR, "lidar", "scans.jsonl")
CAM_DIR = os.path.join(SEQ_DIR, "camera", "rgb")
CAM_INFO = os.path.join(SEQ_DIR, "camera", "camera_info.json")
OUT_DIR = os.path.join(SEQ_DIR, "validation")
os.makedirs(OUT_DIR, exist_ok=True)

print(f"Sequence: {SEQ_DIR}")
print(f"Output:   {OUT_DIR}\n")

# ===========================================================
# LAB 8: ICP ALGORITHM (from rplidar_icp.py)
# ===========================================================

MAX_RANGE_MM = 4000.0
ICP_MAX_ITER = 10
CORRESPONDENCE_THRESH = 0.5  # meters
KEYFRAME_DIST_THRESH = 0.2   # meters
KEYFRAME_ANGLE_THRESH = 0.2  # radians
LOCAL_MAP_SIZE = 20
BLIND_SPOT_MIN = 135.0
BLIND_SPOT_MAX = 225.0


def estimate_normals_pca(points, k=5):
    if len(points) < k + 1:
        return np.zeros((len(points), 2))
    neigh = NearestNeighbors(n_neighbors=k + 1)
    neigh.fit(points)
    _, indices_all = neigh.kneighbors(points)
    normals = np.zeros((points.shape[0], 2))
    for i in range(points.shape[0]):
        neighbor_points = points[indices_all[i]]
        centered = neighbor_points - np.mean(neighbor_points, axis=0)
        cov = np.dot(centered.T, centered) / k
        eig_vals, eig_vecs = np.linalg.eigh(cov)
        normal = eig_vecs[:, 0]
        if np.dot(normal, points[i]) < 0:
            normal = -normal
        normals[i] = normal
    return normals


def solve_point_to_plane(src, dst, dst_normals):
    A, b = [], []
    for i in range(len(src)):
        s, d, n = src[i], dst[i], dst_normals[i]
        cross_term = s[0] * n[1] - s[1] * n[0]
        A.append([cross_term, n[0], n[1]])
        b.append(np.dot(d - s, n))
    if not A:
        return np.identity(3)
    A, b = np.array(A), np.array(b)
    x, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
    c, s_ = np.cos(x[0]), np.sin(x[0])
    R = np.array([[c, -s_], [s_, c]])
    T = np.identity(3)
    T[:2, :2] = R
    T[:2, 2] = [x[1], x[2]]
    return T


def icp_scan_to_map(src_points, map_points, map_normals, init_pose):
    m = src_points.shape[1]
    src_h = np.ones((m + 1, src_points.shape[0]))
    src_h[:m, :] = np.copy(src_points.T)
    current_pose = np.copy(init_pose)
    neigh = NearestNeighbors(n_neighbors=1)
    neigh.fit(map_points)
    for _ in range(ICP_MAX_ITER):
        src_global_h = np.dot(current_pose, src_h)
        src_global = src_global_h[:2, :].T
        distances, indices = neigh.kneighbors(src_global, return_distance=True)
        distances, indices = distances.ravel(), indices.ravel()
        mask = distances < CORRESPONDENCE_THRESH
        if np.sum(mask) < 10:
            break
        T_delta = solve_point_to_plane(src_global[mask], map_points[indices[mask]],
                                       map_normals[indices[mask]])
        current_pose = np.dot(T_delta, current_pose)
        if np.linalg.norm(T_delta[:2, 2]) < 0.001 and abs(np.arctan2(T_delta[1, 0], T_delta[0, 0])) < 0.001:
            break
    return current_pose


def process_scan_to_xy(points_raw, max_range_mm=MAX_RANGE_MM):
    """Convert raw scan points to XY in meters, with blind spot filter."""
    xs, ys = [], []
    for p in points_raw:
        if len(p) == 3:
            quality, angle, distance = p
        elif len(p) == 4:
            _, quality, angle, distance = p
        else:
            continue
        if distance <= 10 or distance > max_range_mm:
            continue
        if BLIND_SPOT_MIN <= angle <= BLIND_SPOT_MAX:
            continue
        rad = math.radians(angle)
        xs.append((distance / 1000.0) * math.cos(rad))
        ys.append((distance / 1000.0) * math.sin(rad))
    if len(xs) < 10:
        return None
    return np.column_stack((xs, ys))


# ===========================================================
# LOAD DATA
# ===========================================================
print("Loading LiDAR scans...")
scans = []
with open(LIDAR_FILE, 'r') as f:
    for line in f:
        if line.strip():
            scans.append(json.loads(line))
print(f"  Loaded {len(scans)} scans")

# Timestamps
t0 = scans[0]["timestamp_unix_s"]
timestamps = [s["timestamp_unix_s"] - t0 for s in scans]
duration = timestamps[-1]
print(f"  Duration: {duration:.1f}s")
print(f"  Scan rate: {len(scans)/duration:.1f} Hz")

# Camera
cam_frames = []
if os.path.isdir(CAM_DIR):
    cam_frames = sorted([f for f in os.listdir(CAM_DIR) if f.endswith(('.jpg', '.png'))])
print(f"  Camera frames: {len(cam_frames)}")

# ===========================================================
# RUN ICP SLAM (Lab 8 algorithm)
# ===========================================================
print("\n--- Running ICP SLAM (Lab 8) ---")

current_pose = np.identity(3)
last_keyframe_pose = np.identity(3)
keyframe_buffer = []
global_map_points = []
trajectory = [[0, 0]]
icp_success = 0
icp_fail = 0

t_start = time.time()

for i, scan_data in enumerate(scans):
    xy = process_scan_to_xy(scan_data["points"])
    if xy is None:
        icp_fail += 1
        continue

    if len(keyframe_buffer) == 0:
        # First scan
        normals = estimate_normals_pca(xy)
        keyframe_buffer.append((xy, normals))
        global_map_points.append(xy)
        icp_success += 1
    else:
        active_points = np.vstack([k[0] for k in keyframe_buffer])
        active_normals = np.vstack([k[1] for k in keyframe_buffer])

        new_pose = icp_scan_to_map(xy, active_points, active_normals, current_pose)
        current_pose = new_pose
        cx, cy = current_pose[0, 2], current_pose[1, 2]
        trajectory.append([cx, cy])

        delta_T = np.dot(np.linalg.inv(last_keyframe_pose), current_pose)
        dx, dy = delta_T[0, 2], delta_T[1, 2]
        dtheta = np.arctan2(delta_T[1, 0], delta_T[0, 0])
        dist_moved = math.sqrt(dx ** 2 + dy ** 2)

        if dist_moved > KEYFRAME_DIST_THRESH or abs(dtheta) > KEYFRAME_ANGLE_THRESH:
            curr_h = np.ones((3, xy.shape[0]))
            curr_h[:2, :] = xy.T
            curr_global = np.dot(current_pose, curr_h)[:2, :].T
            curr_normals = estimate_normals_pca(curr_global)
            keyframe_buffer.append((curr_global, curr_normals))
            global_map_points.append(curr_global)
            last_keyframe_pose = np.copy(current_pose)
            if len(keyframe_buffer) > LOCAL_MAP_SIZE:
                keyframe_buffer.pop(0)

        icp_success += 1

    if (i + 1) % 50 == 0:
        print(f"  Processed {i+1}/{len(scans)} scans...")

icp_time = time.time() - t_start
print(f"  ICP done in {icp_time:.1f}s")
print(f"  Success: {icp_success}, Failed: {icp_fail}")
print(f"  Keyframes: {len(global_map_points)}")

trajectory = np.array(trajectory)

# ===========================================================
# RUN OCCUPANCY GRID (Lab 9 algorithm)
# ===========================================================
print("\n--- Building Occupancy Grid (Lab 9) ---")

GRID_DIM = 800
CELL_SIZE_MM = 20  # 800 * 20mm = 16m

occ_grid = np.full((GRID_DIM, GRID_DIM), 128, dtype=np.uint8)  # 128 = unknown

for i, scan_data in enumerate(scans):
    if i >= len(trajectory):
        break

    # Get pose from ICP trajectory (convert meters to mm)
    if i == 0:
        pose_x_mm = (GRID_DIM * CELL_SIZE_MM) / 2
        pose_y_mm = (GRID_DIM * CELL_SIZE_MM) / 2
        pose_th = 0.0
    else:
        pose_x_mm = trajectory[i][0] * 1000.0 + (GRID_DIM * CELL_SIZE_MM) / 2
        pose_y_mm = trajectory[i][1] * 1000.0 + (GRID_DIM * CELL_SIZE_MM) / 2
        # Get theta from pose
        if i > 0 and i < len(trajectory) - 1:
            dx = trajectory[i][0] - trajectory[i-1][0]
            dy = trajectory[i][1] - trajectory[i-1][1]
            if abs(dx) > 0.001 or abs(dy) > 0.001:
                pose_th = math.atan2(dy, dx)

    rob_gx = int(pose_x_mm / CELL_SIZE_MM)
    rob_gy = int(pose_y_mm / CELL_SIZE_MM)

    for p in scan_data["points"]:
        if len(p) == 3:
            quality, angle, distance = p
        elif len(p) == 4:
            _, quality, angle, distance = p
        else:
            continue
        if distance <= 10 or distance > MAX_RANGE_MM:
            continue
        if BLIND_SPOT_MIN <= angle <= BLIND_SPOT_MAX:
            continue

        rad = math.radians(angle) + pose_th
        gx_mm = distance * math.cos(rad) + pose_x_mm
        gy_mm = distance * math.sin(rad) + pose_y_mm
        px = int(gx_mm / CELL_SIZE_MM)
        py = int(gy_mm / CELL_SIZE_MM)

        if 0 <= px < GRID_DIM and 0 <= py < GRID_DIM:
            # Mark occupied (darken)
            occ_grid[py, px] = max(0, occ_grid[py, px] - 15)

            # Ray trace free space (brighten along the line)
            # Simple Bresenham-ish approach: just sample a few points
            steps = max(abs(px - rob_gx), abs(py - rob_gy))
            if steps > 0:
                for t in np.linspace(0, 0.9, min(steps, 20)):
                    fx = int(rob_gx + t * (px - rob_gx))
                    fy = int(rob_gy + t * (py - rob_gy))
                    if 0 <= fx < GRID_DIM and 0 <= fy < GRID_DIM:
                        occ_grid[fy, fx] = min(255, occ_grid[fy, fx] + 3)

print("  Occupancy grid built.")

# ===========================================================
# CAMERA CHECK
# ===========================================================
print("\n--- Camera Quality Check ---")
cam_report = []
if cam_frames:
    try:
        import cv2
        # Check first, middle, last frames
        check_indices = [0, len(cam_frames)//4, len(cam_frames)//2, 3*len(cam_frames)//4, len(cam_frames)-1]
        for ci in check_indices:
            fpath = os.path.join(CAM_DIR, cam_frames[ci])
            img = cv2.imread(fpath)
            if img is None:
                cam_report.append((ci, cam_frames[ci], "UNREADABLE", 0, 0))
                continue
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            # Laplacian variance = sharpness measure
            lap_var = cv2.Laplacian(gray, cv2.CV_64F).var()
            # Brightness
            brightness = gray.mean()
            # ORB features
            orb = cv2.ORB_create(nfeatures=1000)
            kps = orb.detect(gray, None)
            n_features = len(kps)
            status = "OK" if n_features > 100 and lap_var > 20 else "POOR"
            cam_report.append((ci, cam_frames[ci], status, n_features, lap_var, brightness))
            print(f"  Frame {ci:4d} ({cam_frames[ci]}): {n_features:4d} ORB features, sharpness={lap_var:.0f}, brightness={brightness:.0f} -> {status}")
    except ImportError:
        print("  opencv-python not installed, skipping camera check")

# ===========================================================
# GENERATE PLOTS
# ===========================================================
print("\n--- Generating Plots ---")

fig = plt.figure(figsize=(20, 16))

# 1. ICP Point Cloud Map + Trajectory
ax1 = fig.add_subplot(231)
if global_map_points:
    all_pts = np.vstack(global_map_points)
    ax1.scatter(all_pts[::3, 0], all_pts[::3, 1], s=0.5, c='black', alpha=0.3)
ax1.plot(trajectory[:, 0], trajectory[:, 1], 'b-', linewidth=1.5, label='Trajectory')
ax1.plot(trajectory[0, 0], trajectory[0, 1], 'go', markersize=10, label='Start')
ax1.plot(trajectory[-1, 0], trajectory[-1, 1], 'ro', markersize=10, label='End')
ax1.set_title('ICP Point Cloud Map + Trajectory (Lab 8)')
ax1.set_xlabel('X (m)')
ax1.set_ylabel('Y (m)')
ax1.set_aspect('equal')
ax1.legend()
ax1.grid(True)

# 2. Occupancy Grid
ax2 = fig.add_subplot(232)
ax2.imshow(occ_grid, cmap='gray', origin='lower')
# Draw trajectory on grid
traj_gx = (trajectory[:, 0] * 1000 + GRID_DIM * CELL_SIZE_MM / 2) / CELL_SIZE_MM
traj_gy = (trajectory[:, 1] * 1000 + GRID_DIM * CELL_SIZE_MM / 2) / CELL_SIZE_MM
ax2.plot(traj_gx, traj_gy, 'b-', linewidth=1, alpha=0.7)
ax2.set_title('Occupancy Grid (Lab 9)')
ax2.set_xlabel('Grid X')
ax2.set_ylabel('Grid Y')

# 3. Trajectory X and Y over time
ax3 = fig.add_subplot(233)
traj_times = np.linspace(0, duration, len(trajectory))
ax3.plot(traj_times, trajectory[:, 0], 'r-', label='X')
ax3.plot(traj_times, trajectory[:, 1], 'b-', label='Y')
ax3.set_title('Position Over Time')
ax3.set_xlabel('Time (s)')
ax3.set_ylabel('Position (m)')
ax3.legend()
ax3.grid(True)

# 4. Points per scan
ax4 = fig.add_subplot(234)
pts_per_scan = []
for s in scans:
    xy = process_scan_to_xy(s["points"])
    pts_per_scan.append(len(xy) if xy is not None else 0)
ax4.plot(timestamps, pts_per_scan, 'b-', linewidth=0.5)
ax4.axhline(y=np.mean(pts_per_scan), color='r', linestyle='--', label=f'Mean: {np.mean(pts_per_scan):.0f}')
ax4.set_title('Valid Points Per Scan')
ax4.set_xlabel('Time (s)')
ax4.set_ylabel('Points')
ax4.legend()
ax4.grid(True)

# 5. Loop closure check: distance from start over time
ax5 = fig.add_subplot(235)
dist_from_start = np.sqrt(trajectory[:, 0]**2 + trajectory[:, 1]**2)
ax5.plot(traj_times, dist_from_start, 'g-')
ax5.set_title('Distance from Start Over Time')
ax5.set_xlabel('Time (s)')
ax5.set_ylabel('Distance (m)')
ax5.grid(True)
ax5.axhline(y=0, color='r', linestyle='--', alpha=0.5)

# 6. Individual scan samples
ax6 = fig.add_subplot(236)
for si, color in [(0, 'blue'), (len(scans)//2, 'green'), (len(scans)-1, 'red')]:
    xy = process_scan_to_xy(scans[si]["points"])
    if xy is not None:
        ax6.scatter(xy[:, 0], xy[:, 1], s=3, c=color, alpha=0.5, label=f'Scan {si}')
ax6.set_title('Sample Scans (First/Mid/Last)')
ax6.set_aspect('equal')
ax6.legend()
ax6.grid(True)

plt.tight_layout()
plt.savefig(os.path.join(OUT_DIR, "validation_report.png"), dpi=150)
print(f"  Saved: {OUT_DIR}/validation_report.png")

# Save occupancy grid separately
plt.figure(figsize=(8, 8))
plt.imshow(occ_grid, cmap='gray', origin='lower')
plt.plot(traj_gx, traj_gy, 'b-', linewidth=1)
plt.title('Occupancy Grid Map')
plt.savefig(os.path.join(OUT_DIR, "occupancy_grid.png"), dpi=150)
print(f"  Saved: {OUT_DIR}/occupancy_grid.png")

# Save trajectory
np.savetxt(os.path.join(OUT_DIR, "trajectory.txt"), trajectory, header="x_m y_m")
print(f"  Saved: {OUT_DIR}/trajectory.txt")

# ===========================================================
# FINAL REPORT
# ===========================================================
closure_error = math.sqrt(trajectory[-1, 0]**2 + trajectory[-1, 1]**2)
max_dist = dist_from_start.max()

print(f"\n{'='*50}")
print(f"  VALIDATION REPORT")
print(f"{'='*50}")
print(f"  Duration:           {duration:.1f}s")
print(f"  LiDAR scans:        {len(scans)}")
print(f"  Scan rate:          {len(scans)/duration:.1f} Hz")
print(f"  Camera frames:      {len(cam_frames)}")
print(f"  Camera FPS:         {len(cam_frames)/duration:.1f}" if duration > 0 else "  Camera FPS: N/A")
print(f"  ICP success rate:   {icp_success}/{icp_success+icp_fail} ({100*icp_success/(icp_success+icp_fail):.0f}%)")
print(f"  Keyframes created:  {len(global_map_points)}")
print(f"  Max dist from start:{max_dist:.2f} m")
print(f"  Closure error:      {closure_error:.3f} m (start-to-end distance)")
print(f"{'='*50}")

# Verdict
issues = []
if duration < 60:
    issues.append(f"TOO SHORT ({duration:.0f}s) - need 2-5 min for 2 loops")
if len(cam_frames) < 500:
    issues.append(f"TOO FEW CAMERA FRAMES ({len(cam_frames)}) - need 500+ after SLAM init")
if icp_fail > len(scans) * 0.2:
    issues.append(f"HIGH ICP FAILURE RATE ({icp_fail}/{len(scans)}) - sensor may be obstructed")
if closure_error > 1.0 and duration > 60:
    issues.append(f"LOOP NOT CLOSED (end {closure_error:.2f}m from start) - did you return to start?")
if np.mean(pts_per_scan) < 80:
    issues.append(f"LOW POINT DENSITY (mean {np.mean(pts_per_scan):.0f}) - check sensor placement")

if issues:
    print("\n  ISSUES FOUND:")
    for issue in issues:
        print(f"    - {issue}")
else:
    print("\n  ALL CHECKS PASSED!")

print(f"\n  Open {OUT_DIR}/validation_report.png to see full visual report.")
plt.show()
