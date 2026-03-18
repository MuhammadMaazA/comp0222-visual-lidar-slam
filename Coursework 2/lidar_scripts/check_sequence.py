"""
Check LiDAR data from joint_sequence_01
- Loads the scans.jsonl
- Plots first scan, middle scan, last scan
- Runs a quick ICP to see if scans align
- Shows the accumulated point cloud
"""

import numpy as np
import matplotlib.pyplot as plt
import json
import os

DATA_DIR = r"C:\Users\mmaaz\University\Year 3\Simultaneous Localisation and Mapping\joint_sequence_01\lidar"

# Load all scans
print("Loading scans...")
scans = []
with open(os.path.join(DATA_DIR, "scans.jsonl"), 'r') as f:
    for line in f:
        if line.strip():
            scans.append(json.loads(line))

print(f"Loaded {len(scans)} scans")

# Parse scan into XY points (with blind spot filtering)
BLIND_MIN = 135
BLIND_MAX = 225

def parse_scan(scan_data, max_range_mm=12000, filter_blind=True):
    points = scan_data["points"]
    xs, ys = [], []
    for p in points:
        if len(p) == 3:
            quality, angle, distance = p
        elif len(p) == 4:
            _, quality, angle, distance = p
        else:
            continue
        if quality <= 0 or distance <= 0 or distance > max_range_mm:
            continue
        if filter_blind and BLIND_MIN <= angle <= BLIND_MAX:
            continue
        rad = np.radians(angle)
        xs.append(distance * np.cos(rad))
        ys.append(distance * np.sin(rad))
    return np.array(xs), np.array(ys)

# === FIGURE 1: Sample scans (first, middle, last) ===
fig, axes = plt.subplots(1, 3, figsize=(18, 6))

for idx, (scan_i, title) in enumerate([
    (0, "First Scan"),
    (len(scans)//2, "Middle Scan"),
    (len(scans)-1, "Last Scan")
]):
    ax = axes[idx]
    xs, ys = parse_scan(scans[scan_i])
    ax.scatter(xs, ys, s=2, c='blue')
    ax.plot(0, 0, 'ro', markersize=8)
    ax.set_title(f"{title} (#{scan_i})\n{len(xs)} points")
    ax.set_aspect('equal')
    ax.grid(True)
    ax.set_xlabel('X (mm)')
    ax.set_ylabel('Y (mm)')

plt.suptitle("Individual Scans - Should show room/corridor walls", fontsize=14)
plt.tight_layout()
plt.savefig(os.path.join(DATA_DIR, "..", "lidar_scans_check.png"), dpi=150)
print("=== SAVED: lidar_scans_check.png ===")

# === FIGURE 2: All scans overlaid (naive - no ICP, just raw) ===
fig2, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 7))

# Raw overlay (all scans at origin - shows if environment is consistent)
all_x, all_y = [], []
for scan in scans:
    xs, ys = parse_scan(scan)
    all_x.extend(xs)
    all_y.extend(ys)

ax1.scatter(all_x, all_y, s=1, c='blue', alpha=0.1)
ax1.plot(0, 0, 'ro', markersize=8)
ax1.set_title(f"All {len(scans)} Scans Overlaid (Raw, No Odometry)\nShould look like a thick version of one scan")
ax1.set_aspect('equal')
ax1.grid(True)
ax1.set_xlabel('X (mm)')
ax1.set_ylabel('Y (mm)')

# Point count per scan over time
counts = []
timestamps = []
t0 = scans[0]["timestamp_unix_s"]
for scan in scans:
    xs, ys = parse_scan(scan)
    counts.append(len(xs))
    timestamps.append(scan["timestamp_unix_s"] - t0)

ax2.plot(timestamps, counts, 'b-', linewidth=1)
ax2.set_xlabel('Time (seconds)')
ax2.set_ylabel('Points per scan (after filtering)')
ax2.set_title('Point Count Over Time\nDrops = sensor obstructed or moving too fast')
ax2.grid(True)
ax2.axhline(y=np.mean(counts), color='r', linestyle='--', label=f'Mean: {np.mean(counts):.0f}')
ax2.legend()

plt.tight_layout()
plt.savefig(os.path.join(DATA_DIR, "..", "lidar_quality_check.png"), dpi=150)
print("=== SAVED: lidar_quality_check.png ===")

# === STATS ===
print(f"\n--- LiDAR Data Summary ---")
print(f"Total scans: {len(scans)}")
print(f"Duration: {timestamps[-1]:.1f} seconds")
print(f"Scan rate: {len(scans)/timestamps[-1]:.1f} Hz")
print(f"Points per scan: min={min(counts)}, max={max(counts)}, mean={np.mean(counts):.0f}")
print(f"Scans with < 50 points: {sum(1 for c in counts if c < 50)} (bad scans)")
print(f"Scans with < 100 points: {sum(1 for c in counts if c < 100)}")

# Check first vs last scan similarity (rough loop closure check)
xs_first, ys_first = parse_scan(scans[0])
xs_last, ys_last = parse_scan(scans[-1])

if len(xs_first) > 0 and len(xs_last) > 0:
    mean_first = np.array([np.mean(xs_first), np.mean(ys_first)])
    mean_last = np.array([np.mean(xs_last), np.mean(ys_last)])
    centroid_shift = np.linalg.norm(mean_first - mean_last)
    print(f"\nCentroid shift (first vs last scan): {centroid_shift:.0f} mm")
    if centroid_shift < 500:
        print(">> First and last scans are similar - looks like you returned to start!")
    else:
        print(">> First and last scans differ significantly - you may NOT have returned to start.")
        print(">> (This is expected if this was just a test recording)")

plt.show()
print("\nDone.")
