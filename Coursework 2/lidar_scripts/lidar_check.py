"""
LiDAR Setup Checker
- Captures a few scans
- Saves diagnostic images for review
- Shows RAW scan vs FILTERED scan (blind spot removed)
- Labels directions and blind spot
"""

import numpy as np
import matplotlib.pyplot as plt
from rplidar import RPLidar
import time

PORT = 'COM19'
BLIND_SPOT_MIN = 135  # degrees
BLIND_SPOT_MAX = 225  # degrees

print(f"Connecting to RPLidar on {PORT}...")
lidar = RPLidar(PORT, baudrate=256000)

print("Warming up sensor...")
scans = []
iterator = lidar.iter_scans(max_buf_meas=500)

try:
    for i, scan in enumerate(iterator):
        scans.append(scan)
        print(f"  Scan {i}: {len(scan)} points")
        if i >= 10:
            break

    # Parse all points from last 5 scans
    all_points = []
    for scan in scans[-5:]:
        for point in scan:
            if len(point) == 3:
                quality, angle, distance = point
            elif len(point) == 4:
                _, quality, angle, distance = point
            else:
                continue
            if quality > 0 and distance > 0:
                all_points.append((angle, distance))

    all_points = np.array(all_points)
    angles_deg = all_points[:, 0]
    distances_mm = all_points[:, 1]
    angles_rad = np.radians(angles_deg)

    # === FILTER: Remove blind spot ===
    blind_mask = (angles_deg >= BLIND_SPOT_MIN) & (angles_deg <= BLIND_SPOT_MAX)
    filtered_mask = ~blind_mask

    angles_deg_f = angles_deg[filtered_mask]
    distances_mm_f = distances_mm[filtered_mask]
    angles_rad_f = np.radians(angles_deg_f)

    # === FIGURE 1: RAW vs FILTERED (4 plots) ===
    fig = plt.figure(figsize=(18, 14))

    # --- Top Left: RAW Polar ---
    ax1 = fig.add_subplot(221, projection='polar')
    ax1.scatter(angles_rad, distances_mm, s=2, c='blue', alpha=0.5)
    rmax = np.percentile(distances_mm, 95) * 1.2
    ax1.set_rmax(rmax)
    ax1.set_title("RAW Scan (All Points)\nIncludes blind spot noise", pad=20, fontsize=12)
    # Mark blind spot
    blind_angles = np.linspace(np.radians(BLIND_SPOT_MIN), np.radians(BLIND_SPOT_MAX), 50)
    ax1.plot(blind_angles, np.full(50, rmax * 0.9), 'r-', linewidth=3, label=f'Blind spot ({BLIND_SPOT_MIN}-{BLIND_SPOT_MAX}°)')
    ax1.legend(loc='lower right', fontsize=8)
    ax1.annotate('0° FRONT', xy=(0, rmax), fontsize=9, color='green', fontweight='bold')
    ax1.annotate('180° BACK', xy=(np.pi, rmax*0.6), fontsize=9, color='red', fontweight='bold')

    # --- Top Right: FILTERED Polar ---
    ax2 = fig.add_subplot(222, projection='polar')
    ax2.scatter(angles_rad_f, distances_mm_f, s=2, c='green', alpha=0.5)
    ax2.set_rmax(rmax)
    ax2.set_title("FILTERED Scan (Blind Spot Removed)\nThis is what the SLAM code uses", pad=20, fontsize=12)
    ax2.plot(blind_angles, np.full(50, rmax * 0.9), 'r-', linewidth=3, alpha=0.3)
    ax2.annotate('0° FRONT', xy=(0, rmax), fontsize=9, color='green', fontweight='bold')

    # --- Bottom Left: RAW XY ---
    ax3 = fig.add_subplot(223)
    x = distances_mm * np.cos(angles_rad)
    y = distances_mm * np.sin(angles_rad)
    ax3.scatter(x, y, s=2, c='blue', alpha=0.5, label='Raw points')
    # Highlight blind spot points in red
    x_blind = distances_mm[blind_mask] * np.cos(angles_rad[blind_mask])
    y_blind = distances_mm[blind_mask] * np.sin(angles_rad[blind_mask])
    ax3.scatter(x_blind, y_blind, s=8, c='red', alpha=0.7, label='Blind spot points (removed)')
    ax3.plot(0, 0, 'ko', markersize=10)
    ax3.set_xlabel('X (mm)')
    ax3.set_ylabel('Y (mm)')
    ax3.set_title('RAW Top-Down View\nRed = blind spot points (will be filtered out)')
    ax3.set_aspect('equal')
    ax3.grid(True)
    ax3.legend()
    # Draw blind spot wedge lines
    for angle in [BLIND_SPOT_MIN, BLIND_SPOT_MAX]:
        rad = np.radians(angle)
        r = np.percentile(distances_mm, 90)
        ax3.plot([0, r*np.cos(rad)], [0, r*np.sin(rad)], 'r--', linewidth=2)

    # --- Bottom Right: FILTERED XY ---
    ax4 = fig.add_subplot(224)
    x_f = distances_mm_f * np.cos(angles_rad_f)
    y_f = distances_mm_f * np.sin(angles_rad_f)
    ax4.scatter(x_f, y_f, s=2, c='green', alpha=0.5)
    ax4.plot(0, 0, 'ko', markersize=10, label='Sensor')
    ax4.set_xlabel('X (mm)')
    ax4.set_ylabel('Y (mm)')
    ax4.set_title('FILTERED Top-Down View\nThis is what gets used for mapping')
    ax4.set_aspect('equal')
    ax4.grid(True)
    ax4.legend()
    # Draw direction arrow (0° = front)
    arrow_len = np.percentile(distances_mm_f, 50) * 0.3
    ax4.annotate('FRONT (0°)', xy=(arrow_len, 0), xytext=(0, 0),
                 arrowprops=dict(arrowstyle='->', color='green', lw=2),
                 fontsize=10, color='green', fontweight='bold')

    plt.tight_layout()
    plt.savefig('lidar_check.png', dpi=150)
    print("\n=== SAVED: lidar_check.png ===")

    # === FIGURE 2: Angular histogram ===
    fig2, (ax5, ax6) = plt.subplots(2, 1, figsize=(12, 6))

    ax5.hist(angles_deg, bins=72, color='steelblue', edgecolor='black')
    ax5.axvspan(BLIND_SPOT_MIN, BLIND_SPOT_MAX, alpha=0.3, color='red', label='Blind spot region')
    ax5.set_xlabel('Angle (degrees)')
    ax5.set_ylabel('Point count')
    ax5.set_title('RAW Angular Coverage')
    ax5.legend()

    ax6.hist(angles_deg_f, bins=72, color='green', edgecolor='black')
    ax6.axvspan(BLIND_SPOT_MIN, BLIND_SPOT_MAX, alpha=0.3, color='red', label='Blind spot (filtered out)')
    ax6.set_xlabel('Angle (degrees)')
    ax6.set_ylabel('Point count')
    ax6.set_title('FILTERED Angular Coverage')
    ax6.legend()

    plt.tight_layout()
    plt.savefig('lidar_check_angles.png', dpi=150)
    print("=== SAVED: lidar_check_angles.png ===")

    # === STATS ===
    print(f"\n--- RAW ---")
    print(f"Total points: {len(all_points)}")
    print(f"Min distance: {distances_mm.min():.0f} mm")
    print(f"Max distance: {distances_mm.max():.0f} mm")
    print(f"Mean distance: {distances_mm.mean():.0f} mm")
    print(f"Points in blind spot: {blind_mask.sum()}/{len(angles_deg)} ({blind_mask.sum()/len(angles_deg)*100:.1f}%)")

    print(f"\n--- FILTERED ---")
    print(f"Total points: {len(angles_deg_f)}")
    print(f"Min distance: {distances_mm_f.min():.0f} mm")
    print(f"Max distance: {distances_mm_f.max():.0f} mm")
    print(f"Mean distance: {distances_mm_f.mean():.0f} mm")

    print(f"\n--- SETUP CHECK ---")
    if blind_mask.sum() / len(angles_deg) > 0.15:
        print(">> NOTE: 15%+ points in blind spot. This is fine - the code filters them out.")
        print(">> Just make sure YOUR BODY is in the blind spot direction (135-225°) when walking.")
    else:
        print(">> Blind spot is mostly clear - good setup!")

    print("\n>> Open lidar_check.png to see the scan visually.")
    print(">> The FILTERED views (right side) show what the SLAM algorithm actually uses.")

    plt.show()

except KeyboardInterrupt:
    print("\nStopped by user")
except Exception as e:
    print(f"\nError: {e}")
    import traceback
    traceback.print_exc()
finally:
    print("Shutting down...")
    lidar.stop()
    lidar.stop_motor()
    lidar.disconnect()
    print("Done.")
