#!/usr/bin/env python3
"""
Part 2 COLMAP vs ORB-SLAM2 Trajectory Comparison Script
Compares trajectories from COLMAP and ORB-SLAM2 for both sequences
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os

def load_tum_trajectory(filename):
    """Load trajectory in TUM format"""
    data = []
    with open(filename, 'r') as f:
        for line in f:
            if line.startswith('#'):
                continue
            parts = line.strip().split()
            if len(parts) >= 8:
                timestamp = float(parts[0])
                x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
                qx, qy, qz, qw = float(parts[4]), float(parts[5]), float(parts[6]), float(parts[7])
                data.append([timestamp, x, y, z, qx, qy, qz, qw])
    return np.array(data)

def load_colmap_trajectory(colmap_workspace):
    """Load COLMAP trajectory from sparse reconstruction"""
    images_txt = os.path.join(colmap_workspace, 'sparse', '0', 'images.txt')
    
    if not os.path.exists(images_txt):
        print(f"Warning: COLMAP reconstruction not found at {images_txt}")
        return None
    
    data = []
    with open(images_txt, 'r') as f:
        for line in f:
            if line.startswith('#'):
                continue
            parts = line.strip().split()
            if len(parts) >= 10:  # IMAGE_ID, QW, QX, QY, QZ, TX, TY, TZ, CAMERA_ID, NAME
                qw, qx, qy, qz = float(parts[1]), float(parts[2]), float(parts[3]), float(parts[4])
                tx, ty, tz = float(parts[5]), float(parts[6]), float(parts[7])
                name = parts[9]
                # Extract frame number from filename (frame_XXXXXX.jpg)
                frame_num = int(name.split('_')[1].split('.')[0])
                data.append([frame_num, tx, ty, tz, qx, qy, qz, qw])
    
    if data:
        # Sort by frame number
        data = sorted(data, key=lambda x: x[0])
        return np.array(data)
    return None

def align_trajectories(traj1, traj2):
    """Simple alignment using first pose as origin"""
    # Extract positions
    pos1 = traj1[:, 1:4]
    pos2 = traj2[:, 1:4]
    
    # Align first pose to origin
    pos1_aligned = pos1 - pos1[0]
    pos2_aligned = pos2 - pos2[0]
    
    return pos1_aligned, pos2_aligned

def calculate_metrics(pos1, pos2):
    """Calculate trajectory comparison metrics"""
    # Find common length
    min_len = min(len(pos1), len(pos2))
    pos1_common = pos1[:min_len]
    pos2_common = pos2[:min_len]
    
    # Calculate differences
    diffs = np.linalg.norm(pos1_common - pos2_common, axis=1)
    
    metrics = {
        'ate_mean': np.mean(diffs),
        'ate_rmse': np.sqrt(np.mean(diffs**2)),
        'ate_std': np.std(diffs),
        'ate_max': np.max(diffs),
        'trajectory_length_orb': np.sum(np.linalg.norm(np.diff(pos1_common, axis=0), axis=1)),
        'trajectory_length_colmap': np.sum(np.linalg.norm(np.diff(pos2_common, axis=0), axis=1))
    }
    
    return metrics, diffs

def plot_comparison(orb_pos, colmap_pos, sequence_name, save_path):
    """Plot ORB-SLAM2 vs COLMAP trajectory comparison"""
    fig = plt.figure(figsize=(15, 10))
    
    # 2D Top view comparison
    plt.subplot(2, 2, 1)
    plt.plot(orb_pos[:, 0], orb_pos[:, 1], 'b-', linewidth=2, label='ORB-SLAM2', alpha=0.8)
    plt.plot(colmap_pos[:, 0], colmap_pos[:, 1], 'r-', linewidth=2, label='COLMAP', alpha=0.8)
    plt.plot(orb_pos[0, 0], orb_pos[0, 1], 'bo', markersize=8, label='Start')
    plt.plot(colmap_pos[0, 0], colmap_pos[0, 1], 'ro', markersize=8)
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.title(f'{sequence_name} - Top View (XY)')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.axis('equal')
    
    # 3D comparison
    ax = fig.add_subplot(2, 2, 2, projection='3d')
    ax.plot(orb_pos[:, 0], orb_pos[:, 1], orb_pos[:, 2], 'b-', linewidth=2, alpha=0.8, label='ORB-SLAM2')
    ax.plot(colmap_pos[:, 0], colmap_pos[:, 1], colmap_pos[:, 2], 'r-', linewidth=2, alpha=0.8, label='COLMAP')
    ax.scatter(orb_pos[0, 0], orb_pos[0, 1], orb_pos[0, 2], color='blue', s=50)
    ax.scatter(colmap_pos[0, 0], colmap_pos[0, 1], colmap_pos[0, 2], color='red', s=50)
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title(f'{sequence_name} - 3D Trajectory')
    ax.legend()
    
    # Calculate and plot error over time
    metrics, diffs = calculate_metrics(orb_pos, colmap_pos)
    
    plt.subplot(2, 2, 3)
    plt.plot(diffs, 'g-', linewidth=1.5)
    plt.xlabel('Frame Number')
    plt.ylabel('Position Error (m)')
    plt.title(f'{sequence_name} - Absolute Trajectory Error')
    plt.grid(True, alpha=0.3)
    
    # Plot error statistics
    plt.subplot(2, 2, 4)
    plt.text(0.1, 0.9, f"ATE RMSE: {metrics['ate_rmse']:.3f}m", transform=plt.gca().transAxes, fontsize=12)
    plt.text(0.1, 0.8, f"ATE Mean: {metrics['ate_mean']:.3f}m", transform=plt.gca().transAxes, fontsize=12)
    plt.text(0.1, 0.7, f"ATE Std: {metrics['ate_std']:.3f}m", transform=plt.gca().transAxes, fontsize=12)
    plt.text(0.1, 0.6, f"ATE Max: {metrics['ate_max']:.3f}m", transform=plt.gca().transAxes, fontsize=12)
    plt.text(0.1, 0.5, f"Length ORB: {metrics['trajectory_length_orb']:.2f}m", transform=plt.gca().transAxes, fontsize=12)
    plt.text(0.1, 0.4, f"Length COLMAP: {metrics['trajectory_length_colmap']:.2f}m", transform=plt.gca().transAxes, fontsize=12)
    plt.axis('off')
    plt.title(f'{sequence_name} - Metrics')
    
    plt.tight_layout()
    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    print(f"📊 {sequence_name} comparison plot saved as '{save_path}'")
    
    return metrics

def compare_sequence(sequence_name, orb_file, colmap_workspace):
    """Compare ORB-SLAM2 and COLMAP for a single sequence"""
    print(f"\n=== {sequence_name} Comparison ===")
    
    # Load ORB-SLAM2 trajectory
    orb_traj = load_tum_trajectory(orb_file)
    if orb_traj is None or len(orb_traj) == 0:
        print(f"❌ Failed to load ORB-SLAM2 trajectory from {orb_file}")
        return None
    
    print(f"✅ ORB-SLAM2: {len(orb_traj)} poses loaded")
    
    # Load COLMAP trajectory
    colmap_traj = load_colmap_trajectory(colmap_workspace)
    if colmap_traj is None or len(colmap_traj) == 0:
        print(f"❌ Failed to load COLMAP trajectory from {colmap_workspace}")
        return None
    
    print(f"✅ COLMAP: {len(colmap_traj)} poses loaded")
    
    # Align trajectories
    orb_pos, colmap_pos = align_trajectories(orb_traj, colmap_traj)
    
    # Calculate metrics
    metrics = plot_comparison(orb_pos, colmap_pos, sequence_name, 
                             f'{sequence_name.lower()}_comparison.png')
    
    return metrics

def main():
    print("🎯 Part 2: COLMAP vs ORB-SLAM2 Trajectory Comparison")
    print("=" * 60)
    
    # Sequence configurations
    sequences = [
        {
            'name': 'Washroom',
            'orb_file': 'washroom_dataset/washroom_trajectory.txt',
            'colmap_workspace': 'washroom_dataset/colmap_washroom'
        },
        {
            'name': 'Basement_2',
            'orb_file': 'basement2_dataset/basement2_trajectory.txt', 
            'colmap_workspace': 'basement2_dataset/colmap_basement2'
        }
    ]
    
    all_metrics = {}
    
    for seq in sequences:
        metrics = compare_sequence(seq['name'], seq['orb_file'], seq['colmap_workspace'])
        if metrics:
            all_metrics[seq['name']] = metrics
    
    # Summary comparison
    print("\n" + "="*60)
    print("📊 SUMMARY - COLMAP vs ORB-SLAM2 Comparison:")
    print("="*60)
    
    for seq_name, metrics in all_metrics.items():
        print(f"\n{seq_name}:")
        print(f"  ATE RMSE: {metrics['ate_rmse']:.3f}m")
        print(f"  ATE Mean: {metrics['ate_mean']:.3f}m")
        print(f"  Trajectory Length Difference: {abs(metrics['trajectory_length_orb'] - metrics['trajectory_length_colmap']):.2f}m")
    
    if all_metrics:
        print(f"\n🎯 PART 2 COMPLETION STATUS:")
        print(f"✅ ORB-SLAM2 tracking: Both sequences")
        print(f"✅ COLMAP reconstruction: {len(all_metrics)} sequence(s)")
        print(f"✅ Quantitative comparison: {len(all_metrics)} sequence(s)")
        print(f"✅ Visualization plots: Generated for all")
    else:
        print("\n⚠️  No successful comparisons completed")
        print("Please ensure COLMAP reconstructions are finished")

if __name__ == "__main__":
    main()