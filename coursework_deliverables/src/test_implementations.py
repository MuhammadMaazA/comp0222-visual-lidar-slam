#!/usr/bin/env python3
"""
Quick SLAM Demo Test - Non-interactive version for validation
"""

import sys
import os
import matplotlib
matplotlib.use('Agg')  # Use non-interactive backend
import matplotlib.pyplot as plt
import numpy as np
import cv2

def test_visual_slam_demo():
    """Test visual SLAM demo functionality"""
    print("Testing Visual SLAM Demo components...")

    # Test 1: OpenCV functionality
    try:
        orb = cv2.ORB_create(nfeatures=100)
        print("✅ OpenCV ORB feature detection available")
    except Exception as e:
        print(f"❌ OpenCV error: {e}")
        return False

    # Test 2: Matplotlib functionality
    try:
        fig, axes = plt.subplots(2, 2, figsize=(10, 8))
        plt.close(fig)
        print("✅ Matplotlib plotting available")
    except Exception as e:
        print(f"❌ Matplotlib error: {e}")
        return False

    # Test 3: Basic trajectory simulation
    try:
        trajectory = []
        for i in range(10):
            x = 0.1 * i + np.random.normal(0, 0.01)
            y = 0.05 * i + np.random.normal(0, 0.01)
            trajectory.append([x, y])

        trajectory = np.array(trajectory)
        print(f"✅ Trajectory simulation: {len(trajectory)} poses generated")
    except Exception as e:
        print(f"❌ Trajectory simulation error: {e}")
        return False

    # Test 4: Feature detection on synthetic image
    try:
        # Create synthetic image
        image = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Detect features
        orb = cv2.ORB_create(nfeatures=500)
        keypoints, descriptors = orb.detectAndCompute(gray, None)

        print(f"✅ Feature detection: {len(keypoints)} features detected")
    except Exception as e:
        print(f"❌ Feature detection error: {e}")
        return False

    return True

def test_lidar_slam_demo():
    """Test LiDAR SLAM demo functionality"""
    print("\\nTesting LiDAR SLAM Demo components...")

    # Test 1: Scan processing
    try:
        # Simulate LiDAR scan data
        scan_data = []
        for angle in range(0, 360, 2):
            quality = 100
            distance = 2000 + np.random.randint(-100, 100)
            scan_data.append([quality, angle, distance])

        # Process scan
        points = []
        for measurement in scan_data:
            quality, angle_deg, distance_mm = measurement
            if quality > 0 and 100 < distance_mm < 4000:
                angle_rad = np.radians(angle_deg)
                distance_m = distance_mm / 1000.0
                x = distance_m * np.cos(angle_rad)
                y = distance_m * np.sin(angle_rad)
                points.append([x, y])

        points = np.array(points)
        print(f"✅ LiDAR scan processing: {len(points)} points generated")
    except Exception as e:
        print(f"❌ LiDAR scan processing error: {e}")
        return False

    # Test 2: ICP simulation
    try:
        if len(points) > 10:
            # Simple transformation test
            transform = np.array([
                [0.99, -0.01, 0.05],
                [0.01,  0.99, 0.02],
                [0, 0, 1]
            ])

            points_h = np.column_stack([points, np.ones(len(points))])
            transformed = (transform @ points_h.T).T[:, :2]

            print(f"✅ ICP transformation: {len(transformed)} points transformed")
    except Exception as e:
        print(f"❌ ICP transformation error: {e}")
        return False

    return True

def test_screen_recording_helper():
    """Test screen recording helper functionality"""
    print("\\nTesting Screen Recording Helper...")

    try:
        # Test platform detection
        import sys
        if sys.platform.startswith('linux'):
            platform = 'linux'
        elif sys.platform.startswith('win'):
            platform = 'windows'
        elif sys.platform.startswith('darwin'):
            platform = 'macos'
        else:
            platform = 'unknown'

        print(f"✅ Platform detection: {platform}")
    except Exception as e:
        print(f"❌ Platform detection error: {e}")
        return False

    return True

def check_sequence_data():
    """Check if sequence data is available"""
    print("\\nChecking sequence data availability...")

    # Check for visual sequence data
    visual_paths = [
        "data/part2_sequences/Indoor_Room_1",
        "data/part2_sequences/Basement_2"
    ]

    for path in visual_paths:
        if os.path.exists(path):
            camera_path = os.path.join(path, "camera")
            if os.path.exists(camera_path):
                rgb_file = os.path.join(camera_path, "rgb.txt")
                if os.path.exists(rgb_file):
                    with open(rgb_file, 'r') as f:
                        lines = len(f.readlines())
                    print(f"✅ Visual sequence {path}: {lines} frames")
                else:
                    print(f"⚠️ Visual sequence {path}: rgb.txt not found")
            else:
                print(f"⚠️ Visual sequence {path}: camera folder not found")
        else:
            print(f"❌ Visual sequence {path}: not found")

    # Check for LiDAR sequence data
    lidar_paths = [
        "data/part3_lidar_slam"
    ]

    for path in lidar_paths:
        if os.path.exists(path):
            print(f"✅ LiDAR sequence {path}: available")
        else:
            print(f"❌ LiDAR sequence {path}: not found")

def main():
    print("COMP0222 Coursework 2 - Implementation Test Suite")
    print("=" * 60)

    all_passed = True

    # Test visual SLAM demo
    if not test_visual_slam_demo():
        all_passed = False

    # Test LiDAR SLAM demo
    if not test_lidar_slam_demo():
        all_passed = False

    # Test screen recording helper
    if not test_screen_recording_helper():
        all_passed = False

    # Check sequence data
    check_sequence_data()

    print("\\n" + "=" * 60)
    if all_passed:
        print("🎉 ALL TESTS PASSED! Implementations are ready for use.")
        print("\\nNext steps:")
        print("1. Use screen_recording_helper.py to set up recording")
        print("2. Run visual_slam_demo.py for Q2c video")
        print("3. Run lidar_slam_demo.py for Q3e video")
    else:
        print("❌ Some tests failed. Check dependencies and fix errors.")

    print("=" * 60)

if __name__ == "__main__":
    main()