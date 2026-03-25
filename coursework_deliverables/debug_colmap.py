#!/usr/bin/env python3
"""
Debug COLMAP image preparation
"""

from pathlib import Path
import cv2

def debug_image_preparation():
    sequence_path = Path("data/part2_sequences/Indoor_Room_1")
    camera_path = sequence_path / "camera"
    rgb_file = camera_path / "rgb.txt"

    print(f"Sequence path: {sequence_path}")
    print(f"Camera path: {camera_path}")
    print(f"RGB file: {rgb_file}")
    print(f"RGB file exists: {rgb_file.exists()}")

    if rgb_file.exists():
        print("\\nReading rgb.txt:")
        with open(rgb_file, 'r') as f:
            for i, line in enumerate(f):
                if i >= 5:  # Just check first 5 lines
                    break

                if line.strip() and not line.startswith('#'):
                    parts = line.strip().split()
                    if len(parts) >= 2:
                        timestamp, filename = parts[0], parts[1]
                        source_path = camera_path / filename

                        print(f"  Line {i}: {filename}")
                        print(f"    Full path: {source_path}")
                        print(f"    Exists: {source_path.exists()}")

                        if source_path.exists():
                            img = cv2.imread(str(source_path))
                            if img is not None:
                                print(f"    Image loaded: {img.shape}")
                            else:
                                print(f"    Image loading failed")


if __name__ == "__main__":
    debug_image_preparation()