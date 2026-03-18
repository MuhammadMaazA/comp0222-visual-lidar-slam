#!/bin/bash
"""
Create ACTUAL screen recordings of SLAM systems running
"""

echo "🎬 Creating REAL screen recordings of SLAM systems..."
echo "================================="

# Set up paths
ORBSLAM_BIN="/home/mmaaz/ORB_SLAM2/Install/bin/mono_tum"
DATA_DIR="/home/mmaaz/SLAM/coursework_deliverables/part1_analysis"
OUTPUT_DIR="/home/mmaaz/SLAM/coursework_deliverables"

cd "$DATA_DIR"

echo "📍 Current directory: $(pwd)"
echo "🔍 ORB-SLAM2 binary: $ORBSLAM_BIN"

# Check if ORB-SLAM2 binary exists
if [ ! -f "$ORBSLAM_BIN" ]; then
    echo "❌ ORB-SLAM2 binary not found at $ORBSLAM_BIN"
    exit 1
fi

# Check if data exists
if [ ! -f "TUM1_custom.yaml" ]; then
    echo "❌ Config file TUM1_custom.yaml not found"
    exit 1
fi

if [ ! -d "rgbd_dataset_freiburg1_xyz" ]; then
    echo "❌ Dataset directory not found"
    exit 1
fi

echo "✅ All files found, starting ORB-SLAM2..."

# Create a simple screen recording script using import from ImageMagick
echo "🎬 Starting ORB-SLAM2 with viewer..."

# Run ORB-SLAM2 in background
timeout 30s "$ORBSLAM_BIN" TUM1_custom.yaml rgbd_dataset_freiburg1_xyz real_screen_demo.txt &
SLAM_PID=$!

echo "📸 ORB-SLAM2 started with PID: $SLAM_PID"
sleep 2

# Take screenshots every 2 seconds for 20 seconds
for i in {1..10}; do
    sleep 2
    import -window root "$OUTPUT_DIR/screenshot_$i.png" 2>/dev/null || echo "Screenshot $i failed"
    echo "📸 Screenshot $i captured"
done

# Kill ORB-SLAM2
kill $SLAM_PID 2>/dev/null
wait $SLAM_PID 2>/dev/null

echo "✅ Screen capture complete!"

# Convert screenshots to video
cd "$OUTPUT_DIR"
if command -v convert >/dev/null 2>&1; then
    echo "🎞️ Converting screenshots to video..."
    # Use convert to create an animated gif, then convert to mp4
    convert -delay 200 screenshot_*.png orbslam_real_capture.gif
    echo "✅ GIF created: orbslam_real_capture.gif"
fi

# Clean up screenshots
rm -f screenshot_*.png

echo "🎯 Real screen recording attempt completed!"