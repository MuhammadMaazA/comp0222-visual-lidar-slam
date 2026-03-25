#!/usr/bin/env python3
"""
Screen Recording Setup and Execution Helper
Guides user through screen recording process for coursework videos
"""

import subprocess
import sys
import os
import time
import argparse

class ScreenRecordingHelper:
    def __init__(self):
        """Initialize screen recording helper"""
        self.record_commands = {
            'linux': {
                'ffmpeg': 'ffmpeg -f x11grab -s {width}x{height} -r {fps} -i :0.0+{x},{y} -c:v libx264 -preset fast -crf 23 {output}',
                'recordmydesktop': 'recordmydesktop --x={x} --y={y} --width={width} --height={height} --fps={fps} -o {output}',
                'obs': 'OBS Studio (GUI required)'
            },
            'windows': {
                'ffmpeg': 'ffmpeg -f gdigrab -s {width}x{height} -r {fps} -i desktop -c:v libx264 -preset fast -crf 23 {output}',
                'obs': 'OBS Studio (recommended for Windows)'
            },
            'macos': {
                'ffmpeg': 'ffmpeg -f avfoundation -i "1:0" -s {width}x{height} -r {fps} -c:v libx264 -preset fast -crf 23 {output}',
                'quicktime': 'QuickTime Player (built-in)'
            }
        }

    def detect_platform(self):
        """Detect current operating system"""
        if sys.platform.startswith('linux'):
            return 'linux'
        elif sys.platform.startswith('win'):
            return 'windows'
        elif sys.platform.startswith('darwin'):
            return 'macos'
        else:
            return 'unknown'

    def check_dependencies(self):
        """Check for screen recording tools"""
        available_tools = []

        # Check for FFmpeg
        try:
            subprocess.run(['ffmpeg', '-version'], capture_output=True, check=True)
            available_tools.append('ffmpeg')
        except (subprocess.CalledProcessError, FileNotFoundError):
            pass

        # Check for platform-specific tools
        platform = self.detect_platform()
        if platform == 'linux':
            try:
                subprocess.run(['recordmydesktop', '--help'], capture_output=True, check=True)
                available_tools.append('recordmydesktop')
            except (subprocess.CalledProcessError, FileNotFoundError):
                pass

        return available_tools

    def get_demo_window_geometry(self, demo_type):
        """Get expected window geometry for demos"""
        geometries = {
            'visual_slam': {
                'width': 1600,
                'height': 1200,
                'x': 100,
                'y': 50
            },
            'lidar_slam': {
                'width': 1600,
                'height': 1200,
                'x': 100,
                'y': 50
            }
        }
        return geometries.get(demo_type, geometries['visual_slam'])

    def create_recording_script(self, demo_type, tool='ffmpeg', fps=10):
        """Create platform-specific recording script"""
        platform = self.detect_platform()
        geometry = self.get_demo_window_geometry(demo_type)

        output_file = f"COMP0222_CW2_GRP_1_{demo_type.replace('_', '_')}.mp4"

        if tool in self.record_commands.get(platform, {}):
            command = self.record_commands[platform][tool].format(
                width=geometry['width'],
                height=geometry['height'],
                x=geometry['x'],
                y=geometry['y'],
                fps=fps,
                output=output_file
            )

            script_name = f"record_{demo_type}.sh" if platform != 'windows' else f"record_{demo_type}.bat"

            with open(script_name, 'w') as f:
                if platform != 'windows':
                    f.write("#!/bin/bash\\n")
                    f.write(f"# Screen recording script for {demo_type}\\n")
                    f.write(f"echo 'Starting recording of {demo_type}...'\\n")
                    f.write(f"echo 'Output file: {output_file}'\\n")
                    f.write(f"echo 'Press Ctrl+C to stop recording'\\n")
                    f.write("sleep 3\\n")
                    f.write(f"{command}\\n")
                else:
                    f.write(f"@echo off\\n")
                    f.write(f"echo Starting recording of {demo_type}...\\n")
                    f.write(f"echo Output file: {output_file}\\n")
                    f.write(f"echo Press Ctrl+C to stop recording\\n")
                    f.write("timeout /t 3\\n")
                    f.write(f"{command}\\n")

            # Make script executable on Unix systems
            if platform != 'windows':
                os.chmod(script_name, 0o755)

            return script_name, output_file

        return None, None

    def print_manual_instructions(self, demo_type):
        """Print manual recording instructions"""
        output_file = f"COMP0222_CW2_GRP_1_{demo_type.replace('_', '_')}.mp4"

        print("\\n" + "="*60)
        print(f"MANUAL SCREEN RECORDING INSTRUCTIONS - {demo_type.upper()}")
        print("="*60)
        print("1. Position your screen recording software to capture the demo window")
        print("2. Set recording area to approximately 1600x1200 pixels")
        print("3. Set frame rate to 8-10 FPS for smooth playback")
        print("4. Use H.264 codec for compatibility")
        print(f"5. Output filename: {output_file}")
        print("\\nRecommended settings:")
        print("- Resolution: 1600x1200")
        print("- Frame rate: 10 FPS")
        print("- Codec: H.264")
        print("- Quality: Medium-High")
        print("- Audio: Not required for this demonstration")

    def run_demo_with_recording(self, demo_type, sequence_path=None, frames=300, fps=8):
        """Run demonstration and guide recording process"""
        print("\\n" + "="*60)
        print(f"PREPARING {demo_type.upper()} DEMONSTRATION")
        print("="*60)

        # Check dependencies
        available_tools = self.check_dependencies()

        if available_tools:
            print(f"Available recording tools: {', '.join(available_tools)}")
            tool = available_tools[0]  # Use first available tool

            script_name, output_file = self.create_recording_script(demo_type, tool, fps)

            if script_name:
                print(f"\\nCreated recording script: {script_name}")
                print(f"Output file will be: {output_file}")
                print("\\nTo record:")
                print(f"1. Run the recording script: ./{script_name}")
                print("2. In another terminal, run the demo")
                print("3. Press Ctrl+C in recording terminal to stop")
        else:
            self.print_manual_instructions(demo_type)

        print("\\n" + "="*60)
        print("DEMO EXECUTION INSTRUCTIONS")
        print("="*60)

        if demo_type == 'visual_slam':
            demo_cmd = "python3 src/visual_slam_demo.py"
            if sequence_path:
                demo_cmd += f" --sequence {sequence_path}"
            demo_cmd += f" --frames {frames} --fps {fps}"

            print("Visual SLAM Demo will show:")
            print("- Current camera view with ORB features")
            print("- Real-time 3D trajectory building")
            print("- Map points and tracking status")
            print("- Performance metrics")

        elif demo_type == 'lidar_slam':
            demo_cmd = "python3 src/lidar_slam_demo.py"
            if sequence_path:
                demo_cmd += f" --sequence {sequence_path}"
            demo_cmd += f" --frames {frames} --fps {fps}"

            print("LiDAR SLAM Demo will show:")
            print("- Current LiDAR scan visualization")
            print("- Real-time map building")
            print("- Loop closure detection")
            print("- Pose uncertainty tracking")

        print(f"\\nRun command: {demo_cmd}")
        print("\\nPress Enter when ready to continue...")
        input()

        return demo_cmd


def main():
    parser = argparse.ArgumentParser(description='Screen Recording Helper for SLAM Demos')
    parser.add_argument('demo_type', choices=['visual_slam', 'lidar_slam'],
                       help='Type of demonstration to record')
    parser.add_argument('--sequence', help='Path to sequence data')
    parser.add_argument('--frames', type=int, default=300,
                       help='Number of frames to process')
    parser.add_argument('--fps', type=int, default=8,
                       help='Demo frame rate')
    parser.add_argument('--record-fps', type=int, default=10,
                       help='Recording frame rate')

    args = parser.parse_args()

    helper = ScreenRecordingHelper()

    print("COMP0222 Coursework 2 - Screen Recording Helper")
    print("This tool helps create screen recordings for Q2c and Q3e")

    demo_cmd = helper.run_demo_with_recording(
        args.demo_type,
        args.sequence,
        args.frames,
        args.fps
    )

    print("\\n" + "="*60)
    print("READY TO START!")
    print("="*60)
    print("1. Start your screen recording")
    print("2. Run the demo command below")
    print("3. Stop recording when demo finishes")
    print("\\nDemo command:")
    print(f"  {demo_cmd}")
    print("\\nGood luck with your recording!")


if __name__ == "__main__":
    main()