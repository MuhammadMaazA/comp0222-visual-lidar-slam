#!/usr/bin/env python3
"""
Quick analysis of all sequences to find best quality ones
"""

import os
import subprocess
import time

def test_sequence(seq_name, timeout=120):
    """Test ORB-SLAM2 on a sequence with timeout"""
    print(f"\n🔍 Testing {seq_name}...")
    
    # Prepare dataset
    os.makedirs(f"{seq_name}_dataset", exist_ok=True)
    
    # Copy data
    src_path = f"/home/mmaaz/SLAM/extracted_data/tmp_recordings/tmp_recordings/{seq_name}/camera"
    subprocess.run(f"cp -r {src_path}/rgb {seq_name}_dataset/", shell=True)
    subprocess.run(f"cp {src_path}/rgb.txt {seq_name}_dataset/", shell=True)
    
    # Count images
    rgb_count = len(os.listdir(f"{seq_name}_dataset/rgb"))
    print(f"  📸 Images: {rgb_count}")
    
    # Test ORB-SLAM2 with timeout
    try:
        start_time = time.time()
        result = subprocess.run([
            "timeout", str(timeout),
            "mono_tum", "../D455_Simple.yaml", 
            f"{seq_name}_dataset", f"{seq_name}_trajectory.txt"
        ], capture_output=True, text=True, timeout=timeout+10)
        
        elapsed = time.time() - start_time
        
        if result.returncode == 0:
            # Check if trajectory file exists and has content
            if os.path.exists(f"{seq_name}_trajectory.txt"):
                traj_lines = sum(1 for line in open(f"{seq_name}_trajectory.txt"))
                print(f"  ✅ SUCCESS: {traj_lines} poses in {elapsed:.1f}s")
                
                # Extract some key metrics from output
                output = result.stderr
                if "New Map created with" in output:
                    map_points = [line for line in output.split('\n') if "New Map created with" in line][-1]
                    print(f"  🗺️ {map_points}")
                    
                return True, traj_lines, elapsed
            else:
                print(f"  ❌ FAILED: No trajectory saved")
                return False, 0, elapsed
        else:
            print(f"  ⏰ TIMEOUT or ERROR after {elapsed:.1f}s")
            return False, 0, elapsed
            
    except Exception as e:
        print(f"  💥 EXCEPTION: {e}")
        return False, 0, 0

def main():
    print("🔍 Quick Quality Analysis of All Sequences")
    print("=" * 60)
    
    sequences = ["Basement_1", "Basement_2", "Floor7_Hallway", "Outdoor_1", "Washroom"]
    results = {}
    
    os.chdir("/home/mmaaz/SLAM_DATA/sequence_analysis")
    
    for seq in sequences:
        success, poses, time_taken = test_sequence(seq, timeout=90)
        results[seq] = {
            'success': success,
            'poses': poses,
            'time': time_taken
        }
    
    print("\n" + "="*60)
    print("📊 SUMMARY - Best Quality Sequences:")
    print("="*60)
    
    # Sort by success and number of poses
    successful = {k: v for k, v in results.items() if v['success']}
    
    if successful:
        sorted_success = sorted(successful.items(), key=lambda x: x[1]['poses'], reverse=True)
        
        for i, (seq, data) in enumerate(sorted_success):
            status = "🥇" if i == 0 else "🥈" if i == 1 else "🥉" if i == 2 else "✅"
            print(f"{status} {seq}: {data['poses']} poses in {data['time']:.1f}s")
            
        print(f"\n🎯 RECOMMENDATION:")
        print(f"Best 2 sequences: {sorted_success[0][0]} + {sorted_success[1][0]}")
        
        # Check indoor/outdoor requirements
        indoor_seqs = [s for s in sorted_success if not s[0].startswith('Outdoor')]
        outdoor_seqs = [s for s in sorted_success if s[0].startswith('Outdoor')]
        
        if outdoor_seqs and indoor_seqs:
            print(f"For CW2 (indoor+outdoor): {indoor_seqs[0][0]} + {outdoor_seqs[0][0]}")
    else:
        print("❌ No sequences worked successfully!")
    
    print("\n💡 Failed sequences had initialization problems or timeouts")

if __name__ == "__main__":
    main()