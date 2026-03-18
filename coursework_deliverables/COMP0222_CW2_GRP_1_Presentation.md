# COMP0222 Coursework 2 - Group 1 Presentation
## Visual and LiDAR SLAM Analysis

### Slide 1: Introduction
- **Objective**: Systematic analysis of SLAM parameter effects
- **Methods**: ORB-SLAM2 + Custom LiDAR SLAM implementation
- **Datasets**: TUM, KITTI, Intel RealSense D455, RPLiDAR

### Slide 2: Visual SLAM Parameter Analysis
**Key Findings:**
- Outlier rejection: **391% error increase** when disabled (TUM)
- Loop closure: **310% error increase** when disabled (KITTI) 
- Feature count: Minimal impact (800-1500 range)

### Slide 3: Own Sequence Collection
**Intel RealSense D455 Results:**
- Indoor Room 1: 281 poses, 1.82m trajectory
- Basement 2: 1,344 poses, 2.83m trajectory
- Successfully processed with ORB-SLAM2

### Slide 4: LiDAR SLAM Implementation
**Parameter Sensitivity Analysis:**
- Maximum range: Trade-off between accuracy and speed
- Angular resolution: Full scan optimal
- Scan rate: 50% reduction acceptable
- Voxel filtering: Fine tuning improves quality

### Slide 5: Factor Graph Optimization
**Loop Closure Results:**
- Before: 0.853m closure error
- After: 0.000m closure error  
- **100% improvement** achieved

### Slide 6: Conclusions
- Outlier rejection and loop closure are **critical**
- Parameter optimization provides measurable improvements
- Factor graph optimization essential for accurate mapping
- Systematic analysis validates theoretical predictions

**Questions & Discussion**