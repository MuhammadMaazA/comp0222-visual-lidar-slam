# COMP0222 Coursework 2: Visual and LiDAR SLAM Requirements

**Course:** Computer Vision and AI
**Assessment:** Coursework 2 (100 marks total)
**Deadline:** [As specified in coursework brief]
**Group Size:** Individual work

## 📋 General Submission Requirements

### File Naming Convention (Requirements 1-4)
- **All files must follow:** `COMP0222_CW2_GRP_g.*` format
- **Where g =** your group number
- **Report:** `COMP0222_CW2_GRP_g.pdf`
- **Code:** `COMP0222_CW2_GRP_g.zip`
- **Videos:** `COMP0222_CW2_GRP_g_[description].mp4`

### Submission Package
- **Main report:** PDF format, maximum 8 pages
- **Source code:** Complete implementation in zip file
- **Videos:** Screen recordings as specified per question
- **Data:** Results files and experimental data

---

## 🎯 Question 1: Visual SLAM with Standard Datasets (30 marks)

### Q1a: Baseline Evaluation (15 marks)
**Requirements:**
- Use **KITTI sequence 07** and **TUM dataset**
- Run ORB-SLAM2 on both datasets
- Generate trajectory files in TUM format
- Evaluate using **EVO toolkit** with ATE metrics
- Create plots showing trajectory comparison with ground truth

**Deliverables:**
- Trajectory files: `kitti07-baseline.txt`, `tum-baseline.txt`
- EVO evaluation plots (trajectory and error plots)
- Analysis of tracking performance on both datasets

### Q1b: Feature Parameter Analysis (5 marks)
**Requirements:**
- Test **3 different ORB feature counts:** 800, 1200, 1500
- Run on same datasets as Q1a
- Compare tracking accuracy vs baseline (1000 features)
- Document parameter changes in configuration files

**Deliverables:**
- Trajectory files for each configuration
- Comparative analysis of feature count impact
- Configuration files showing parameter modifications

### Q1c: Outlier Rejection Analysis (5 marks)
**Requirements:**
- **Disable outlier rejection** in ORB-SLAM2
- Run on both KITTI and TUM datasets
- Compare tracking accuracy with/without outlier rejection
- Analyze failure modes and drift patterns

**Deliverables:**
- Trajectory files: `kitti07-nooutlier.txt`, `tum-nooutlier.txt`
- Analysis of outlier rejection importance
- Error comparison plots

### Q1d: Loop Closure Analysis (5 marks)
**Requirements:**
- **Disable loop closure** in ORB-SLAM2
- Run on both KITTI and TUM datasets
- Compare tracking accuracy with/without loop closure
- Analyze cumulative drift and error patterns

**Deliverables:**
- Trajectory files: `kitti07-noloop.txt`, `tum-noloop.txt`
- Analysis of loop closure impact
- Drift accumulation analysis

---

## 📹 Question 2: Visual SLAM with Your Own Sequences (33 marks)

### Q2a: Data Collection (20 marks)
**Requirements:**
- Collect **2 indoor + 1 outdoor** sequences
- Use **Intel RealSense D455** or equivalent RGB-D camera
- **Minimum requirements per sequence:**
  - Indoor: 500+ frames, textured environment
  - Outdoor: 1000+ frames, include loop closures
- Include camera calibration information
- Ensure good lighting and texture for feature matching

**Deliverables:**
- 3 complete RGB-D sequences with timestamps
- Camera intrinsic parameters (K matrix, distortion coefficients)
- Sequence metadata (duration, frame count, environment description)

### Q2b: COLMAP Reconstruction (10 marks)
**Requirements:**
- Run **COLMAP Structure-from-Motion** on your sequences
- Generate sparse 3D reconstruction
- Extract camera trajectory from COLMAP results
- Compare with ORB-SLAM2 trajectory on same data

**Deliverables:**
- COLMAP sparse reconstruction (points3D.txt, cameras.txt, images.txt)
- Camera trajectory in TUM format
- 3D point cloud visualization
- Trajectory comparison analysis vs ORB-SLAM2

### Q2c: ORB-SLAM2 Processing + Video (3 marks)
**Requirements:**
- Process your sequences with ORB-SLAM2
- **Create screen recording video** showing:
  - Live tracking visualization
  - 3D point cloud reconstruction
  - Camera trajectory in real-time
- Include narrative explaining the processing

**Deliverables:**
- **Video file:** `COMP0222_CW2_GRP_g_Visual_SLAM.mp4`
- ORB-SLAM2 trajectory files for your sequences
- Analysis of tracking quality and map consistency

---

## 🔴 Question 3: LiDAR SLAM (37 marks)

### Q3a: LiDAR Data Collection (4 marks)
**Requirements:**
- Collect **2 indoor + 1 outdoor** LiDAR sequences
- Use **Intel RealSense L515** or equivalent LiDAR sensor
- **Each sequence requirements:**
  - Minimum 2 complete loops for loop closure
  - Varied environments (corridors, rooms, open spaces)
  - Sufficient point cloud density for ICP registration

**Deliverables:**
- 3 LiDAR sequences in standard format (PLY, PCD, or ROS bag)
- Sensor specifications and calibration data
- Sequence documentation (path, environment, duration)

### Q3b: Parameter Sensitivity Analysis (10 marks)
**Requirements:**
- Implement **LiDAR SLAM with ICP registration**
- Test sensitivity to **4 key parameters:**
  1. **Maximum correspondence distance** (0.1m, 0.5m, 1.0m)
  2. **Voxel grid resolution** (0.05m, 0.1m, 0.2m)
  3. **Keyframe insertion threshold** (1m, 2m, 5m translation)
  4. **Scan matching frequency** (every scan, every 5 scans, every 10 scans)

**Deliverables:**
- Parameter experiment results table
- Trajectory accuracy comparison for each parameter set
- Processing time analysis
- Recommendation for optimal parameter selection

### Q3c: Loop Closure Detection (8 marks)
**Requirements:**
- Implement **loop closure detection** using scan matching
- Test on sequences with known loop closures
- Measure loop closure detection accuracy:
  - True positives, false positives, false negatives
  - Closure detection distance thresholds
  - Map consistency before/after closure

**Deliverables:**
- Loop closure detection algorithm implementation
- Performance metrics (precision, recall, F1-score)
- Map quality analysis with/without loop closure
- Visualization of detected loop closures

### Q3d: Factor Graph Optimization (10 marks)
**Requirements:**
- Implement **factor graph optimization** (use G2O or GTSAM)
- Include factors for:
  - Odometry constraints (scan-to-scan)
  - Loop closure constraints
  - Optional: GPS or other sensor constraints
- Compare optimization before/after loop closure correction

**Deliverables:**
- Factor graph implementation code
- Optimization results showing pose correction
- Trajectory accuracy improvement metrics
- Graph structure visualization

### Q3e: Real-time Mapping Video (5 marks)
**Requirements:**
- **Create screen recording video** showing:
  - Real-time LiDAR scan processing
  - Incremental map building
  - Loop closure detection and correction
  - Final optimized trajectory and map
- Include all 3 sequences in the video

**Deliverables:**
- **Video file:** `COMP0222_CW2_GRP_g_LiDAR_SLAM.mp4`
- Demonstration of real-time processing capability
- Commentary on mapping quality and performance

---

## 📊 Technical Requirements

### Implementation Standards
- **Programming languages:** Python, C++, MATLAB (choose one consistently)
- **Libraries allowed:** OpenCV, PCL, Open3D, g2o, GTSAM, ROS
- **Code quality:** Well-commented, modular, reproducible
- **Performance:** Real-time capable for reasonable input sizes

### Evaluation Metrics
- **Trajectory accuracy:** ATE (Absolute Trajectory Error) using EVO
- **Processing speed:** Frame rate, computation time per operation
- **Map quality:** Point cloud density, consistency, completeness
- **Robustness:** Performance across different environments

### Hardware Recommendations
- **Camera:** Intel RealSense D455 (RGB-D), Kinect, or equivalent
- **LiDAR:** Intel RealSense L515, Velodyne VLP-16, or equivalent
- **Computing:** Multi-core CPU, GPU recommended for real-time processing
- **Storage:** Sufficient space for raw sensor data and results

---

## 🎯 Marking Scheme Summary

| Component | Marks | Key Assessment Criteria |
|-----------|-------|-------------------------|
| **Q1: Standard Datasets** | 30 | Algorithm understanding, parameter analysis |
| **Q1a: Baseline** | 15 | Correct evaluation methodology, trajectory accuracy |
| **Q1b: Features** | 5 | Systematic parameter testing, analysis quality |
| **Q1c: Outlier Rejection** | 5 | Understanding of robust estimation |
| **Q1d: Loop Closure** | 5 | Analysis of drift and correction |
| **Q2: Custom Visual SLAM** | 33 | Data collection quality, method comparison |
| **Q2a: Data Collection** | 20 | Sequence quality, calibration accuracy |
| **Q2b: COLMAP** | 10 | Reconstruction quality, trajectory comparison |
| **Q2c: Video** | 3 | Demonstration quality, explanation clarity |
| **Q3: LiDAR SLAM** | 37 | Implementation complexity, performance analysis |
| **Q3a: Data Collection** | 4 | LiDAR sequence quality and variety |
| **Q3b: Parameter Analysis** | 10 | Systematic experimentation, optimization |
| **Q3c: Loop Closure** | 8 | Detection accuracy, map improvement |
| **Q3d: Factor Graph** | 10 | Advanced optimization implementation |
| **Q3e: Video** | 5 | Real-time demonstration capability |
| **Total** | **100** | **Overall technical quality and presentation** |

---

## ⚠️ Critical Success Factors

### Must-Have Elements
1. **Correct file naming** throughout submission
2. **Screen recording videos** for Q2c and Q3e
3. **Complete source code** with clear documentation
4. **Quantitative evaluation** using standard metrics
5. **Professional report** with clear methodology

### Common Pitfalls to Avoid
- Missing trajectory evaluation with EVO toolkit
- Insufficient data collection (too few frames or environments)
- Poor quality videos (no audio explanation, unclear visualization)
- Incomplete parameter analysis (missing systematic testing)
- Code that doesn't run or lacks documentation

### Excellence Criteria
- **Beyond requirements:** Additional sensors, algorithms, or analysis
- **Technical depth:** Advanced optimization, robust error handling
- **Professional presentation:** Clear writing, excellent visualizations
- **Innovation:** Creative solutions to technical challenges
- **Reproducibility:** Complete methodology enabling replication

---

**Note:** This document provides comprehensive coverage of all coursework requirements. Ensure each deliverable meets the specified criteria and maintains professional academic standards throughout the submission.