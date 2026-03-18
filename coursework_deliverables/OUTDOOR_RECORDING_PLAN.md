# Outdoor Sequence Recording Plan

## Equipment Required
- **Visual**: Intel RealSense D455 camera (or mobile phone as backup)
- **LiDAR**: RPLiDAR A1M8 scanning laser
- **Laptop**: For data recording and initial verification

## Visual SLAM Outdoor Sequence (Q2a)

### Location Strategy
- **Recommended**: Outdoor courtyard, garden, or open area
- **Requirements**: 
  - Sufficient texture and features
  - Good lighting conditions  
  - Minimum 500 frames after initialization

### Recording Procedure
1. **Setup**: Mount camera securely, ensure stable recording
2. **Initialization**: Start in feature-rich area (building corner, textured wall)
3. **Path**: Create loop trajectory with good baselines between keyframes
4. **Features**: Include distant objects for scale estimation
5. **Duration**: Record 30-60 seconds at 30fps (900-1800 frames)

### Success Criteria
- **ORB-SLAM2 initialization**: Green tracking status
- **Loop closure**: Return to starting area for drift correction
- **Map points**: Target >200 map points for robust tracking

## LiDAR SLAM Outdoor Sequence (Q3a)

### Location Requirements
- **Environment**: Large open area (courtyard, parking lot)
- **Obstacles**: Some walls/objects for mapping reference
- **Size**: Minimum 20m x 20m area

### Recording Procedure
1. **Marker Placement**: Place visible start/end marker
2. **First Loop**: Complete full perimeter (clockwise)
3. **Second Loop**: Repeat same path (counter-clockwise or same direction)  
4. **Return**: End exactly at start marker
5. **Recording**: Ensure continuous LiDAR data throughout

### Success Criteria
- **Closed-loop trajectory**: Return to within 0.5m of start
- **Map consistency**: Clear occupancy grid of environment
- **Data quality**: >1000 valid LiDAR scans recorded

## Data Processing Workflow

### Immediate Post-Recording
1. **Verification**: Quick check that files recorded properly
2. **Backup**: Copy raw data to multiple locations
3. **Initial Processing**: Test ORB-SLAM2/LiDAR SLAM on sequences

### Analysis Pipeline
1. **Visual SLAM**: Process with ORB-SLAM2, compare to indoor results
2. **LiDAR SLAM**: Run parameter analysis, compare indoor vs outdoor
3. **Documentation**: Record environmental differences and challenges

## Timeline Estimate
- **Setup and recording**: 2-3 hours
- **Initial verification**: 30 minutes  
- **Full processing**: 2-3 hours
- **Documentation**: 1 hour

## Risk Mitigation
- **Weather**: Plan for dry, moderate lighting conditions
- **Equipment**: Test all equipment before going outdoors
- **Backup plan**: Have mobile phone ready as camera backup
- **Location**: Scout location beforehand for optimal features

## Expected Challenges
- **Scale drift**: Outdoor sequences more prone to scale drift
- **Feature matching**: Natural outdoor scenes may have fewer distinct features
- **Lighting**: Variable outdoor lighting affects feature detection
- **Range limitations**: LiDAR range may limit mapping in very open areas

## Success Definition
Successful outdoor data collection will provide:
- **Q2**: Outdoor visual sequence for COLMAP/ORB-SLAM2 comparison
- **Q3**: Outdoor LiDAR sequence for environment comparison analysis
- **Complete coursework requirements**: All missing outdoor elements addressed