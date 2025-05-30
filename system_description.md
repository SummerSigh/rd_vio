# RD-VIO: Robust Direct Visual-Inertial Odometry System

## Table of Contents
1. [System Overview](#system-overview)
2. [Architecture](#architecture)
3. [Data Flow Pipeline](#data-flow-pipeline)
4. [Core Components](#core-components)
5. [Algorithms and Techniques](#algorithms-and-techniques)
6. [Configuration System](#configuration-system)
7. [Performance Considerations](#performance-considerations)
8. [Future Extensions](#future-extensions)

## System Overview

RD-VIO (Robust Direct Visual-Inertial Odometry) is a state-of-the-art real-time visual-inertial odometry system that fuses camera and IMU measurements to estimate 6-DOF pose trajectory. 

### Key Features
- **Sliding window optimization** with marginalization
- **IMU pre-integration** for efficient high-rate sensor fusion  
- **PARSAC** (Preemptive Adaptive RANSAC) for robust outlier rejection
- **Multi-threaded architecture** for real-time performance
- **Flexible sensor support** (monocular/stereo cameras, various IMUs)
- **Bearing vector representation** for numerical stability

### Supported Platforms
- **Datasets**: EuRoC MAV, ADVIO
- **Live sensors**: Intel RealSense D435/D455
- **Operating Systems**: Linux (tested on Ubuntu 20.04+)

## Architecture

### Module Organization

The codebase is organized into several interdependent modules:

```
rd_vio/
├── src/
│   ├── rdvio/              # Core VIO pipeline
│   ├── rdvio_estimation/   # Optimization backend
│   ├── rdvio_map/          # Map data structures
│   ├── rdvio_geometry/     # Geometric algorithms
│   ├── rdvio_util/         # Utilities and helpers
│   └── rdvio_extra/        # Image processing extras
└── examples/               # Dataset/sensor interfaces
```

### Threading Model

When multi-threading is enabled, the system uses three main threads:

1. **Input Thread**: Handles sensor data acquisition and queuing
2. **Feature Tracking Thread**: Processes images and tracks features
3. **Frontend Thread**: Runs optimization and state estimation

Communication between threads uses lock-free queues to minimize latency.

## Data Flow Pipeline

### 1. Data Acquisition

#### Camera Data Flow
```
Camera → Image Capture → Grayscale Conversion → OpenCvImage Wrapper → Handler::track_camera()
```

#### IMU Data Flow  
```
IMU → Acc/Gyro Readings → Buffering → Interpolation → ImuData → Handler::track_imu()
```

### 2. Frame Processing Pipeline

```
1. Image Arrival
   ├─→ Create Frame object
   ├─→ Apply CLAHE enhancement
   └─→ Buffer until IMU data available

2. Feature Processing (FeatureTracker)
   ├─→ Predict features using IMU
   ├─→ KLT optical flow tracking
   ├─→ PARSAC outlier rejection
   └─→ New feature detection (if needed)

3. Frame Association
   ├─→ Associate IMU pre-integration
   ├─→ Link tracked features to landmarks
   └─→ Queue for optimization
```

### 3. State Estimation Pipeline

```
1. Initialization
   ├─→ Collect frames with sufficient parallax
   ├─→ SfM initialization (Essential/Homography)
   ├─→ IMU initialization (bias, scale, gravity)
   └─→ Validate and start main VIO

2. Tracking Mode
   ├─→ Predict pose using IMU
   ├─→ Visual update via PnP
   ├─→ Sliding window optimization
   ├─→ Marginalization of old states
   └─→ Landmark management
```

## Core Components

### 1. Handler (`handler.h/cpp`)

The central coordinator that:
- Manages sensor data streams
- Synchronizes visual and inertial data
- Routes frames through the pipeline
- Handles multi-threading coordination

**Key responsibilities:**
- IMU data interpolation and buffering
- Frame creation and queuing
- Thread lifecycle management
- State machine coordination

### 2. Feature Tracker (`feature_tracker.h/cpp`)

Handles all visual feature processing:

**Detection Strategy:**
- Periodic detection (not every frame)
- OpenCV GoodFeaturesToTrack or FAST
- Poisson disk filtering for spatial distribution
- Adaptive thresholds based on scene

**Tracking Pipeline:**
1. **IMU-aided prediction**: Use pre-integration to predict feature locations
2. **KLT tracking**: Lucas-Kanade optical flow with pyramids
3. **Outlier rejection**: 
   - Essential matrix RANSAC for general motion
   - Rotation-only RANSAC for low parallax
   - PARSAC for spatially-aware rejection

**Track Management:**
- Prioritizes long tracks over short ones
- Maintains track history for PARSAC
- Removes tracks with high reprojection error

### 3. Frontend (`frontend.h/cpp`)

The high-level state estimator that implements:

**Initialization:**
```cpp
1. waitForInitialization()
   - Collect 8+ keyframes
   - Ensure sufficient parallax
   
2. initializeSfM()
   - Decompose Essential/Homography
   - Triangulate initial map
   - Bundle adjustment
   
3. initializeImu()
   - Estimate gyroscope bias
   - Solve for scale and gravity
   - Initialize velocities
```

**Tracking:**
```cpp
1. processFrame()
   - Predict with IMU
   - Localize with vision
   - Decide if keyframe
   
2. optimizeWindow()
   - Bundle adjustment
   - Update landmarks
   - Maintain window size
```

### 4. Sliding Window Tracker (`sliding_window_tracker.h/cpp`)

Manages the optimization window:

**Window Structure:**
- Fixed-size sliding window (10-12 frames)
- Keyframes as main frames
- Sub-frames between keyframes
- Marginalization maintains information

**Optimization Process:**
1. **Localize**: Quick optimization for new frame
2. **Track**: Triangulate new landmarks
3. **Refine**: Full bundle adjustment
4. **Slide**: Marginalize oldest frame

### 5. Solver (`solver.h/cpp`)

The optimization backend using Ceres Solver:

**State Variables:**
- Pose: SO(3) × R³ (rotation + position)
- Motion: R⁹ (velocity + IMU biases)  
- Landmarks: Inverse depth parameterization

**Cost Functions:**
- **Visual**: Reprojection errors
- **Inertial**: Pre-integration constraints
- **Priors**: Marginalization factors

**Solver Configuration:**
```cpp
options.linear_solver_type = ceres::SPARSE_SCHUR;
options.trust_region_strategy_type = ceres::DOGLEG;
options.loss_function = new ceres::CauchyLoss(1.0);
```

### 6. Map Management (`map.h`, `frame.h`, `track.h`)

**Map** class:
- Thread-safe container for frames and tracks
- Deque for sliding window frames
- Vector for all tracks
- Reference frame management

**Frame** class:
```cpp
struct Frame {
    // Identification
    size_t id;
    double t;  // timestamp
    
    // State
    PoseState pose;      // R, p
    MotionState motion;  // v, bg, ba
    
    // Measurements  
    vector<Vector3d> keypoints;  // bearing vectors
    vector<Track*> tracks;       // associated landmarks
    
    // IMU data
    shared_ptr<PreIntegrator> preintegration;
    vector<Frame*> subframes;    // non-keyframes
};
```

**Track** class:
```cpp
struct Track {
    size_t id;
    TrackStatus status;
    
    // Observations
    map<Frame*, size_t> keypoint_map;  // frame → keypoint_idx
    
    // 3D position
    optional<InverseDepth> landmark;
    Frame* first_frame;  // anchor for inverse depth
};
```

### 7. IMU Pre-integration (`preintegrator.h/cpp`)

Implements on-manifold pre-integration following Forster et al.:

**State Propagation:**
```cpp
// Rotation update
delta_R = delta_R * Exp(omega * dt)

// Velocity update  
delta_v = delta_v + delta_R * accel * dt

// Position update
delta_p = delta_p + delta_v * dt + 0.5 * delta_R * accel * dt²
```

**Jacobian Computation:**
- Maintains derivatives w.r.t. biases
- Propagates uncertainty covariance
- Enables bias correction without re-integration

**Noise Model:**
- Continuous-time white noise
- Configurable noise densities
- Integration considers discrete time steps

### 8. Initializer (`initializer.h/cpp`)

Two-stage initialization process:

**Stage 1: Structure from Motion**
```cpp
1. Feature matching between frames
2. Compute Essential and Homography matrices
3. RANSAC to find best model
4. Decompose to get relative pose
5. Triangulate initial landmarks
6. PnP for intermediate frames
7. Bundle adjustment refinement
```

**Stage 2: IMU Alignment**
```cpp
1. Gyroscope bias estimation
   - Use rotation constraints
   - Least squares solution
   
2. Scale, gravity, velocity estimation
   - Construct linear system
   - SVD solution
   - Optional gravity refinement
```

## Algorithms and Techniques

### 1. PARSAC (Preemptive Adaptive RANSAC)

An enhanced RANSAC that maintains spatial statistics:

**Algorithm:**
```cpp
1. Divide image into spatial bins
2. Track inlier/outlier ratio per bin
3. Bias sampling towards successful regions
4. Use IMU predictions as priors
5. Adapt dynamically to scene changes
```

**Benefits:**
- Better handling of dynamic objects
- Improved convergence speed
- Spatial awareness of outliers

### 2. Bearing Vector Representation

Instead of 2D pixel coordinates, features are stored as 3D unit vectors:

```cpp
Vector3d bearing = K_inv * Vector3d(u, v, 1.0);
bearing.normalize();
```

**Advantages:**
- Unified multi-camera handling
- Numerically stable optimization
- Natural integration with SO(3)

### 3. Inverse Depth Parameterization

Landmarks use inverse depth anchored to first observation:

```cpp
Point3d = first_frame_position + depth * first_bearing_vector
where depth = 1.0 / inverse_depth
```

**Benefits:**
- Handles infinite/far points
- Better initialization uncertainty
- Improved numerical conditioning

### 4. Marginalization Strategy

When removing frames from the window:

```cpp
1. Construct information matrix from all factors
2. Partition into keep/remove blocks
3. Schur complement to marginalize
4. Eigenvalue decomposition for stability
5. Create new factor with linearization point
```

### 5. Robust Cost Functions

Uses Cauchy robust loss:
```cpp
ρ(r) = log(1 + r²/c²)
```

Applied to visual measurements to handle outliers that pass RANSAC.

## Configuration System

### Hierarchical Configuration

1. **Default values** in `Config` class
2. **Sensor configs** (e.g., `euroc_sensor.yaml`)
3. **Algorithm settings** (`setting.yaml`)

### Key Parameters

**Feature Tracking:**
```yaml
feature:
  min_distance: 30          # Minimum pixel distance between features
  max_features: 150         # Maximum features to track
  ransac_threshold: 1.0     # RANSAC inlier threshold (pixels)
  tracker_frequency: 2      # Keyframe frequency
```

**Optimization:**
```yaml
solver:
  iteration_limit: 10       # Max optimization iterations
  time_limit: 0.05         # Max optimization time (seconds)
  refine_landmarks: true   # Refine 3D points
```

**IMU:**
```yaml
imu:
  acc_noise_density: 0.01   # Accelerometer noise (m/s²/√Hz)
  gyro_noise_density: 0.001 # Gyroscope noise (rad/s/√Hz)
  acc_random_walk: 0.0001   # Accelerometer bias random walk
  gyro_random_walk: 1e-5    # Gyroscope bias random walk
```

## Performance Considerations

### Computational Complexity

- **Feature tracking**: O(N × M) where N = features, M = pixels/window
- **Bundle adjustment**: O(K³ + K²L) where K = keyframes, L = landmarks
- **Marginalization**: O(S³) where S = state dimension

### Memory Usage

- **Per frame**: ~10KB (100 features × 100 bytes)
- **Per landmark**: ~200 bytes
- **Sliding window**: ~2MB (10 frames × 200KB)

### Real-time Performance

Achieves 30+ FPS on modern hardware by:
- Multi-threading critical paths
- Efficient sparse matrix operations
- Limiting optimization iterations
- Smart keyframe selection

### Bottlenecks and Optimizations

1. **Feature tracking**: Pyramid levels, search windows
2. **Bundle adjustment**: Sparse Schur complement
3. **Marginalization**: Incremental updates
4. **Memory allocation**: Object pools for frames/tracks

## Future Extensions

### 1. Loop Closure (Planned)
- ORB feature extraction for keyframes
- DBoW vocabulary for place recognition
- g2o backend for pose graph optimization
- Global map consistency

### 2. Dense Reconstruction
- Semi-dense depth estimation
- Mesh generation from keyframes
- Real-time visualization

### 3. Multi-Camera Support
- Already uses bearing vectors
- Need extrinsic calibration
- Modified feature tracking

### 4. Semantic Integration
- Object detection for dynamic removal
- Semantic constraints in optimization
- Scene understanding

### 5. Learning-Based Components
- Learned features for tracking
- IMU denoising networks
- Adaptive parameter tuning