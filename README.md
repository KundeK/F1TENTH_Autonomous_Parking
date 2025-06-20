# F1TENTH Autonomous Parking

An autonomous parking system for F1TENTH race cars using ArUco marker detection and computer vision. The system enables precise parking maneuvers by detecting visual markers and implementing intelligent path planning with safety features.

## Overview

This project implements a complete autonomous parking solution that:
- Detects ArUco markers using computer vision
- Plans optimal parking trajectories with lookahead and parking points
- Controls vehicle speed and steering with PID feedback
- Implements safety features using LiDAR obstacle detection
- Provides real-time status monitoring and control

## System Architecture

### Core Components

1. **ArUco Detection** (`arucoDetection.py`)
   - Computer vision pipeline for marker detection
   - 6DOF pose estimation (position and orientation)
   - Camera calibration and distortion correction

2. **Parking Planner** (`aruco_parking_planner.py`)
   - Intelligent path planning with state machine
   - Dynamic target point selection (lookahead vs parking)
   - Configurable distance thresholds and speeds

3. **Speed Controller** (`controllers.py`)
   - PID-based speed control system
   - Joystick integration for manual override
   - Feedback from VESC motor controller

4. **Main Controller** (`parking_control.py`)
   - ROS node orchestrating all components
   - LiDAR-based safety system
   - Real-time control loop integration

## Features

### Parking Modes
- **Approaching**: High-speed approach using lookahead points
- **Parking**: Precision parking with reduced speed
- **Stopped**: Automatic stopping when parking is complete

### Safety Systems
- Real-time LiDAR obstacle detection
- Dynamic speed adjustment based on proximity
- Emergency stopping capabilities
- Ramer-Douglas-Peucker algorithm for efficient point processing

### Control Features
- Proportional steering control based on target angles
- Adaptive speed control with configurable PID parameters
- Joystick override for manual control
- Status monitoring and logging

## Hardware Requirements

- F1TENTH race car platform
- Intel RealSense D435i camera
- LiDAR sensor (270° field of view)
- VESC motor controller
- Joystick controller
- ArUco markers (4x4, ID 0-49, 127mm size)

## Software Dependencies

```bash
# ROS packages
sudo apt install ros-noetic-cv-bridge
sudo apt install ros-noetic-ackermann-msgs
sudo apt install ros-noetic-sensor-msgs
sudo apt install ros-noetic-geometry-msgs

# Python packages
pip install opencv-python
pip install numpy
pip install rospy
```

## Configuration

### Camera Calibration
Update the camera matrix in `parking_control.py`:
```python
camera_matrix = np.array([
    [606.0244751,   0,              325.15777588    ],
    [0,             605.73144531,   248.24261475    ],
    [0,             0,              1               ]
], dtype=np.float64)
```

### Parking Parameters
Adjust parking behavior in the `ArucoParkingPlanner` initialization:
- `distance_threshold`: Switch point between approach and parking (1.1m)
- `parking_distance`: Final parking distance from marker (0.1m)
- `lookahead_distance`: Approach targeting distance (0.7m)
- `cruise_speed`: Normal driving speed (1.2 m/s)
- `parking_speed`: Reduced parking speed (0.8 m/s)

### PID Controller Tuning
Modify speed controller parameters:
```python
self.speed_controller = SpeedController(
    kp=0.5,     # Proportional gain
    ki=0.35,    # Integral gain
    kd=0.0,     # Derivative gain
    ke=0.3      # Error gain
)
```


### ROS Topics

**Subscribed Topics:**
- `/scan` - LiDAR data for obstacle detection
- `/D435I/color/image_raw` - Camera feed for ArUco detection
- `/vesc/joy` - Joystick input for manual override
- `/vesc/sensors/core` - Motor feedback for speed control

**Published Topics:**
- `/aruco/pose` - Detected marker pose (6DOF)
- `/vesc/low_level/ackermann_cmd_mux/input/navigation` - Vehicle control commands

### Operating Modes

1. **Automatic Mode**: System autonomously detects markers and parks
2. **Manual Override**: Joystick button 5 enables/disables autonomous control
3. **Safety Mode**: LiDAR automatically stops vehicle when obstacles detected

## Algorithm Details

### State Machine
```
no_marker → approaching → parking → stopped
     ↑         ↓            ↓         ↓
     └─────────┴────────────┴─────────┘
```

### Coordinate Transformations
- Camera coordinates converted to vehicle reference frame
- 2D projection for ground-plane navigation
- Euler angle extraction from rotation vectors

### Safety Implementation
- Dynamic safety threshold based on vehicle speed
- Steering-aware obstacle detection
- Point cloud simplification for real-time processing

## Performance Characteristics

- **Detection Range**: Up to 3 meters
- **Parking Accuracy**: ±5cm positioning
- **Processing Rate**: 80Hz control loop
- **Safety Response**: <50ms obstacle detection

## Troubleshooting

### Common Issues

1. **No marker detected**
   - Check camera calibration
   - Verify marker size (127mm)
   - Ensure proper lighting conditions

2. **Inaccurate parking**
   - Recalibrate camera matrix
   - Adjust parking distance parameters
   - Check marker orientation

3. **Erratic steering**
   - Tune steering gain (default: 0.4)
   - Verify coordinate transformations
   - Check for camera distortion

### Debug Mode
Uncomment print statements in the code for detailed debugging:
```python
# In aruco_parking_planner.py
print(f"Status: {target_info['status']}")
print(f"Target Point: {target_info['target_point']}")
```
