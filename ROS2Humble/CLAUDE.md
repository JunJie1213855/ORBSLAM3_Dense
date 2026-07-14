# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ROS2 wrapper for the ORB_SLAM3 visual SLAM library. Provides ROS2 nodes that subscribe to camera/IMU topics and feed frames into ORB_SLAM3's tracking pipeline.

- **Package name**: `orbslam3`
- **ROS2 distro**: Foxy (tested)
- **C++ standard**: C++14
- **Build system**: colcon + ament_cmake

## Build

```bash
# From colcon workspace root:
colcon build --symlink-install --packages-select orbslam3
```

**Before building**: Two hardcoded paths must be set:
- `CMakeLists.txt` line 5: `PYTHONPATH` — your ROS2 python site-packages path
- `CMakeModules/FindORB_SLAM3.cmake` line 8: `ORB_SLAM3_ROOT_DIR` — path to your ORB_SLAM3 build

## Run

All modes require a vocabulary file (`vocabulary/ORBvoc.txt`) and a YAML config (from `config/<mode>/`):

```bash
source install/local_setup.bash

# Monocular
ros2 run orbslam3 mono PATH_TO_VOCABULARY PATH_TO_YAML_CONFIG_FILE

# Stereo
ros2 run orbslam3 stereo PATH_TO_VOCABULARY PATH_TO_YAML_CONFIG_FILE BOOL_RECTIFY

# RGB-D
ros2 run orbslam3 rgbd PATH_TO_VOCABULARY PATH_TO_YAML_CONFIG_FILE

# Stereo-Inertial
ros2 run orbslam3 stereo-inertial PATH_TO_VOCABULARY PATH_TO_YAML_CONFIG_FILE BOOL_RECTIFY [BOOL_EQUALIZE]
```

## Architecture

Each SLAM mode follows an identical pattern with a mode-specific main entry point and ROS2 Node class, all in `src/<mode>/`. There are no shared base classes between modes — each mode's Node is independent.

### Common flow (all modes)

1. `main()` parses CLI args, initializes rclcpp, constructs the `ORB_SLAM3::System` with the appropriate sensor type enum, creates the ROS2 Node (passing the SLAM system pointer), and calls `rclcpp::spin()`.
2. The Node destructor calls `m_SLAM->Shutdown()` and `m_SLAM->SaveKeyFrameTrajectoryTUM()`, so trajectory is always saved on clean exit.
3. `Utility::StampToSec()` (in `include/utility.hpp`) converts ROS2 timestamps to double-precision seconds for ORB_SLAM3.

### Mode-specific details

| Mode | Executable | Node class | ROS topics subscribed | Sync mechanism |
|---|---|---|---|---|
| Mono | `mono` | `MonocularSlamNode` | `camera` (Image) | None (single topic) |
| Stereo | `stereo` | `StereoSlamNode` | `camera/left`, `camera/right` (Image) | `message_filters::ApproximateTime` |
| RGB-D | `rgbd` | `RgbdSlamNode` | `camera/rgb`, `camera/depth` (Image) | `message_filters::ApproximateTime` |
| Stereo-Inertial | `stereo-inertial` | `StereoInertialNode` | `camera/left`, `camera/right` (Image), `imu` (Imu) | Manual sync via dedicated `std::thread` + queues |

**Stereo-Inertial is the most complex node**: it runs a separate `SyncWithImu()` thread that drains IMU and image queues, matches timestamps within 10ms tolerance, optionally applies CLAHE equalization and rectification, then calls `SLAM_->TrackStereo()` with the accumulated IMU measurements vector.

**Stereo mode** optionally supports rectification; loads `LEFT.K/D/R/P` and `RIGHT.K/D/R/P` from the YAML config when `BOOL_RECTIFY` is true.

### Key dependencies (all required at build time)

- `ORB_SLAM3` (external, found via custom CMake module)
- `Pangolin` (visualization)
- `Sophus` (Lie algebra, must be installed from ORB_SLAM3's Thirdparty)
- `OpenCV` 4.2.0
- ROS2 packages: `rclcpp`, `sensor_msgs`, `cv_bridge`, `message_filters`

### Config files

Pre-configured YAML files and timestamp/IMU association files for common datasets (EuRoC, TUM, KITTI, RealSense) live under `config/<mode>/`. MultiSession subdirectories contain configs for ORB_SLAM3's multi-session mapping feature.

### CMakeModules

`FindORB_SLAM3.cmake` locates ORB_SLAM3 and its built-in dependencies (DBoW2, g2o) from a single `ORB_SLAM3_ROOT_DIR`. Sets `ORB_SLAM3_LIBRARIES` and `ORB_SLAM3_INCLUDE_DIRS`.

## Known quirks

- The `ORB_SLAM3::System` is stack-allocated in `main()` and a raw pointer is passed to the Node — the System must outlive the Node.
- Stereo-Inertial mode has a buffer size of 1 for images (old frames are dropped) but accumulates all IMU measurements.
- Launch/config/vocabulary files are NOT installed by default (the install directives are commented out in CMakeLists.txt).
- The gitignore intentionally excludes `vocabulary/ORBvoc.txt` (must be separately downloaded and extracted from the tarball).
