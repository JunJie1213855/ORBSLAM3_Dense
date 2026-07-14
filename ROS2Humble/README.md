# ORB_SLAM3 ROS2 Wrapper — Dense Mapping Edition

ROS2 Humble wrapper for ORB_SLAM3 with dense point cloud reconstruction. Supports Mono / Stereo / RGB-D / Stereo-Inertial modes. Dense mapping is enabled via YAML config.

---

## Prerequisites

| Dependency | Version |
|------------|---------|
| Ubuntu | 22.04 |
| ROS2 | Humble |
| OpenCV | 4.5+ |
| C++ | 14 |
| ORB_SLAM3 | This project (with dense mapping) |

### Install ROS2 dependencies

```bash
bash install_dep_humble.sh
# Or:
sudo apt install ros-humble-vision-opencv ros-humble-message-filters
```

System deps for ORB_SLAM3:
```bash
sudo apt install cmake build-essential libeigen3-dev libboost-all-dev \
    libopencv-dev libpcl-dev libgl1-mesa-dev libglew-dev libglfw3-dev libssl-dev -y
```

### Pangolin

```bash
git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin && git checkout v0.8
cmake -S . -B build -DBUILD_EXAMPLES=OFF
cmake --build build -j$(nproc)
cd build && sudo make install
```

### Sophus

```bash
cd ORB_SLAM3/Thirdparty/Sophus/build
sudo make install
```

---

## Build

### 1. Configure paths

**`CMakeLists.txt` line 5** — PYTHONPATH (Humble uses Python 3.10):
```cmake
set(ENV{PYTHONPATH} "/opt/ros/humble/lib/python3.10/site-packages/")
```

**`CMakeModules/FindORB_SLAM3.cmake` line 8** — ORB_SLAM3 root:
```cmake
set(ORB_SLAM3_ROOT_DIR "/home/ros/lib/SLAM/VIO/orbslam3_dense_new/ORB_SLAM3")
```

### 2. Build ORB_SLAM3 library (if not done)

```bash
cd /path/to/ORB_SLAM3
bash build.sh                         # CPU dense (ELAS/SGBM)
# cmake -DWITH_TENSORRT=ON ..         # GPU dense (IGEV TensorRT)
```

### 3. colcon build

```bash
cd ~/colcon_ws
ln -sf /path/to/ORB_SLAM3/ROS2Humble src/orbslam3

# Extract ORB vocabulary
cd src/orbslam3/vocabulary && tar -xf ORBvoc.txt.tar.gz

# Build
colcon build --symlink-install --packages-select orbslam3
```

---

## Run

```bash
source install/local_setup.bash

# Mono
ros2 run orbslam3 mono PATH_TO_VOCABULARY PATH_TO_YAML_CONFIG_FILE

# Stereo
ros2 run orbslam3 stereo PATH_TO_VOCABULARY PATH_TO_YAML_CONFIG_FILE BOOL_RECTIFY

# RGB-D
ros2 run orbslam3 rgbd PATH_TO_VOCABULARY PATH_TO_YAML_CONFIG_FILE

# Stereo-Inertial
ros2 run orbslam3 stereo-inertial PATH_TO_VOCABULARY PATH_TO_YAML_CONFIG_FILE BOOL_RECTIFY [BOOL_EQUALIZE]
```

### Topic interface

| Mode | Subscribed topics | Sync |
|------|------------------|------|
| Mono | `/camera` (Image) | None |
| Stereo | `/camera/left`, `/camera/right` (Image) | `ApproximateTime` |
| RGB-D | `/camera/rgb`, `/camera/depth` (Image) | `ApproximateTime` |
| Stereo-Inertial | `/camera/left`, `/camera/right` (Image), `/imu` (Imu) | Manual thread + 10ms tolerance |

---

## Dense mapping

Add these parameters to your YAML config:

```yaml
# Point cloud (required for all dense modes)
PointCloudMapping.Resolution: 0.04
PointCloudMapping.MeanK: 10
PointCloudMapping.StdThresh: 1.0
PointCloudMapping.Unit: 1.0           # 1.0=meters, 1000.0=mm

# Stereo matching (required for Stereo / Stereo-Inertial dense mode)
Stereo.Type: 1                        # 0=ELAS, 1=SGBM, 2=IGEV(TensorRT)
Stereo.DispMin: 0.0
Stereo.DispMax: 128.0
```

- **RGB-D mode:** PointCloudMapping params only (no Stereo section needed).
- **Stereo / Stereo-Inertial:** Both `PointCloudMapping.*` AND the `Stereo.*` section are required — the presence of `Stereo.DispMin/Max` triggers the stereo dense path.
- **IGEV GPU:** Build with `-DWITH_TENSORRT=ON`, then add `Stereo.Type: 2`, `Stereo.TensorRTModelPath`, `Stereo.InputWidth/Height`.

`PointCloudmapping.ply` is saved on clean shutdown.

---

## Signal handling

Graceful shutdown (saves trajectory + point cloud):

| Signal | Trigger | Handled |
|--------|---------|---------|
| `SIGINT` | Ctrl+C | ✅ |
| `SIGTERM` | `kill <pid>` | ✅ |
| `SIGQUIT` | `kill -3 <pid>` | ✅ |
| `SIGKILL` | `kill -9 <pid>` | ❌ kernel-enforced, uncatchable |

---

## Run with rosbag (EuRoC .bag → ROS1 bridge → ROS2)

```bash
# Shell A: roscore
source /opt/ros/noetic/setup.bash && roscore

# Shell B: bridge
source /opt/ros/noetic/setup.bash && source /opt/ros/humble/setup.bash
ros2 run ros1_bridge dynamic_bridge

# Shell C: play bag (ROS1 side)
source /opt/ros/noetic/setup.bash
rosbag play V1_02_medium.bag --pause \
    /cam0/image_raw:=/camera/left /cam1/image_raw:=/camera/right /imu0:=/imu

# Shell D: SLAM (ROS2 side)
source /opt/ros/humble/setup.bash
ros2 run orbslam3 stereo-inertial vocabulary/ORBvoc.txt config/stereo-inertial/EuRoC.yaml false
```

> Press spacebar in Shell C to start playback. For extracted image sequences, use the native `Examples/Stereo-Inertial/stereo_inertial_euroc` instead.

---

## File structure

```
ROS2Humble/
├── CMakeLists.txt
├── CMakeModules/FindORB_SLAM3.cmake
├── package.xml
├── install_dep_humble.sh
├── src/
│   ├── monocular/          # Mono node
│   ├── stereo/             # Stereo node
│   ├── rgbd/               # RGB-D node
│   └── stereo-inertial/    # Stereo-IMU node (most complex)
├── include/utility.hpp     # Timestamp converter
├── config/                 # Pre-built YAML configs per mode
│   ├── monocular/
│   ├── monocular-inertial/
│   ├── rgb-d/
│   ├── rgb-d-inertial/
│   ├── stereo/
│   └── stereo-inertial/
└── vocabulary/ORBvoc.txt.tar.gz
```

---

## Troubleshooting

1. **`sophus/se3.hpp` not found** — `cd ORB_SLAM3/Thirdparty/Sophus/build && sudo make install`
2. **ORB_SLAM3 not found** — verify `ORB_SLAM3_ROOT_DIR` in `FindORB_SLAM3.cmake`
3. **PYTHONPATH error** — Humble → `python3.10`, Foxy → `python3.8`; check CMakeLists.txt line 5
4. **No ply output** — confirm YAML has `PointCloudMapping.*` params; for Stereo modes also need `Stereo.DispMin/Max`; ensure clean shutdown (Ctrl+C / `kill`, NOT `kill -9`)
5. **Qt/Pangolin window conflict** — if running headless, set `bUseViewer=false` in the Euroc example, or the headless try/catch in Pangolin will handle it gracefully
