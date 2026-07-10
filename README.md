## <div align="center">📄 ORB_SLAM3 Dense Mapping</div>
<p align="center">
    <a href="./LICENSE"><img alt="License" src="https://img.shields.io/github/license/UZ-SLAMLab/ORB_SLAM3?style=for-the-badge&color=rgb(47%2C154%2C231)"></a>
    <img alt="C++" src="https://img.shields.io/badge/C++-14-blue?style=for-the-badge">
    <img alt="CUDA" src="https://img.shields.io/badge/CUDA-13.0-green?style=for-the-badge">
    <img alt="TensorRT" src="https://img.shields.io/badge/TensorRT-10.9-76B900?style=for-the-badge">
</p>

基于 ORB-SLAM3 扩展的**稠密点云建图**项目，支持 RGB-D 和双目相机的稠密三维重建。双目部分提供 ELAS、SGBM、IGEV（TensorRT 深度学习）三种立体匹配方法。

---

## <div align="center">🛠️ 环境</div>

### 基础版（main 分支 — ELAS / SGBM）

| 依赖 | 版本 |
|------|------|
| Ubuntu | 22.04 |
| CMake | 3.15+ |
| OpenCV | 4.5+ |
| PCL | 1.12+ |
| Eigen3 | 3.4+ |
| Pangolin | 0.8 |
| Boost | 1.74+ |

### TensorRT 版（WITH_TENSORRT=ON — IGEV 深度学习）

基础版之上增加：

| 依赖 | 版本 |
|------|------|
| CUDA | 13.0 |
| TensorRT | 10.9 |

---

## <div align="center">📦 编译</div>

### 1. 系统依赖

```bash
sudo apt install cmake git build-essential \
    libeigen3-dev libboost-all-dev libopencv-dev libpcl-dev \
    libgl1-mesa-dev libglew-dev libglfw3-dev libwayland-dev \
    libssl-dev -y
```

### 2. Pangolin

```bash
git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin && git checkout v0.8
cmake -S . -B build -DBUILD_EXAMPLES=OFF -DBUILD_TESTS=OFF
cmake --build build -j$(nproc)
cd build && sudo make install
```

### 3. 项目编译

**基础版：**

```bash
cd ORB_SLAM3
bash build.sh
```

**TensorRT 版：**

修改 `CMakeLists.txt` 和 `Thirdparty/TensorRTTemplate/CMakeLists.txt` 中路径后：
```bash
# 编译 ELAS
cd Thirdparty/elas && mkdir build && cd build && cmake .. && make -j$(nproc) && cd ../../..
# 编译 TensorRTTemplate
cd Thirdparty/TensorRTTemplate && mkdir build && cd build && cmake .. && make -j$(nproc) && cd ../../..
# 编译 ORB_SLAM3
mkdir build && cd build
cmake .. -DWITH_TENSORRT=ON
make -j$(nproc)
```

> TensorRT 路径配置：将 CMakeLists.txt 中 `CUDA_ROOT_DIR` 和 `TensorRT_ROOT_DIR` 改为实际路径。

---

## <div align="center">🚀 运行 EuRoC 数据集</div>

数据集需解压（不能是 `.bag`），结构为 `mav0/cam0/data/` + `mav0/cam1/data/` + `mav0/imu0/data.csv`。

### 配置文件

| 文件 | 稠密建图 | 立体匹配 | 适用场景 |
|------|---------|---------|---------|
| `EuRoC.yaml` | ❌ | — | 纯稀疏 SLAM |
| `EuRoC_dense_elas.yaml` | ✅ | ELAS | CPU 高精度 |
| `EuRoC_dense_sgbm.yaml` | ✅ | SGBM | CPU 快速 (**推荐**) |
| `EuRoC_dense_igev.yaml` | ✅ | IGEV | GPU 深度学习 |

### 运行

```bash
cd Examples/Stereo-Inertial

# 纯稀疏
./stereo_inertial_euroc ../../Vocabulary/ORBvoc.txt ./EuRoC.yaml \
    /path/to/V1_01_easy ./EuRoC_TimeStamps/V101.txt

# CPU 稠密 (SGBM)
./stereo_inertial_euroc ../../Vocabulary/ORBvoc.txt ./EuRoC_dense_sgbm.yaml \
    /path/to/V1_01_easy ./EuRoC_TimeStamps/V101.txt

# GPU 稠密 (IGEV, 需 WITH_TENSORRT=ON)
./stereo_inertial_euroc ../../Vocabulary/ORBvoc.txt ./EuRoC_dense_igev.yaml \
    /path/to/V1_01_easy ./EuRoC_TimeStamps/V101.txt
```

运行结束后生成 `PointCloudmapping.ply`，可用 MeshLab / CloudCompare 查看。

### 稠密参数说明

```yaml
PointCloudMapping.Resolution: 0.04   # 体素滤波分辨率 (m)，越大点越稀疏
PointCloudMapping.MeanK: 10          # 离群滤波近邻数
PointCloudMapping.StdThresh: 1.0     # 离群滤波标准差阈值
PointCloudMapping.Unit: 1.0          # 深度单位: 1.0=m, 1000.0=mm

Stereo.Type: 1                       # 0=ELAS, 1=SGBM, 2=IGEV(TensorRT)
Stereo.DispMin: 0.0
Stereo.DispMax: 128.0
Stereo.TensorRTModelPath: "/path/to/model.engine"  # IGEV 专用
Stereo.InputWidth: 752               # IGEV 专用
Stereo.InputHeight: 480              # IGEV 专用
```

---

## <div align="center">🐳 Docker</div>

### 前置条件

- **基础版**：Docker 20.10+
- **TensorRT 版**：额外安装 [nvidia-container-toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)

```bash
# 验证 GPU 容器运行时
docker run --rm --gpus all nvidia/cuda:13.0-base nvidia-smi
```

### 基础版镜像

纯 CPU 环境，支持 ELAS / SGBM 稠密建图，镜像约 2.5 GB。

```bash
cd ORB_SLAM3

# 构建
docker build -f Dockerfile.base -t orbslam3-dense:base .

# 运行
docker run -it --rm \
    -v /home/ros/dataset/Euroc:/data \
    -v $(pwd)/output:/opt/orbslam3/Examples/Stereo-Inertial \
    orbslam3-dense:base \
    ./stereo_inertial_euroc \
    ../../Vocabulary/ORBvoc.txt \
    ./EuRoC_dense_sgbm.yaml \
    /data/vicon_room1/V1_01_easy \
    ./EuRoC_TimeStamps/V101.txt
```

> `-v $(pwd)/output:/opt/orbslam3/Examples/Stereo-Inertial` 将 `PointCloudmapping.ply` 保存到宿主机 `output/` 目录。

### TensorRT 版镜像

GPU 加速 IGEV 深度学习，镜像约 8 GB（含 CUDA + TensorRT）。构建前需将 TensorRT SDK 包放到项目根目录。

```bash
cd ORB_SLAM3

# 1. 准备 TensorRT SDK tarball
cp /home/ros/lib/Deploy/TensorRT-10.9.0.34.Linux.x86_64-gnu.cuda-13.0.tar.gz .

# 2. 构建（传递 tarball 文件名）
docker build -f Dockerfile.trt -t orbslam3-dense:trt .

# 3. 运行（--gpus all 挂载 GPU）
docker run -it --rm --gpus all \
    -v /home/ros/dataset/Euroc:/data \
    -v $(pwd)/weights/stereo.engine:/opt/orbslam3/weights/stereo.engine \
    -v $(pwd)/output:/opt/orbslam3/Examples/Stereo-Inertial \
    orbslam3-dense:trt \
    ./stereo_inertial_euroc \
    ../../Vocabulary/ORBvoc.txt \
    ./EuRoC_dense_igev.yaml \
    /data/vicon_room1/V1_01_easy \
    ./EuRoC_TimeStamps/V101.txt
```

| 挂载 | 说明 |
|------|------|
| `-v /path/to/dataset:/data` | EuRoC 数据集目录 |
| `-v ./weights/stereo.engine:...` | TensorRT 引擎文件 (TRT 版必选) |
| `-v $(pwd)/output:...` | 输出目录，`PointCloudmapping.ply` 保存位置 |

### 进入容器调试

```bash
# 交互式 shell
docker run -it --rm --gpus all \
    -v /home/ros/dataset/Euroc:/data \
    -v $(pwd)/weights/stereo.engine:/opt/orbslam3/weights/stereo.engine \
    orbslam3-dense:trt /bin/bash

# 在容器内手动运行
cd /opt/orbslam3/Examples/Stereo-Inertial
./stereo_inertial_euroc ../../Vocabulary/ORBvoc.txt ./EuRoC_dense_igev.yaml \
    /data/vicon_room1/V1_01_easy ./EuRoC_TimeStamps/V101.txt
```

### 镜像对比

| | `Dockerfile.base` | `Dockerfile.trt` |
|------|------|------|
| 基础镜像 | `ubuntu:22.04` | `ubuntu:22.04` |
| GPU 依赖 | ❌ | CUDA 13.0 + TensorRT 10.9 (需 nvidia-container-toolkit) |
| 稠密匹配 | ELAS / SGBM (CPU) | IGEV (TensorRT GPU) |
| 编译时间 | ~10 min | ~15 min |
| 镜像大小 | ~2.5 GB | ~8 GB |
| 稠密速度 | ELAS: 2-5s/KF, SGBM: 0.2-0.5s/KF | IGEV: 0.05-0.1s/KF |

---

## <div align="center">📁 核心文件</div>

```
ORB_SLAM3/
├── include/PointCloudMapping.h     # 稠密点云建图类
├── include/StereoMatch.h           # 双目匹配 (ELAS/SGBM/IGEV)
├── src/PointCloudMapping.cc        # 稠密建图实现（手动滤波）
├── src/StereoMatch.cc              # 双目匹配实现
├── Thirdparty/elas/                # ELAS 立体匹配库
├── Thirdparty/TensorRTTemplate/    # TensorRT 推理封装
├── Examples/Stereo-Inertial/
│   ├── EuRoC.yaml                  # 纯稀疏
│   ├── EuRoC_dense_elas.yaml       # ELAS 稠密
│   ├── EuRoC_dense_sgbm.yaml       # SGBM 稠密
│   └── EuRoC_dense_igev.yaml       # IGEV 稠密
├── Dockerfile.base                 # 基础版 Docker
├── Dockerfile.trt                  # TensorRT 版 Docker
├── docs/problem.md                 # 迁移记录 & 问题排查
└── weights/stereo.engine           # TensorRT 引擎 (133MB)
```

---

## <div align="center">🪛 常见问题</div>

1. **`Q.size() == Size(4,4)` 断言失败**
   - YAML 缺少稠密建图参数。确保使用 `EuRoC_dense_*.yaml`

2. **Ctrl+C 后程序不退出 / 线程卡死**
   - 手动滤波大数据量时耗时较长，可增大 `PointCloudMapping.Resolution`（如 0.1）
   - shutdown 采用轮询机制，最多等 500ms 后会重新通知线程退出

3. **`NvInfer.h` not found**
   - 基础版默认 `WITH_TENSORRT=OFF`，不会编译 TensorRT 代码
   - TRT 版确保路径配置正确，且 TensorRT SDK 已安装

4. **IGEV 推理失败**
   - 检查 `.engine` 路径和图像尺寸匹配 (默认 752×480)
   - 确认 TensorRT 和 CUDA 版本兼容

5. **点云文件点数过少**
   - 降低 `StdThresh` 放宽离群滤波
   - 检查 `Unit` 深度单位是否正确
