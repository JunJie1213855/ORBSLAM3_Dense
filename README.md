<<<<<<< HEAD
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
=======
## <div align="center">📄 ORBSLAM3_Dense项目介绍</div>
<p align="center">
    <!--"https://img.shields.io/github/${property}/${user}/${project}?style=${first_param}&color=${second_param}-->
    <a href="./LICENSE"><img alt="GitHub License" src="https://img.shields.io/github/license/5p6/ORBSLAM3_Dense?style=for-the-badge"></a>
    <a href="https://github.com/5p6/TensorRT-YOLO/commits"><img alt="GitHub commit activity" src="https://img.shields.io/github/commit-activity/m/5p6/ORBSLAM3_Dense?style=for-the-badge&color=rgb(47%2C154%2C231)"></a>
    <img alt="GitHub Repo stars" src="https://img.shields.io/github/stars/5p6/ORBSLAM3_Dense?style=for-the-badge&color=%2350e472">
    <img alt="GitHub forks" src="https://img.shields.io/github/forks/5p6/ORBSLAM3_Dense?style=for-the-badge&color=%2320878f">
</p>

ORBSLAM3_Dense 是一个支持深度相机、双目相机稠密重建的SLAM二次开发项目，双目相机对于针孔相机、鱼眼相机传感器类型也是支持稠密重建。

该项目使用PCL建立三维稠密点云，支持ELAS、SGBM、IGEV（基于深度学习的方法）双目稠密重建方法。


## <div align="center">🛠️ 项目环境</div>
基础版本 main 分支：
| 依赖 | 版本要求 |
|------|----------|
| Ubuntu | 20.04 |
| CMake | 3.15+ |
| OpenCV | 4.5+ |
| PCL | 1.10 |
| Eigen3 | 3.3+ |
| Pangolin | 0.6 |
| Boost | 1.71+ |

使用深度学习版本 with_trt 分支（可选）：

| 依赖 | 版本要求 |
|------|----------|
| Ubuntu | 20.04 |
| CMake | 3.15+ |
| OpenCV | 4.5+ |
| PCL | 1.10 |
| Eigen3 | 3.3+ |
| Pangolin | 0.6 |
| Boost | 1.71+ |
| CUDA | 11.x / 12.x |
| TensorRT | 10.x |


在配置好CUDA和TensorRT的时候，如果开发者的电脑上CUDA和TensorRT的路径分别为
* your_cuda_root
    * cuda
        * lib64
        * bin
        * ...
* your_tensorrt_root
    * TensorRT
        * bin  
        * include  
        * lib
        * ...


将 `./CMakeLists.txt` 和 `./Thirdparty/TensorRTTemplate/CMakeLists.txt` 中的
```cmake
set(CUDA_ROOT_DIR "/usr/local/cuda")
set(TensorRT_ROOT_DIR "/root/TensorRT")
```
重新设置
```
set(CUDA_ROOT_DIR "${your_cuda_root}/cuda")
set(TensorRT_ROOT_DIR "${your_tensorrt_root}/TensorRT")
```
具体路径设置看lib文件夹在哪。



## <div align="center">📦 源码编译</div>

### 1.基础编译环境
```bash
sudo apt install cmake git build-essential libeigen3-dev libboost-all-dev libopencv-dev libpcl-dev -y
```

### 2.Pangolin
编译：
```bash
git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
git checkout v0.6
cmake -S . -B build
cmake --build ./build -j 4
```
安装
```bash
cd build 
sudo make install
```

### 3.项目编译

当你满足项目环境时
```shell
bash build.sh
```


## <div align="center">📦 相机参数配置 </div>
见 [DataPrepare](DataPrepare.md) 和 [双目相机yaml](双目相机yaml参数解释.md)。



## <div align="center">✨ 项目示例</div>
对于代码的展示效果，可以根据链接[效果](https://zhuanlan.zhihu.com/p/694281711)查看.

### 深度相机
对于深度相机数据集，每个图像的名称必须是`1，2，3，...`的数字，因为内部采用了字符串转换为数字来排序的算法读取图像，可以使用经典的**Tum**数据集做示例.
```shell
- root_dir
    - rgb
        - 1.jpg
        - 2.jpg
        ...
    - depth
        - 1.png
        - 2.png
        ...
```
上述的数据当你配置好`rgbdslam.yaml`文件后，即可运行
```shell
./MyExample/rgbd_slam -r /root_dir
```
如果你想查看参数，添加选项 `--help`
```shell
./MyExample/rgbd_slam --help
```



### 双目相机
对于双目数据集，格式如下，图像名称也必须是数字，否则内部图像排序算法无效.
```shell
- root_dir
    - left
        - 1.jpg
        - 2.jpg
        ...
    - right
        - 1.png
        - 2.png
        ...
    - disp(optional)
        - 1.png
        - 2.png
        ...
```

#### 1.传统方法和有视差图
可以利用**作者的百度网盘** [Euroc数据集](https://pan.baidu.com/s/1SjzAdgzRN1PjRmzsQYwrvA?pwd=kyan)，示例代码的 `stereoslam.yaml` 或者 `stereoslam_disp.yaml`文件就是 `Euroc` 的配置文件，可以参照这个文件修改运行自己的数据集，即可运行
```shell
# 有视差图
./MyExample/stereoslam_disp  -l root_dir/left -r root_dir/right -d root_dir/disp
# 无视差图
./MyExample/stereoslam_disp -l root_dir/left -r root_dir/right
```
命令行添加`--help`选项，可以知晓参数怎么输入
```shell
./MyExample/stereoslam_disp --help
# 无视差图
./MyExample/stereoslam_disp --help
```


#### 2.深度学习方法
首先，数据集也必须是上文所述，另外你还需要采用获取IGEV模型的onnx权重文件，以 `Euroc` 数据集为例，本文已经提前导出了 IGEV 在图像尺寸为(480, 752) = (H, W) 的ONNX模型，在[Google云](https://drive.google.com/drive/folders/19UBgYWeEADKTA1w44HIDkzn2oPxKATOH?usp=drive_link)中可以下载 `igev_480_752.onnx`文件。在获取完模型文件后，转换为TensorRT模型，命令如下
```bash
trtexec --onnx=igev_480_752.onnx --saveEngine=./igev_480_752.engine
```

之后修改 `stereoslam.yaml`中的选项为
```yaml
Camera.width: 752           # 图像的宽
Camera.height: 480          # 图像的高 
Stereo.Type: 2              # 立体匹配算法，设置为IGEV
Stereo.TensorRTModelPath : ./igev_480_752.engine # TensorRT权重路径
```

之后运行
```
./MyExample/stereoslam -l root_dir/left \
                            -r root_dir/right \
                            -p MyExample/stereoslam.yaml \
                            -v Vocabulary/ORBvoc.txt
```

就会开始运行稠密重建了。

**自己的数据集**。如果用自己双目相机的数据该怎么做稠密重建呢？流程和上面是一致的
* 标定：获取内参、外参以及Q矩阵
* 获取立体算法ONNX权重：具体可以参考[链接](https://github.com/JunJie1213855/StereoAlgorithms/tree/main/IGEV-Stereo)下的 `export_onnx.py`，注意，好好的设计**输入图像**的**尺寸**，这是因为本文设计的TensorRT的模型，其输入数据的尺寸必须是固定的，这样TensorRT才会开更多的优化，让立体匹配计算更快。
* 转换为TensorRT权重
```bash
trtexec --onnx=${your_stereo_model}.onnx --saveEngine=${your_stereo_model}.engine
```
* 修改 `stereoslam.yaml`
* 运行

#### 3.结果查看
程序运行结束后，自动保存点云文件到当前目录：

- `PointCloudmapping.ply` - 稠密点云 (PLY 格式)

使用 MeshLab 或 CloudCompare 查看：
```bash
# 使用 MeshLab
meshlab PointCloudmapping.ply

# 使用 CloudCompare
cloudcompare PointCloudmapping.ply
```





## <div align="center">🪛 常见问题</div>

1. **编译错误: NvInfer.h not found**
   - 确保 TensorRT 路径正确配置
   - 检查 CUDA 和 TensorRT 版本兼容性

2. **点云可视化卡顿**
   - 减少 `PointCloudMapping.Resolution` 值 (如 0.05)
   - 降低 ORB 特征点数量

3. **IGEV 推理失败**
   - 确认模型路径正确
   - 检查图像尺寸是否与模型输入尺寸匹配 (默认 480x752)
>>>>>>> origin/main
