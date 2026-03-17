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
./MyExample/stereoslam_disp -l root_dir/left \
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

4. **pcl点云位姿表换失败**
   - 在PointCloudMapping.cc中用如下代码替换 `pcl::transformPointCloud(*tmp, *cloud, transform);`
```C++
cloud->width = tmp->width;
cloud->height = tmp->height;
cloud->is_dense = tmp->is_dense;
cloud->points.resize(tmp->points.size());

for (size_t i = 0; i < tmp->points.size(); ++i) {
    const PointT& pt = tmp->points[i];
    Eigen::Vector4f v(pt.x, pt.y, pt.z, 1.0);
    Eigen::Vector4f vt = transform * v;
    cloud->points[i].x = vt[0];
    cloud->points[i].y = vt[1];
    cloud->points[i].z = vt[2];
    cloud->points[i].r = pt.r;
    cloud->points[i].g = pt.g;
    cloud->points[i].b = pt.b;
}
```

5. **Ubuntu 22.04/24.04运行突然卡住**
    - Qt的定时器只能支持单线程。
    - 解决办法：将主线程的cv::imshow、cv::waitKey函数注释掉
    - 在PointCloudMapping.cc中pcl的viewer也要关掉