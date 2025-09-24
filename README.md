## <div align="center">📄 ORBSLAM3_Dense项目介绍</div>
<p align="center">
    <!--"https://img.shields.io/github/${property}/${user}/${project}?style=${first_param}&color=${second_param}-->
    <a href="./LICENSE"><img alt="GitHub License" src="https://img.shields.io/github/license/5p6/ORBSLAM3_Dense?style=for-the-badge"></a>
    <a href="https://github.com/5p6/TensorRT-YOLO/commits"><img alt="GitHub commit activity" src="https://img.shields.io/github/commit-activity/m/5p6/ORBSLAM3_Dense?style=for-the-badge&color=rgb(47%2C154%2C231)"></a>
    <img alt="GitHub Repo stars" src="https://img.shields.io/github/stars/5p6/ORBSLAM3_Dense?style=for-the-badge&color=%2350e472">
    <img alt="GitHub forks" src="https://img.shields.io/github/forks/5p6/ORBSLAM3_Dense?style=for-the-badge&color=%2320878f">
</p>

ORBSLAM3_Dense 是一个支持深度相机、双目相机稠密重建的SLam二次开发项目，并且对于针孔相机和鱼眼相机传感器类型也是支持稠密重建。该项目不仅使用了PCL建立三维稠密点云图，还在双目没有提供视差图时采用了ELas算法计算双目视差图，以达到快速开始使用的目的。


## <div align="center">🛠️ 项目环境</div>
* Ubuntu 20.04
* Boost 1.71.0
* Eigen3
* Pangolin 0.6版本
* PCL 1.10
* OpenCV 4.5.5




## <div align="center">📦 源码编译</div>
当你满足项目环境时
```shell
sh build.sh
```
## <div align="center">📦 数据集准备 </div>
见 [DataPrepare](DataPrepare.md)。

## <div align="center">✨ 项目示例</div>
### 深度相机
对于深度相机数据集,每个图像的名称必须是`1,2,3,...`的数字,因为内部采用了字符串转换为数字来排序的算法读取图像,可以使用经典的**Tum**数据集做示例.
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
上述的数据当你配置好`rgbdslam.yaml`文件后,即可运行
```shell
./MyExample/rgbd_slam -r /root_dir
```
如果你想查看参数,添加选项 `--help`
```shell
./MyExample/rgbd_slam --help
```



### 双目相机
对于双目数据集,格式如下,图像名称也必须是数字,否则内部图像排序算法无效.
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
可以利用**作者的百度网盘** `Euroc`[数据集](https://pan.baidu.com/s/1SjzAdgzRN1PjRmzsQYwrvA?pwd=kyan),示例代码的 `stereoslam.yaml` 或者 `stereoslam_disp.yaml`文件就是 `Euroc` 的配置文件,可以参照这个文件修改运行自己的数据集,即可运行
```shell
# 有视差图
./MyExample/stereoslam_disp  -l root_dir/left -r root_dir/right -d root_dir/disp
# 无视差图
./MyExample/stereoslam_disp -l root_dir/left -r root_dir/right
```
命令行添加`--help`选项,可以知晓参数怎么输入
```shell
./MyExample/stereoslam_disp --help
# 无视差图
./MyExample/stereoslam_disp --help
```

对于上述代码的展示效果，可以根据链接[效果](https://zhuanlan.zhihu.com/p/694281711) 查看.
