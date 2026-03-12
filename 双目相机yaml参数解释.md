### 基础相机参数
```yaml
Camera.type: "PinHole"           # 相机类型: PinHole 或 Fisheye

# 左相机内参
Camera1.fx: 468.0
Camera1.fy: 468.3
Camera1.cx: 375.5
Camera1.cy: 239.5

# 左相机畸变
Camera1.k1: -0.29
Camera1.k2: 0.09
Camera1.p1: 0.001
Camera1.p2: -0.001

# 右相机内参和畸变 (双目)
Camera2.fx: 466.3
Camera2.fy: 466.9
Camera2.cx: 375.5
Camera2.cy: 239.5
Camera2.k1: -0.28
Camera2.k2: 0.08
Camera2.p1: 0.001
Camera2.p2: -0.0005

Camera.width: 752               # 图像宽度
Camera.height: 480              # 图像高度
Camera.fps: 20                  # 帧率
Camera.RGB: 1                   # 颜色通道顺序: 0=BGR, 1=RGB
```

### 双目参数

```yaml
Stereo.ThDepth: 60.0            # 深度阈值 (基线倍数)

# 双目外参 (左目到右目的变换矩阵)
Stereo.T_c1_c2: !!opencv-matrix
  rows: 4
  cols: 4
  dt: f
  data: [0.999577, -0.00514625, -0.0286285, 0.110106,
         0.00428911, 0.999543, -0.0299215, 0.000647657,
         0.0287694, 0.029786, 0.999142, 0.00280352,
         0, 0, 0, 1]

# 重投影矩阵 Q (opencv 双目标定获取)
Stereo.Q: !!opencv-matrix
  rows: 4
  cols: 4
  dt: d
  data: [1., 0., 0., -3.62e+02,
         0., 1., 0., -2.42e+02,
         0., 0., 0., 4.48e+02,
         0., 0., 9.07e-03, 0.]
```

### 立体匹配参数

```yaml
# 立体匹配算法选择
# 0=ELAS, 1=SGBM, 2=IGEV, 3=LiteAnyStereo
Stereo.Type: 2

# TensorRT 模型路径 (IGEV/LiteAnyStereo 需要)
Stereo.TensorRTModelPath: "/path/to/model.engine"

# 视差范围 (ELAS/SGBM 使用)
Stereo.DispMin: 0.0
Stereo.DispMax: 128.0
```

### 点云建图参数

```yaml
PointCloudMapping.Resolution: 0.01   # 体素滤波分辨率
PointCloudMapping.MeanK: 3.0         # 离群点滤波邻居数
PointCloudMapping.StdThresh: 1.0     # 离群点滤波标准差阈值
PointCloudMapping.Unit: 1.0          # 单位: 1.0=m, 1000.0=mm