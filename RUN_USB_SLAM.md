# 运行USB相机和IMU的单目惯性SLAM

## 硬件设置

- **摄像头**: Logitech BRIO Ultra HD Webcam at `/dev/video4`
- **IMU**: WitMotion IMU via CH340 serial at `/dev/ttyUSB0`
  - 协议: WitMotion binary protocol (230400 baud)
  - 输出频率: ~200Hz (accelerometer + gyroscope)

## 编译状态

✅ 所有第三方库已编译:
- DBoW2
- g2o  
- Sophus

✅ ORB_SLAM3库已编译
✅ mono_inertial_usb程序已编译

## 运行步骤

### 1. 确认设备连接

检查USB设备:
```bash
# 检查摄像头
ls -l /dev/video* | grep video4

# 检查IMU串口
ls -l /dev/ttyUSB0

# 测试IMU数据
cd /home/weida/slam_new/ORB_SLAM3
timeout 5 python3 read_witmotion_imu.py
```

### 2. 运行SLAM

```bash
cd /home/weida/slam_new/ORB_SLAM3

# 基本运行命令
./Examples/Monocular-Inertial/mono_inertial_usb \
    Vocabulary/ORBvoc.txt \
    Examples/Monocular-Inertial/USB_Webcam.yaml

# 或者完整命令(指定设备路径)
./Examples/Monocular-Inertial/mono_inertial_usb \
    Vocabulary/ORBvoc.txt \
    Examples/Monocular-Inertial/USB_Webcam.yaml \
    /dev/video4 \
    /dev/ttyUSB0
```

### 3. 使用说明

**初始化要求:**
- SLAM系统需要摄像头移动来初始化
- 启动后请轻轻移动摄像头(旋转+平移)
- 避免剧烈晃动,保持平稳运动
- 观察Pangolin可视化窗口中的特征点和轨迹

**IMU数据:**
- 程序会自动读取WitMotion IMU数据(230400 baud)
- 加速度计范围: ±16g
- 陀螺仪范围: ±2000°/s
- 与相机帧同步

**退出:**
- 按 `Ctrl+C` 停止程序
- 或在Pangolin窗口按 `Esc`

## 配置文件

当前配置: `Examples/Monocular-Inertial/USB_Webcam.yaml`

**关键参数:**

```yaml
# 相机内参 (需要标定)
Camera.fx: 500.0
Camera.fy: 500.0
Camera.cx: 320.0
Camera.cy: 240.0

# IMU参数 (WitMotion)
IMU.Frequency: 200
IMU.NoiseGyro: 1.7e-4  # rad/s * 1/sqrt(Hz)
IMU.NoiseAcc: 2.0e-3   # m/s^2 * 1/sqrt(Hz)
IMU.GyroWalk: 1.9393e-05
IMU.AccWalk: 3.0e-3

# Camera-IMU外参 (需要标定)
IMU.T_b_c1: 单位矩阵 (假设对齐)
```

## ⚠️ 下一步优化

### 1. 相机标定 (必需)
当前使用的是估算值,需要精确标定:

```bash
# 使用OpenCV标定
# 1. 准备棋盘格标定板
# 2. 采集多角度图像 (20-30张)
# 3. 运行标定程序得到内参和畸变系数
```

推荐工具:
- OpenCV Camera Calibration
- MATLAB Camera Calibrator
- ROS camera_calibration包

### 2. Camera-IMU外参标定 (强烈推荐)
标定相机和IMU之间的相对位置和姿态:

```bash
# 使用Kalibr工具包
# 1. 同时采集相机图像和IMU数据
# 2. 使用AprilTag或棋盘格标定板
# 3. 运行Kalibr得到T_b_c矩阵
```

推荐工具:
- Kalibr (https://github.com/ethz-asl/kalibr)
- imu_utils + camera_calibration

### 3. IMU噪声参数优化
通过静态数据采集优化IMU噪声模型:

```bash
# 使用imu_utils工具
# 1. 将IMU静置3-4小时
# 2. 采集原始IMU数据
# 3. 计算Allan方差得到精确噪声参数
```

## 故障排查

### 问题: IMU显示乱码
**解决**: IMU使用二进制协议,不是文本。使用提供的Python脚本验证:
```bash
python3 read_witmotion_imu.py
```

### 问题: 相机无法打开
**解决**: 检查设备路径和权限
```bash
ls -l /dev/video*
sudo chmod a+rw /dev/video4  # 如果需要
v4l2-ctl --list-devices
```

### 问题: SLAM无法初始化
**可能原因**:
1. 相机没有移动 -> 需要移动相机
2. 特征点太少 -> 改善光照或纹理
3. IMU数据不稳定 -> 检查连接和波特率
4. 相机内参不准确 -> 需要重新标定

### 问题: 轨迹漂移严重
**可能原因**:
1. 相机内参不准 -> 进行相机标定
2. Camera-IMU外参不准 -> 进行外参标定  
3. IMU噪声参数不准 -> 优化噪声模型
4. 动态场景或模糊 -> 降低运动速度

## 已验证的配置

✅ 设备识别
- Webcam: /dev/video4 (640x480 @ 30fps)
- IMU: /dev/ttyUSB0 (230400 baud)

✅ IMU数据质量
- 加速度: ~(0.05, 0.05, -9.82) m/s² (静止时)
- 陀螺仪: ~(0, 0, 0) rad/s (静止时)
- 输出频率: ~200Hz

✅ WitMotion协议解析
- 数据包格式: 0x55 + type + 6字节数据 + checksum
- 加速度包 (0x51) 和陀螺仪包 (0x52) 交替
- 16位有符号整数,小端序

## 参考资料

- ORB-SLAM3论文: https://arxiv.org/abs/2007.11898
- WitMotion传感器手册: 查看产品文档
- OpenCV相机标定教程: https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html
- Kalibr标定工具: https://github.com/ethz-asl/kalibr
