# USB Device Check and Mono-Inertial SLAM Setup

## Device Detection Summary

### ✅ USB Devices Found

1. **Webcam: Logitech BRIO Ultra HD Webcam**
   - Bus 003 Device 022
   - Device path: `/dev/video4` (or `/dev/video5`, `/dev/video6`, `/dev/video7`)
   - Multiple video devices are streams from the same camera

2. **IMU: USB Serial Device (CH340 converter)**
   - Bus 003 Device 021  
   - Serial port: `/dev/ttyUSB0`
   - This is likely your USB IMU connected via a CH340 USB-to-serial converter

## Files Created

### 1. Configuration File
**Location:** `Examples/Monocular-Inertial/USB_Webcam.yaml`

This YAML configuration file contains:
- Camera parameters (you'll need to calibrate for accurate values)
- IMU noise parameters (adjust based on your specific IMU specs)
- ORB feature extraction parameters
- Viewer settings

### 2. C++ Source Code
**Location:** `Examples/Monocular-Inertial/mono_inertial_usb.cc`

A custom mono-inertial SLAM program that:
- Reads frames from USB webcam
- Reads IMU data from serial port
- Feeds data to ORB-SLAM3
- Displays tracking results

### 3. Test Script
**Location:** `test_devices.py`

Python script to test your devices before running SLAM.

## Next Steps

### Step 1: Test Your Devices

Run the test script to verify webcam and IMU connectivity:

```bash
cd /home/weida/slam_new/ORB_SLAM3
python3 test_devices.py
```

**Note:** If you get permission errors for the IMU, add yourself to the dialout group:
```bash
sudo usermod -a -G dialout $USER
# Then log out and log back in
```

### Step 2: Fix ORB-SLAM3 Build Issues

The ORB-SLAM3 project has compilation errors with C++11. You need to update it to use C++14:

**Option A: Modify CMakeLists.txt**

Find this line in `CMakeLists.txt`:
```cmake
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
```

Change it to:
```cmake
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
```

**Option B: Build the project properly**

Run the build script which should handle dependencies:
```bash
cd /home/weida/slam_new/ORB_SLAM3
./build.sh
```

### Step 3: Characterize Your IMU

Before running mono-inertial SLAM, you need to know:

1. **IMU Data Format**: What format does your IMU output data in?
   - Check the IMU's datasheet or documentation
   - Common formats: `ax ay az gx gy gz` or comma-separated values
   
2. **IMU Noise Parameters**: Update these in `USB_Webcam.yaml`:
   - `IMU.NoiseGyro`: Gyroscope noise density
   - `IMU.NoiseAcc`: Accelerometer noise density  
   - `IMU.GyroWalk`: Gyroscope random walk
   - `IMU.AccWalk`: Accelerometer random walk
   
3. **IMU Frequency**: The rate at which IMU publishes data (default: 200 Hz)

4. **IMU Serial Settings**: 
   - Baudrate (default in code: 115200)
   - Data format/protocol

### Step 4: Calibrate Your Camera

You need to calibrate the Logitech BRIO for accurate SLAM:

```bash
# Use OpenCV calibration or MATLAB Camera Calibrator
# Update these parameters in USB_Webcam.yaml:
# - Camera.fx, Camera.fy (focal lengths)
# - Camera.cx, Camera.cy (principal point)
# - Camera.k1, Camera.k2, Camera.k3 (radial distortion)
# - Camera.p1, Camera.p2 (tangential distortion)
```

### Step 5: Calibrate Camera-IMU Extrinsics

You need the transformation between camera and IMU frames:

- Update `IMU.T_b_c1` in `USB_Webcam.yaml`
- This is a 4x4 transformation matrix
- Tools: Kalibr, or manual measurement

### Step 6: Update IMU Reading Code

The `mono_inertial_usb.cc` file has a placeholder for IMU reading. You need to:

1. Understand your IMU's data protocol
2. Implement the `readImuData()` function to parse your specific IMU format
3. Alternatively, you can use ROS to publish IMU data and modify the code to subscribe to ROS topics

### Step 7: Build and Run

Once everything is configured:

```bash
cd /home/weida/slam_new/ORB_SLAM3
cd build
make mono_inertial_usb -j4

# Run the program
cd /home/weida/slam_new/ORB_SLAM3
./Examples/Monocular-Inertial/mono_inertial_usb \
    Vocabulary/ORBvoc.txt \
    Examples/Monocular-Inertial/USB_Webcam.yaml \
    /dev/video4 \
    /dev/ttyUSB0 \
    trajectory.txt  # Optional: save trajectory
```

## Important Notes

1. **IMU is Critical**: Mono-inertial SLAM requires accurate IMU measurements. Make sure your IMU is properly calibrated.

2. **Initialization**: The system needs sufficient motion for initialization. Move the camera with rotation and translation.

3. **Synchronization**: Camera and IMU timestamps must be synchronized. The current code uses system time, which may not be accurate enough.

4. **Performance**: Real-time SLAM requires sufficient CPU. The Logitech BRIO at 640x480@30fps should be manageable.

## Troubleshooting

### If IMU is not detected:
```bash
# Check if device exists
ls -la /dev/ttyUSB*

# Check permissions
groups  # Should include 'dialout'

# Test serial communication
sudo screen /dev/ttyUSB0 115200  # Ctrl+A then K to exit
```

### If webcam is not detected:
```bash
# List video devices
v4l2-ctl --list-devices

# Test with basic capture
ffplay /dev/video4
# or
vlc v4l2:///dev/video4
```

### If ORB-SLAM3 doesn't build:
- Make sure all dependencies are installed
- Check the Dependencies.md file
- Try running the full `./build.sh` script

## Alternative Approach: Use ROS

If you're having trouble with the C++ implementation, consider using ROS:

1. Install ROS (if not already)
2. Create ROS nodes for camera and IMU
3. Use the ORB-SLAM3 ROS wrapper
4. This provides better timestamp synchronization and easier device management

Would you like help setting up the ROS approach instead?
