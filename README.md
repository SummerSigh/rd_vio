# rd_vio

This repo is the plain CMake version for rd-vio, separated and modified from [xrslam](https://github.com/openxrlab/xrslam). Check the original repo for detail.

*RD-VIO: Robust Visual-Inertial Odometry for Mobile Augmented Reality in Dynamic Environments*

And now it is:
- mono visual-inertial odometry

What is removed:
- global localizer
- global interface

What is add:
- a simple viewer using [Pangolin](https://github.com/stevenlovegrove/Pangolin)
- a simple dataset reader (currently euroc only)

#### notes
The main system APIs can be founded in 'src/rdvio/rdvio.hpp'.

EuRoC example can be found in 'examples', it works well on seq `v101-easy`.

Multi-threading is enabled by default, you can disable it by adding parameter when cmake:
```sh
cmake .. -DTHREADING=OFF
```


\*About testing on [ADVIO](https://github.com/AaltoVision/ADVIO)
- Basic code can be found in 'examples'
- Still wrong estimation, working on it


#### install
Make sure you have installed:
- OpenCV 4.x
- Eigen 3.4.0
- Ceres-Solver
- yaml-cpp
- pangolin

For the RealSense example, you also need:
- librealsense2 (for Intel RealSense cameras)

To install librealsense2:
```sh
# For Ubuntu/Debian systems
sudo apt-get install librealsense2-dev librealsense2-dkms
```
See the [librealsense installation guide](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md) for more detailed instructions.

Then run the following commands:
```sh
# clone the repo
cd rd_vio

mkdir build
cd build

cmake ..
make -j4
```

#### test on EuRoC
```sh
cd build
./examples/test_euroc ../configs/euroc_sensor.yaml ../configs/setting.yaml $/path/to/euroc/mav0
```

#### test with Intel RealSense D455
First, generate an auto-calibrated configuration using the RealSense's built-in calibration data:
```sh
cd build
./examples/realsense_d455 my_d455_calib.yaml  # Creates calibration file from device
```

Then run the VIO with the generated calibration file:
```sh
./examples/test_realsense my_d455_calib.yaml ../configs/setting.yaml
```

Notes:
- The D455 camera must be connected to your computer
- This example uses the infrared camera stream instead of RGB for better performance in SLAM
- IMU data (gyroscope and accelerometer) is fused with visual tracking for robust odometry
- The calibration tool automatically extracts intrinsics and extrinsics from the device
- If calibration looks incorrect, try placing the camera on a stable surface and rerunning the calibration

#### preview
![preview](preview.png)  
*\*test on euroc v101 easy*
