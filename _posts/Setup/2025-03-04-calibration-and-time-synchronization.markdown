---
title: "Calibration and Time Synchronization"
layout: post
date: 2025-03-04 12:00
image: /assets/image/sia.png
headerImage: false
tag:
- markdown
- components
- extra
hidden: false
category: Setup
author: Yanpeng Jia
description: Calibration and Time Synchronization
---

# Calibration and Time Synchronization

Our dataset offers precise intrinsic and extrinsic calibration parameters, and the coordinate systems of the sensors in both robots as shown in Figure 4. This section presents the calibration methods and procedures. The parameters for all sequences are generated using the [development kits](https://github.com/Yaepiii/M2UD/Development_Kit) and stored in ```calibration.yaml```.

![figure](../../assets/image/figure4.png)

1. **Intrinsic calibration:** For the camera intrinsic parameters, we use the manufacturer-provided intrinsic parameters for the color and depth cameras. Then, we collect a set of calibration images with a self-designed calibration board for validating the accuracy of these parameters using the MATLAB Camera Toolbox. The IMU intrinsic parameters are obtained by recording a static IMU sequence for over six hours at 200 Hz. The intrinsic parameters, including noise density and random walk bias, are calibrated using the [imu_utils](https://github.com/gaowenliang/imu\_utils) toolbox. To support the validation of other calibration algorithms, the [calibration sequences](https://1drv.ms/f/c/c1806c2e19f2193f/Emnejgu3QXlIjV98CUbzDhoBd9cvpDwOOFe0OBqqwBHOGQ?e=USBC7q) are publicly available alongside the dataset.

2. **Extrinsic calibration:** For the calibration of extrinsic parameters, an initial estimation is first obtained from the SolidWorks model. The extrinsic parameters between the LiDAR and IMU are obtained using the method described in [this paper](https://ieeexplore.ieee.org/document/9982225); The Kalibr toolbox is used to calibrate the extrinsic parameters between the camera and IMU; The extrinsic parameters between the color and depth cameras are provided by the manufacturer; The transformation between the LiDAR and camera is determined using the MATLAB LiDAR-Camera Calibration Toolbox. Although the sparsity of the LiDAR point cloud poses a challenge for calibration, snapshots of the calibration process (Figure 5) demonstrate that the LiDAR point cloud is accurately projected onto the calibration plate, highlighting the precision of our extrinsic calibration. The originalal [calibration data](https://1drv.ms/f/c/c1806c2e19f2193f/Emnejgu3QXlIjV98CUbzDhoBd9cvpDwOOFe0OBqqwBHOGQ?e=USBC7q) is publicly available on the dataset website.

3. **Time synchronization:** Instead of using hardware signals to trigger all sensors simultaneously, we record data from different sensors with the same system timestamp. In other word, LiDAR, camera, and IMU are synchronized via software by calling the APIs to trigger data capture at the same instance. Test results indicate that this software-based synchronization method achieves a time synchronization accuracy of less than 10 ms.

![figure](../../assets/image/figure5.png)
