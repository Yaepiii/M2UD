---
title: "Development Kit"
layout: post
date: 2025-03-05 12:00
image: /assets/image/sia.png
headerImage: false
tag:
- markdown
- components
- extra
hidden: false
category: Dataset
author: Yanpeng Jia
description: Development Kit
---

# Development Kit

To enhance usability and facilitate evaluation, a series of development kits are also released on the M2UD dataset website, which are implemented in C`````` and built based on ROS. A brief overview of their main functions is provided as follows:

1. **Ground Truth Smooth:** The *groundtruth_create.cpp* file reads the RTK topic from the bag file and smooths the data. Eventually, it creates a ```GTCreate``` folder in the same directory and generates four files: timestamps, raw RTK data, smoothed ground truth values at the RTK frequency, and ground truth values at the LiDAR frequency.

2. **File Format Conversion:** The *bag2file.cpp* file reads topics from the bags and converts color images, depth images, and point clouds into 8-bit ```.png``` files and ```.bin``` binary files, respectively, naming them based on timestamps. It stores accelerometer, gyroscope, and magnetometer data from the IMU in a ```.csv``` file, the RTK data and smoothed trajectory ground truth in ```.txt``` files, and the sensor extrinsic and intrinsic parameters of the recording platform in a ```calibration.yaml``` file. The resulting file structure is illustrated in Figure 10.

![figure](../../assets/image/figure10.png)

3. **Time Alignment:** The *time_align.cpp* file is primarily used to evaluate localization accuracy. Since some algorithms that adopt the keyframe strategy and incorporate a loop closure detection module (e.g., LIO-SAM) may produce sparse trajectories for evaluation, the resulting trajectory may appear discontinuous when compared with the ground truth. This tool interpolates the algorithm's output based on the timestamps of the ground truth, enabling a more reasonable and continuous evaluation of localization accuracy.

4. **RTK Topic Publisher:** The *RTK_process.cpp* file reads GGA or VTG protocol data from the serial port, then repackages and publishes the original position information, the number of GPS satellites, timestamps, signal quality, and other relevant data as RTK ROS topics.

5. **Localization Evaluation:** To facilitate the evaluation of localization accuracy using the novel metric-"[EA-Drift](https://yaepiii.github.io/M2UD//ea-drift/)", we modify the widely used trajectory evaluation tool [evo](https://github.com/MichaelGrupp/evo) in SLAM and introduce EA-evo. Users first need to export a frame-by-frame time consumption file in the format ```<timestamp time-consumption>```. Following the original style of evo, users only need to add the ```-ea time_consumption.txt``` parameter to apply efficiency-aware Drift rate and ATE for trajectory evaluation. Additionally, the ```-sf``` parameter can be used to specify the sensor frequency.
