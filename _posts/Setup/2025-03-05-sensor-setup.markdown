---
title: "Sensor Setup"
layout: post
date: 2025-03-05 12:00
image: /assets/image/sia.png
headerImage: false
tag:
- markdown
- components
- extra
hidden: false
category: Setup
author: Yanpeng Jia
description: Sensor Setup
---

# Sensor Setup

We argue that while smooth motion, clear images, and dense point clouds obviously facilitate algorithm processing, they may not be sufficient for practical applications. Therefore, we select cost-effective and widely used sensors to replicate real-world robotic application conditions as closely as possible.

![figure](../../assets/image/figure2.png)

1. **LiDAR:** We utilize the [Velodyne VLP-16](https://ouster.com/products/hardware/vlp-16) LiDAR to capture 3D point clouds of the surrounding environment at 10 Hz, collecting over 300,000 points per second. The sensor features a 360 ° horizontal field of view (FoV) and a 30 ° vertical FoV, with a scanning range over 100 m. To ensure efficient data transmission, we connect the LiDAR to the host via an Ethernet port.

2. **Camera:** The Intel [RealSense D455](https://www.intelrealsense.com/depth-camera-d455/) is a global shutter RGB-D camera capable of capturing time-synchronized color and depth images at a resolution of 720×480 pixels at 10 Hz, whose optimal depth range extends from 0.6 m to 6 m from the image plane, with an error margin of less than 2% within a 4 m range.

3. **IMU:** To ensure higher accuracy in IMU data acquisition, we select not to use the internal IMU (Bosch BMI055) of the RealSense D455. Instead, we integrate the [Xsens MTi 300](https://www.xsens.com/hubfs/Downloads/Leaflets/MTi-300.pdf) to capture 9-axis measurement data at 200 Hz. The accelerometer exhibits a bias stability of 15 $\mu g$ and a noise density of 60 $\mu g/\sqrt{Hz}$, the gyroscope achieves a bias stability of 10 $°/s$ with a noise density of 0.003 $°/s$, and the magnetometer provides a resolution of 0.25 $mG$.

4. **GNSS:** The [M68UGI-G GNSS](https://www.devecent.com/M68.html) receiver is utilized to acquire raw positioning data, including GPS satellite count, timestamps, signal quality, and other relevant information at 5 Hz. By leveraging RTK technology, high-precision robot positions are estimated as ground truth for the trajectory, achieving a horizontal accuracy of $\pm(8mm+1ppm)$ and a vertical accuracy of $\pm(15mm+1ppm)$. According to the accuracy test report provided by the manufacturer (Figure 3), the repeatability positioning accuracy is $\pm1.5cm$.

![figure](../../assets/image/figure3.png)
