---
title: "Data Format"
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
description: Data Format
---

# Data Format

The following table presents the information included in the ROS bags, along with their corresponding message types and frequencies. We aim to provide concise and comprehensive topics. For RTK data, the ROS bags record measurement positions, connection status, and GPS satellite conditions in the ```NavSatFix``` format. LiDAR data is provided in the widely used ```PointCloud2``` format to enhance usability. IMU data includes acceleration, angular velocity, and magnetometer readings. To preserve image quality, image data remains uncompressed, and the original image topic is provided as ```sensor_msgs/Image```. All ROS drivers for LiDAR, IMU, and cameras utilize manufacturer-released ROS drivers, while the RTK topic conversion script is open-sourced alongside our dataset.

| Type   | Topic Name         | Message Type                 | Rate (Hz) |
|--------|--------------------|------------------------------|----------:|
| RTK    | /RTK/data         | sensor_msgs/NavSatFix        | 5         |
| LiDAR  | /velodyne_points  | sensor_msgs/PointCloud2      | 10        |
| IMU    | /imu/data         | sensor_msgs/Imu              | 200       |
|        | /imu/mag          | sensor_msgs/MagneticField    | 100       |
| Camera | /camera/color     | sensor_msgs/Image            | 10        |
|        | /camera/depth     | sensor_msgs/Image            | 10        |

