---
title: "Dataset Overview"
layout: post
date: 2025-03-07 12:00
image: /assets/images/sia.jpg
headerImage: false
tag:
- markdown
- components
- extra
hidden: false
category: overview
author: Yanpeng Jia
description: Dataset Overview
---

# Dataset Overview

<iframe width="780" height="480" src="https://www.youtube.com/embed/Vcbka12Dah4?si=wCrhxKtRH1iDRXcK" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

To advance the application of SLAM in ground robots, we present a multi-modal, multi-scenario, uneven-terrain SLAM dataset, collected using two special ground robot platforms equipped with well-calibrated multi-channel LiDAR, RGB-D cameras, high-frequency IMUs, and GNSS, to be designed for maximum replicating the motion patterns of ground robots. Additionally, to reproduce various common application scenarios of ground robots, we collect a series of scenarios around the Shenyang Institute of Automation, Chinese Academy of Sciences (Figure 1 (A)) and in Xinjiang, China (Figure 1 (B), (C)). The dataset includes 58 sequences across 12 categories, collected from a diverse set of highly challenging environments, including urban areas, rural regions, open fields, long corridors, plazas, underground parking, and mixed scenarios, covering a total distance of over 50 km. Moreover, we collect data under extreme weather conditions, including darkness, smoke, snow, sand, and dust, to support researchers in developing SLAM solutions for such extreme situations. The data collection trajectory and scenario snapshots of this dataset are illustrated in Figure 1.

![figure](./assets/image/figure1.png)

To benchmark SLAM algorithms, the dataset provides smoothed location ground truth obtained via Real-time Kinematics (RTK) and ground truth maps of two representative scenes acquired using a high-precision millimeter-level laser scanner, facilitating the development and evaluation of localization and mapping algorithms. Additionally, current evaluations of SLAM algorithm localization performance primarily focus on accuracy comparisons, while in practical applications, algorithm efficiency is also a crucial consideration. Therefore, we propose a localization evaluation metric that considers both accuracy and efficiency, aiding developers in selecting algorithms better suited for real-world engineering applications. To enhance usability, the dataset is accompanied by a suite of development kits, including data transformation, timestamp alignment, ground truth smooth, etc.









































