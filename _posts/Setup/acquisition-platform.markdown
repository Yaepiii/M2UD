---
title: "Acquisition Platform"
layout: post
date: 2025-03-06 12:00
image: /assets/images/sia.jpg
headerImage: true
tag:
- markdown
- components
- extra
hidden: false
category: Setup
author: Yanpengjia
description: Acquisition Platform
---

# Acquisition Platform

As shown in Figure 2, two self-developed ground robots are utilized for data collection, both equipped with identical sensors to ensure data consistency. The six-wheeled special robot in Figure 2 (A) is designed for outdoor data collection, whose unique rocker-arm design provides excellent adaptability to normal road surfaces, uneven terrains, and stairs, making it particularly suitable for challenging outdoor environments. As shown in Figure 2 (B), the four-wheeled mobile robot is compact, highly maneuverable, and capable of fast movement, which can navigate freely through various indoor environments and is primarily used for data collection in locations such as plazas, parking, and long corridors. The Robot Operating System (ROS) is utilized to store data in bag format on a 4TB solid-state drive in the on-board computer, facilitating its use for SLAM algorithms.

![figure](/assets/image/figure2.png)
