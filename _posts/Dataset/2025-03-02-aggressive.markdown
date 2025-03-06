---
title: "Aggressive"
layout: post
date: 2025-03-02 12:00
image: /assets/image/sia.png
headerImage: false
tag:
- markdown
- components
- extra
hidden: false
category: Dataset
author: Yanpeng Jia
description: Aggressive
---

# Aggressive

We meticulously designed a series of routes incorporating aggressive motion, such as continuously walk up and down stairs, steep slopes with significant elevation changes, and rapid rotations(with a maximum rotational speed of 3.77 rad/s). Under these conditions, issues such as image blurring and LiDAR motion distortion frequently arise.

![figure](../../assets/image/aggressive.png)

| Sequence        | Duration (s) | Distance (m) | Difficulty | Description | Ground Truth |
|----------------|-------------|-------------|------------|-------------|-------------|
| aggressive_01  | 137.8       | 97.5        | ⭐         | Shenyang Library, continuous long stair ascent and descent, descending first then ascending. | ground truth |
| aggressive_02  | 157.4       | 174.3       | ⭐         | Ice rink with continuous undulating slopes, aggressive pitch movements, few dynamic people. | ground truth |
| aggressive_03  | 43.4        | 29.3        | ⭐⭐⭐       | Rapid in-place spinning + drifting, counterclockwise -> clockwise -> counterclockwise -> drifting. | ground truth |
| aggressive_04  | 493.0       | 508.7       | ⭐⭐⭐       | One loop along riverbank, park, and skate park, stair ascent and descent, aggressive roll/pitch movements, large elevation changes, uneven terrain, unstructured environment. | ground truth |
| aggressive_05  | 1189.6      | 1267.3      | ⭐⭐⭐⭐      | Ascending and descending pedestrian bridges, enters a Park, includes various scenes (rugged paths, lakeside, narrow bridges, slopes, steep stairs, downhill, structured square), contains several small loops. | ground truth |
| aggressive_06  | 27.8        | 3.5         | ⭐⭐        | Day, indoor spinning, counterclockwise -> clockwise -> counterclockwise (rotation speed 3.77 rad/s). | ground truth |
| aggressive_07  | 26.8        | 3.8         | ⭐⭐⭐       | Night, indoor spinning, counterclockwise -> clockwise -> counterclockwise (rotation speed 3.77 rad/s). | ground truth |




