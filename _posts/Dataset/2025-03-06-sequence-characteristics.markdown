---
title: "Sequence Characteristics"
layout: post
date: 2025-03-06 12:00
image: /assets/image/sia.png
headerImage: false
tag:
- markdown
- components
- extra
hidden: false
category: Dataset
author: Yanpeng Jia
description: Sequence Characteristics
---

# Sequence Characteristics

![figure](../../assets/image/figure5.png)

Our dataset comprises 58 sequences with specifically designed routes, categorized into 12 types based on collection scenarios or characteristics, with difficulty levels assigned for user reference. Figure 5 presents a first-person view of the robot during data recording, illustrating that the sequences contain common application scenarios of ground robots as well as challenging conditions, including complete darkness (Figure 5 (A)), textureless (Figure 5 (B)), lift (Figure 5 (C)), overexposure (Figure 5 (D), (E)), reflections (Figure 5 (F)–(H)), extreme weather conditions (Figure 5 (I)–(K)), aggressive motion (Figure 5 (L)–(O)), dynamic objects (Figure 5 (P)–(T)), and open areas or highly-repetitive structures (Figure 5 (U)–(Y)). Our dataset offers trajectories and two maps ground truth, which can be directly utilized for evaluating localization and mapping in SLAM. The details of the 12 categories are presented in the following table.

| Scene      | Number | Size (GB) | Total Time (s) | Total Distance (m) | Average Difficulty       | Description |
|------------|--------|-----------|----------------|---------------------|--------------------------|-------------|
| [open](https://yaepiii.github.io/M2UD//open/)       | 8      | 60        | 5055.7         | 5596.7              | ⭐⭐                   | Lakeside and open field, sparse features, no dynamics, uneven-terrain. |
| [rural](https://yaepiii.github.io/M2UD//rural/)      | 2      | 6.32      | 3111.9         | 3835.5              | ⭐⭐⭐                   | Unstable features, sparsely populated, flat-terrain. |
| [urban](https://yaepiii.github.io/M2UD//urban/)      | 5      | 100       | 6577.5         | 7770.0              | ⭐⭐⭐⭐                   | High dynamics, various types of vehicles, flat-terrain. |
| [aggressive](https://yaepiii.github.io/M2UD//aggressive/) | 7      | 38.8      | 2075.8         | 2080.9              | ⭐⭐⭐                   | Continuous aggressive motion, uneven-terrain. |
| [campus](https://yaepiii.github.io/M2UD//campus/)     | 6      | 51.3      | 2728.2         | 3152.3              | ⭐⭐                   | Research institute campus, structured environment, few dynamic objects, flat-terrain. |
| [park](https://yaepiii.github.io/M2UD//park/)       | 6      | 61.7      | 2904.6         | 3205.2              | ⭐⭐                   | Unstructured environment, few dynamic objects, uneven-terrain. |
| [extreme](https://yaepiii.github.io/M2UD//extreme/)    | 4      | 0.97      | 1669.5         | 1976.8              | ⭐⭐⭐                   | Severe dust, snow, few dynamics, flat-terrain. |
| [mixed](https://yaepiii.github.io/M2UD//mixed/)      | 5      | 116       | 9659.8         | 12161.7             | ⭐⭐⭐⭐⭐                   | Mixed scenarios such as indoor+outdoor, urban+park, parking+plaza, featuring characteristics of the mixed categories. |
| [smoke](https://yaepiii.github.io/M2UD//smoke/)      | 4      | 16.8      | 1155.5         | 1164.9              | ⭐⭐⭐                   | Navigating through smoke, visual occlusion, open scenario with sparse features, no dynamics, uneven-terrain. |
| [parking](https://yaepiii.github.io/M2UD//parking/)    | 3      | 16.8      | 1919.9         | 2810.6              | ⭐                  | Indoor, highly structured environment, rich features, few dynamics, flat-terrain. |
| [plaza](https://yaepiii.github.io/M2UD//plaza/)      | 4      | 41.5      | 4329.1         | 6566.8              | ⭐⭐⭐                   | Indoor, multi-layer, lifts, high-dynamic crowds, flat-terrain. |
| [corridor](https://yaepiii.github.io/M2UD//corridor/)   | 4      | 5.41      | 605.1          | 686.9               | ⭐⭐⭐                   | Indoor, highly repetitive geometric structures, no dynamics, flat-terrain. |
| **total**  | **58** | **515.6** | **41792.6**    | **51008.3**         | **⭐⭐⭐**               | **A total of 58 sequences covering common ground robot application scenarios and challenging elements.** |
| [mapping](https://yaepiii.github.io/M2UD//mapping/)   | 4      | 14.13      | 1,710.41          | 1,869.28               | ⭐⭐⭐                   | Two scenes specifically designed for mapping evaluation. |

