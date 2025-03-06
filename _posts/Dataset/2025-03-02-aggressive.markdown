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
| [aggressive_01](https://1drv.ms/u/c/c1806c2e19f2193f/EXVm3Ek6fQVBhK8rg-npqOMByIcFjQfDaEpinUMWZbNCwA?e=mWDoxf)  | 137.8       | 97.5        | ⭐         | Shenyang Library, continuous long stair ascent and descent, descending first then ascending. | [ground truth](https://1drv.ms/t/c/c1806c2e19f2193f/ERpBw8w8oh5IpFQOF9iscrwBkqD03E0vT6Mf97vls6ePEg?e=eswiQj) |
| [aggressive_02](https://1drv.ms/u/c/c1806c2e19f2193f/ETBaPLPCE_JLnvbko5YbfyoBOUhydpzkDbTS8JtwZf2znA?e=2KnvIG)  | 157.4       | 174.3       | ⭐         | Ice rink with continuous undulating slopes, aggressive pitch movements, few dynamic people. | [ground truth](https://1drv.ms/t/c/c1806c2e19f2193f/EUJiXQE2vJZNnOhBvjXzaPoBFfUY2J45nxWzLKSzbexHCA?e=88ba7X) |
| [aggressive_03](https://1drv.ms/u/c/c1806c2e19f2193f/EVd5a0wiWpRDr2FK9fK70EMB6i5BpAjpysfiYPkoZn3Vlg?e=25oaPn)  | 43.4        | 29.3        | ⭐⭐⭐       | Rapid in-place spinning + drifting, counterclockwise -> clockwise -> counterclockwise -> drifting. | [ground truth](https://1drv.ms/t/c/c1806c2e19f2193f/ESIxTR2wDblOhvgV8706II0Bv6iU8XJso8wRlg4z9yGt6g?e=zcubGc) |
| [aggressive_04](https://1drv.ms/u/c/c1806c2e19f2193f/EddkWLWHD1pFuFuQbik-iPMBOwLeQuMrJzUvPkvIcRBMEQ?e=MYXBAO)  | 493.0       | 508.7       | ⭐⭐⭐       | One loop along riverbank, park, and skate park, stair ascent and descent, aggressive roll/pitch movements, large elevation changes, uneven terrain, unstructured environment. | [ground truth](https://1drv.ms/t/c/c1806c2e19f2193f/EWL0uG83OllAhn_OZM65L9cBNuArQ7OlI-nGiy3_ksRN4Q?e=Lj6Ehz) |
| [aggressive_05](https://1drv.ms/u/c/c1806c2e19f2193f/EUwg08rV1hBLk6AcL8-cU48BddJXxES7csqBW9se7bEmlQ?e=ebTAbF)  | 1189.6      | 1267.3      | ⭐⭐⭐⭐      | Ascending and descending pedestrian bridges, enters a Park, includes various scenes (rugged paths, lakeside, narrow bridges, slopes, steep stairs, downhill, structured square), contains several small loops. | [ground truth](https://1drv.ms/t/c/c1806c2e19f2193f/EdA6TwQE25BFjhAvmyyxxQMBui8J3-it4vBCzm1Q-bzKDA?e=RMYSkX) |
| [aggressive_06](https://1drv.ms/u/c/c1806c2e19f2193f/EXnrkkFqjqFClBkJdpOm5w4BoEPxcfWBzjOqXfVWeg98ZQ?e=a3Nqne)  | 27.8        | 3.5         | ⭐⭐        | Day, indoor spinning, counterclockwise -> clockwise -> counterclockwise (rotation speed 3.77 rad/s). | [ground truth](https://1drv.ms/t/c/c1806c2e19f2193f/ETItEHo4Zh1BufC40eVTjO4B_NCcg82SA2-v3bFK2oGpzA?e=kV9J1L) |
| [aggressive_07](https://1drv.ms/u/c/c1806c2e19f2193f/ETPxru0UKpRMnZrwp9gpHqoBfsYDWWZfchahpi8CfrRk_A?e=iTeSUB)  | 26.8        | 3.8         | ⭐⭐⭐       | Night, indoor spinning, counterclockwise -> clockwise -> counterclockwise (rotation speed 3.77 rad/s). | [ground truth](https://1drv.ms/t/c/c1806c2e19f2193f/Efgl-9jVK2dEpZkE9IbJ1ecBS5es9FV8_uBKjt36f9K4WA?e=2fXZXW) |




