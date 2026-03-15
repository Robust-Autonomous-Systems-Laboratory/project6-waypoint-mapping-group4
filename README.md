# Project 6: Naive Mapping by Waypoints

## Ian Q. Mattson & Malcolm Benedict
---

## Introduction

Mapping is a key part of mobile robotics, and this exercise served as an introduction to it with the TurtleBots. To begin, a map of the environment was sketched, waypoints were selected, marked, and their location was recorded. Then, a map of the same environment was generated using the LiDAR on a TurtleBot, by recording a point cloud at each of the designated waypoints.

## 1. Navigation Strategy Summary
<div style="text-align: center;">

#### Environment Sketch
![A drawing of EERC 722](figures/roomSketch.png "Sketch of the room with waypoints")
*Sketch of the floorplan of EERC 722. The waypoints are denoted by 'X', and a recycle bin has been added to the center of the room to serve as a second landmark.*
</div>

Our navigation strategy is to use the north wall and the recycle bin in the center of the room as landmarks. For points 1,5, and 6, the robot uses the wall as its landmark. For points 2,3, and 5, it uses the recycle bin. At every waypoint the robot's orientation is one of the four cardinal directions (N,W,S,E), with 0° corresponding to south. While only five points are required, a sixth point was added, which is identical to the first one, for loop closure purposes. A detailed discussing of the strategy can be found in the `navigation_strategy.md` document, located in the `docs/` folder


## 2. System Architecture
- Data flow diagram
- Your EKF/UKF configuration summary
- How you would incorporate Project 5 sensor characterization into the mapping pipeline

## 3. Map Accuracy Results
<div align="center">

#### Distance accuracy table 

| Waypoint | Landmark | Measured (m) | RViz (m) | Error (m) | Error (%) |
|----------|----------|--------------|----------|-----------|-----------|
| 1 | North wall | 1.37 | - | - | -% |
| 2 | Recycle bin | 1.598 | - | - | -% |
| 3 | Recycle bin | 1.4439 | - | - | -% |
| 4 | Recycle bin | 1.963 | - | - | -% |
| 5 | North wall | 1.373 | - | - | -% |
| 6 | North wall | 1.37 | - | - | -% |

</div>


- Orientation assessment for each waypoint
- RViz screenshots showing:
  - Individual scan captures at each waypoint
  - Measurement tool usage
  - Overall map with all scans visualized

## 4. Discussion
- Analysis of mapping accuracy
- Sources of error (localization, measurement, sensor)
- Map consistency assessment
- Recommendations for improvement

## 5. Usage Instructions
- How to launch your localization node
- How to run the scan capture system
- How to visualize the captured map

