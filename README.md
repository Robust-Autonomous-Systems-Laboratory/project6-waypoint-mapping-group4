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

Our navigation strategy is to use the north wall and the recycle bin in the center of the room as landmarks. For points 1,5, and 6, the robot uses the wall as its landmark. For points 2,3, and 5, it uses the recycle bin. At every waypoint the robot's orientation is one of the four cardinal directions (N,W,S,E), with 0° corresponding to south. While only five points are required, a sixth point was added, which is identical to the first one, for loop closure purposes. A detailed discussing of the strategy can be found in the [`navigation_strategy.md`](./docs/navigation_strategy.md) document.


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

To use this package, first clone this repo into a ROS2 workspace:

```
$ mkdir -p ~/project_ws/src
$ cd ~/project_ws/src
$ git clone git@github.com:Robust-Autonomous-Systems-Laboratory/project6-waypoint-mapping-group4.git
```

Next, build and source the package:

```
$ cd ~/project_ws
$ colcon build
$ source install/setup.bash
```

#### <u>Important Note: </u>

If this system is running with live Turtlebot data, each terminal must set the ROS_DOMAIN_ID enviornment variable that corresponds to the Turtlebot's domain ID.  If not, remote Turtlebot topics will not be received by the nodes.

### How to launch your localization node

An EKF node from [project 4](https://github.com/Robust-Autonomous-Systems-Laboratory/proj4_group3) was imported to the  `scan_capture_pkg` and adjusted to publish a `PoseStamped` msg on the `/localization/pose` topic, in addition to the filter's Odom and Path messages.

To start the localization system, ensure the package is sourced and run the node with the following:

```
$ cd ~/project_ws
$ source install/setup.bash
$ ros2 run scan_capture_pkg ekf_node.py 
```

The node will remain idle until it receives Turtlebot topics `/imu`, `/joint_states`, and `/cmd_vel` from either the live robot or playback data from a bag file.

### How to run the scan capture system

To capture laser scan data for naive mapping applications, the CaptureScan service is leveraged to record information and convert LaserScan message data to a PointCloud2 message that forms the basis of the map.

Several terminals are required to run the scan capture system. All terminals require the workspace to be sourced (`source install/setup.bash`) and have a domain ID enviornment variable if using live Turtlebot data.

#### Terminal 1 - EKF node

The localization node descirbed in the previous subsection starts the extended kalman filter node for robot localization.  Ensure it is running and error-free before proceeding

#### Terminal 2 - Scan Capture Node

This is the main node and ROS2 service 'server' that advertises and processes CaptureScan service requests.

Navigate to the root of the git repositiory (~/project_ws/src/project6-waypoint-mapping-group4) and start the launch file. This is required to ensure the generated artifacts are saved in the proper [`data/`](./data/) directory.

```
$ cd ~/project_ws/src/project6-waypoint-mapping-group4
$ ros2 launch scan_capture_pkg scan_capture.launch.py
```

#### Terminal 3 - Keyboard Capture

While the robot is initialized and the previous nodes are running, the keyboard capture node will trigger the CaptureScan service and associate waypoints and export generated data.  Run via:

```
$ ros2 run scan_capture_pkg keyboard_capture.py
```
and enter the corresponding waypoint ID to save artifacts at each point.  Press `q` to quit the node.

### How to visualize the captured map
WIP
