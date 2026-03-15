 # Testing the Service:

## Terminal 1
```
$ source ~/ee5531/scripts/turtlebot_connect.sh
$ cd ~/catkin_ws/src/project6-waypoint-mapping-group4
$ ros2 launch scan_capture_pkg scan_capture.launch.py
```


 ## Terminal 2: 
 ```
 $ source ~/ee5531/scripts/turtlebot_connect.sh
 $ ros2 service call /capture_scan capture_service/srv/CaptureScan "waypoint_id: 1
description: 'test'"
```

## Terminal 3:
```
$ source ~/ee5531/scripts/turtlebot_connect.sh
$ ros2 run scan_capture_pkg ekf_node.py 
```

## Terminal 4:
Start the turtlebot node via SSH
