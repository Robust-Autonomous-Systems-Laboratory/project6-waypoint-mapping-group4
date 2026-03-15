#!/usr/bin/env python3
"""
Scan Capture Service Node

This node provides a service to capture laser scans at waypoints.
It subscribes to /scan and /localization/pose, and when triggered,
saves the current scan as a PointCloud2 along with the pose estimate.

Author: Team 4 - Malcolm Benedict + Ian Mattson
Course: EE5531 Introduction to Robotics
Project: 6 - Waypoint Mapping
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from datetime import datetime


from capture_service.srv import CaptureScan

import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import os
import yaml
import math


class ScanCaptureNode(Node):
    """
    ROS2 node providing scan capture service for waypoint mapping.

    TODO: Implement this node to:
    - Subscribe to /scan (LaserScan) and /localization/pose (PoseStamped)
    - Provide a /scan_capture/capture service (CaptureScan)
    - When the service is called, convert the latest scan to PointCloud2,
      publish it, and save the scan data and pose to files in output_dir
    """

    def __init__(self):
        super().__init__('scan_capture_node')

        # =====================================================================
        # Parameters
        self.declare_parameter('output_dir', 'data/captures')
        self.declare_parameter('pose_topic', '/localization/pose')
        self.declare_parameter('scan_topic', '/scan')

        self.output_dir = self.get_parameter('output_dir').get_parameter_value().string_value
        self.pose_topic = self.get_parameter('pose_topic').get_parameter_value().string_value
        self.scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value

        # =====================================================================
        # State variables
        self.pose = PoseStamped()
        self.scan = LaserScan()
        self.odom_pose = PoseStamped() # backup if no pose
        self.counter = 0 # number of captures taken

        # =====================================================================
        # Subscribers
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, # Key setting for best effort
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # laserscan topic /scan
        self.scan_subcription = self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, qos_profile=qos_profile)

        # localization node (ekf) pose topic - given as odom but converted similar to /odom
        self.pose_subscription = self.create_subscription(PoseStamped, self.pose_topic, self.ekf_pose_callback, qos_profile=qos_profile)

        # default fallback pose topic from TB3 /odom
        self.odom_subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, qos_profile=qos_profile)

        # =====================================================================
        # Publishers
        self.pc2_publisher_ = self.create_publisher(PointCloud2, '/scan_capture/pointcloud', 10)

        # =====================================================================
        # Service
        self.srv = self.create_service(CaptureScan, 'capture_scan', self.capture_callback)
        self.get_logger().info('Scan Capture Node started')


    # =====================================================================
    # Callbacks and Functions

    def scan_callback(self, msg: LaserScan):
        """Store the latest laser scan."""
        self.scan = msg

    # ekf updated to publish PoseStamped
    def ekf_pose_callback(self, msg: PoseStamped):
        """Store the latest pose estimate from the localization node."""
        self.pose.header.frame_id = msg.header.frame_id
        self.pose.header.stamp = msg.header.stamp
        self.pose.pose.position = msg.pose.position
        self.pose.pose.orientation = msg.pose.orientation

    def odom_callback(self, msg: Odometry):
        """Fallback: use odometry pose if no localization pose is available."""
        self.odom_pose.header.frame_id = msg.header.frame_id
        self.odom_pose.header.stamp = msg.header.stamp
        self.odom_pose.pose.position = msg.pose.pose.position
        self.odom_pose.pose.orientation = msg.pose.pose.orientation

    def laserscan_to_pointcloud2(self, scan: LaserScan) -> PointCloud2:
        """
        Convert input LaserScan() msg to PointCloud2() msg
        Filter out invalid ranges, convert to XYZ, populate PointCloud2 msg
        """
        range_data = scan.ranges # meters
        points = [] # list of [x,y,z] points

        for i, range in enumerate(range_data):

            # reject data outside of valid ranges
            if range > scan.range_max or range < scan.range_min:
                break

            # calculate x,y coordinates of all valid data
            angle = scan.angle_min + i * scan.angle_increment # Radians
            x = range * math.cos(angle)
            y = range * math.sin(angle)
            points.append([x,y,0]) # 2D measurement, no z data (set to zero)

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = scan.header.frame_id

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        return pc2.create_cloud(header, fields, points)



    def save_capture(self, waypoint_id: int, description: str, scan: LaserScan, pose: PoseStamped) -> str:
        
        #1. Generate a timestamped filename using waypoint_id        
        filename = f"waypoint{waypoint_id}_{datetime.now().strftime('%Y_%m_%d-%H_%M_%S')}"
        yaml_path = os.path.join(self.output_dir, filename + ".yaml")
        npy_path = os.path.join(self.output_dir, filename + ".npy")

        # 2. Save pose data (x, y, yaw) and scan metadata to a YAML file
        yaw = math.atan2(2*(pose.pose.orientation.w * pose.pose.orientation.z + pose.pose.orientation.x * pose.pose.orientation.y), 1-2*(pose.pose.orientation.y**2 + pose.pose.orientation.z**2))

        yaml_data = {
            'waypoint_id': waypoint_id,
            'timestamp': datetime.now().strftime('%Y_%m_%d-%H_%M_%S'),
            'pose': {
                'x': pose.pose.position.x,
                'y': pose.pose.position.y,
                'yaw': yaw
            },
            'description': description,
            'scan_metadata': {
                'frame': scan.header.frame_id,
                'angle_min': scan.angle_min,
                'angle_max': scan.angle_max,
                'angle_increment': scan.angle_increment,
                'time_increment': scan.time_increment,
                'scan_time': scan.scan_time,
                'range_min': scan.range_min,
                'range_max': scan.range_max
            }
        }

        with open(yaml_path, 'w+') as f:
            yaml.dump(yaml_data, f, sort_keys=False)

        # 3. format range data as npy and write to output
        np.save(npy_path, np.asarray(scan.ranges))

        # 4.return the path to the saved yaml file
        return yaml_path



    def capture_callback(self, request, response):
        """
        Service callback: capture the current scan and pose.
        """

        if self.scan.header.frame_id and self.pose.header.frame_id:
            # convert scan to pc2 and publish pc2
            pc2_msg = self.laserscan_to_pointcloud2(self.scan)
            self.pc2_publisher_.publish(pc2_msg)

            # populate the response msg fields and save the data
            response.pose = self.pose
            response.filename = self.save_capture(request.waypoint_id, request.description, self.scan, self.pose)
            response.success = True
            response.message = 'Successful using pose'
            return response
        
        elif self.scan.header.frame_id and self.odom_pose.header.frame_id:
            pc2_msg = self.laserscan_to_pointcloud2(self.scan)
            self.pc2_publisher_.publish(pc2_msg)

            response.pose = self.odom_pose
            response.filename = self.save_capture(request.waypoint_id, request.description, self.scan, self.odom_pose)
            response.success = True
            response.message = 'Successful using odom as backup, no pose data'
            return response
        
        elif not self.scan.header.frame_id:
            response.pose = ''
            response.success = False
            response.message = 'Failed, no scan data'
            response.filename = ''
            return response
        
        else:
            response.pose = ''
            response.success = False
            response.message = "Failed, no pose data"
            response.filename = ''
            return response


def main(args=None):
    rclpy.init(args=args)
    node = ScanCaptureNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
