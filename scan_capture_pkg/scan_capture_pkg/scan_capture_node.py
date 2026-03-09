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

from scan_capture_pkg.srv import CaptureScan

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
        # TODO: Declare and read parameters:
        #   - 'output_dir': directory to save captures (default: 'data/captures')
        #   - 'pose_topic': topic for pose estimates (default: '/localization/pose')
        #   - 'scan_topic': topic for laser scans (default: '/scan')
        # =====================================================================

        # =====================================================================
        # State variables
        # TODO: Initialize variables to hold the latest scan and pose messages,
        #       and a counter for the number of captures taken
        # =====================================================================
        self.outliers = 0

        # =====================================================================
        # Subscribers
        # TODO: Subscribe to the scan topic (use BEST_EFFORT QoS) and pose topic.
        #       Also subscribe to /odom as a fallback pose source.
        # =====================================================================

        # =====================================================================
        # Publishers
        # TODO: Publish captured scans as PointCloud2 on /scan_capture/pointcloud
        # =====================================================================

        # =====================================================================
        # Service
        # TODO: Create a service server for /scan_capture/capture using the
        #       CaptureScan service type
        # =====================================================================

        self.get_logger().info('Scan Capture Node started (stub - not yet implemented)')

    def scan_callback(self, msg: LaserScan):
        """Store the latest laser scan."""
        # TODO: Save the incoming scan message to a member variable
        pass

    def pose_callback(self, msg: PoseStamped):
        """Store the latest pose estimate."""
        # TODO: Save the incoming pose message to a member variable
        pass

    def odom_callback(self, msg: Odometry):
        """Fallback: use odometry pose if no localization pose is available."""
        # TODO: If no pose has been received yet, convert the Odometry message
        #       to a PoseStamped and store it
        pass

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
                self.outliers = self.outliers + 1
                break

            # calculate x,y coordinates of all valid data
            angle = scan.angle_min + i * scan.angle_increment # Radians
            x = range * math.cos(angle)
            y = range * math.sin(angle)
            points.append([x,y,0]) # 2D measurement, no z data (set to zero)

        header = Header()
        header.stamp = self.get_clock().now()
        header.frame_id = scan.header.frame_id

        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
        ]

        return pc2.create_cloud(header, fields, points)



    def save_capture(self, waypoint_id: int, description: str,
                     scan: LaserScan, pose: PoseStamped) -> str:
        """
        Save captured scan and pose to files.

        TODO: Implement file saving:
        1. Generate a timestamped filename using waypoint_id
        2. Save pose data (x, y, yaw) and scan metadata to a YAML file
        3. Save raw range data to a .npy file alongside the YAML
        4. Return the path to the saved YAML file
        """
        raise NotImplementedError('save_capture not yet implemented')



    def capture_callback(self, request, response):
        """
        Service callback: capture the current scan and pose.

        TODO: Implement the service handler:
        1. Check that latest scan and pose data are available; return a
           failure response with an informative message if either is missing
        2. Convert the scan to PointCloud2 and publish it
        3. Save the scan and pose using save_capture()
        4. Populate and return the response (success, message, filename, pose)
        """
        response.success = False
        response.message = 'Not yet implemented'
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
