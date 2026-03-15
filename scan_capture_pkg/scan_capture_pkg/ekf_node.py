#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from geometry_msgs.msg import TwistStamped, PoseStamped
from builtin_interfaces.msg import Time
from nav_msgs.msg import Path, Odometry
from scan_capture_pkg.extended_kalman import ekf


class KalmanFilters(Node):

    def __init__(self):
        super().__init__('kalman_filters')
        self.get_logger().info('KF node started, awaiting bag playback ...')

        # create subscribers
        self.imu_sub = self.create_subscription(Imu,'imu', self.imu_cb, 20)
        self.joint_state_sub = self.create_subscription(JointState, 'joint_states', self.joint_state_cb, 20)
        self.twist_sub = self.create_subscription(TwistStamped, 'cmd_vel', self.twist_cb, 10)

        # init to avoid unused warnings
        self.imu_sub
        self. joint_state_sub
        self.twist_sub

        # create publishers
        self.ekf_path_pub = self.create_publisher(Path, 'localization_node/ekf/path', 20)
        self.ekf_odom_pub = self.create_publisher(Odometry, 'localization_node/ekf/odometry', 20)
        self.ekf_pose_pub = self.create_publisher(PoseStamped, 'localization/pose', 20)

        # init variables
        self.twist_linear_x = 0     # /cmd_vel linear x velocity command [m/s]
        self.twist_angular_z = 0    # /cmd_vel angular z velocity command [m/s]
        self.imu_linear_acc = 0     # /imu linear acceleration [m/s^2]
        self.imu_angular_vel = 0    # /imu angular velocity [m/s]
        self.L = 0.166      # Turtlebot3 wheelbase [m]
        self.left_pos = 0      
        self.right_pos = 0
        self.prev_left_pos = 0      
        self.prev_right_pos = 0       
        self.wheel_radius = 0.033   # Turtlebot3 BurgerBot Wheel radius [m] 
        self.ekf_path = Path()

        self.Q = np.diag([0.01, 0.01, 0.01, 0.1, 0.1])
        self.R = np.diag([0.2, 0.2, 0.3, 0.3])
        
        self.ekf_prev_t = Time()
        self.ekf_prev_t.sec = 0.0
        self.ekf_prev_t.nanosec = 0.0
        self.ekf_x = np.zeros([5, 1])
        self.ekf_P = np.eye(5) * 0.1


    def imu_cb(self, imu_msg):
        # non-transformed IMU equations, transform to odom frame in the observation eqs.
        self.imu_linear_acc = imu_msg.linear_acceleration.x
        self.imu_angular_vel = imu_msg.angular_velocity.z

        # call to update the kfs, fastest msg callback, 20 Hz
        self.update_ekf()

    def joint_state_cb(self, joint_state_msg):
        # update the left, right wheel positions at 20 Hz

        self.prev_left_pos = self.left_pos
        self.prev_right_pos = self.right_pos

        self.left_pos = joint_state_msg.position[0]
        self.right_pos = joint_state_msg.position[1]
    

    def twist_cb(self, twist_msg):
        # update the cmd_vel values at 10 Hz
        self.twist_linear_x = twist_msg.twist.linear.x
        self.twist_angular_z = twist_msg.twist.angular.z


    def update_ekf(self):
        
        # iterate each kalman filter w/ new topic information
        x_pos, y_pos, theta, s, residual, P = ekf(self, self.twist_linear_x, self.twist_angular_z, self.left_pos, self.right_pos, self.prev_left_pos, self.prev_right_pos, self.imu_linear_acc, self.imu_angular_vel)

        qz = np.sin(theta/2)
        qw = np.cos(theta/2)

        pose_temp = PoseStamped()
        pose_temp.header.frame_id = 'odom'
        pose_temp.pose.position.x = x_pos
        pose_temp.pose.position.y = y_pos
        pose_temp.pose.orientation.z = qz
        pose_temp.pose.orientation.w = qw
        pose_temp.header.stamp = self.get_clock().now().to_msg()

        self.ekf_pose_pub.publish(pose_temp)

        self.ekf_path.header.frame_id = 'odom'
        self.ekf_path.header.stamp = self.get_clock().now().to_msg()
        self.ekf_path.poses.append(pose_temp)
        self.ekf_path_pub.publish(self.ekf_path)


        self.ekf_odom = Odometry()
        self.ekf_odom.header.frame_id = 'odom'
        self.ekf_odom.child_frame_id = 'odom'
        self.ekf_odom.pose.pose.position.x = x_pos
        self.ekf_odom.pose.pose.position.y = y_pos
        self.ekf_odom.pose.pose.orientation.z = qz
        self.ekf_odom.pose.pose.orientation.w = qw
        self.ekf_odom.header.stamp = self.get_clock().now().to_msg()
        self.ekf_odom_pub.publish(self.ekf_odom)

    

def main(args=None):
    rclpy.init(args=args)
    kalman_filters = KalmanFilters()
    rclpy.spin(kalman_filters)

    kalman_filters.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()