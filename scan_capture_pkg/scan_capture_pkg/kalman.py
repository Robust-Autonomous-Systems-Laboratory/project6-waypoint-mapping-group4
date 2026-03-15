import numpy as np

# function takes all ROS topic inputs and updates KF prediction
# v = twist.linear.x
# w = twist.angular.z
# l_pos/r_pos = self.joint_state.position[x]
# imu_a = imu.linear_acceleration.x
# imu_omega = imu.angular_velocity.z

def kf(self, v, w, l_pos, r_pos, prev_l_pos, prev_r_pos, imu_a, imu_omega):
    
    if self.kf_prev_t.sec == 0 and self.kf_prev_t.nanosec == 0:
        delta_t = 1e-9
    else:
        prev_t = self.kf_prev_t.sec + (self.kf_prev_t.nanosec * 1e-9)
        curr_t_msg = self.get_clock().now().to_msg()
        curr_t = curr_t_msg.sec + (curr_t_msg.nanosec * 1e-9)
        delta_t = curr_t - prev_t

    self.kf_prev_t = self.get_clock().now().to_msg()

    # Propagate

    self.kf_theta = self.kf_theta + w * delta_t

    u = np.array([
        [v * np.cos(self.kf_theta)],
        [v * np.sin(self.kf_theta)],
        [self.kf_theta],
        [v],
        [w]
    ])

    B = np.diag([delta_t, delta_t, 1, 1, 1])

    self.kf_x = self.kf_x + (B @ u)

    if not prev_l_pos:
        v_left = (l_pos / delta_t) * 0.033  #0.033 [m] is the wheel radius on the turtlebot
        v_right = (r_pos / delta_t) * 0.033
    else:
        v_left = ((l_pos - prev_l_pos) / delta_t) * 0.033
        v_right = ((r_pos - prev_r_pos) / delta_t) * 0.033


    z = np.array([
        [v_left],
        [v_right],
        [imu_a],
        [imu_omega]
    ])

    self.kf_P = self.kf_P + self.Q

    # Update

    H = np.array([
        [0, 0, 0, 1, -self.L/2],
        [0, 0, 0, 1, self.L/2],
        [0, 0, 0, 1/delta_t, 0],
        [0, 0, 0, 0, 1]
    ])

    S = H @ self.kf_P @ H.T + self.R
    K = self.kf_P @ H.T @ np.linalg.inv(S)

    variance = S[0,0]
    residual = (z - H @ self.kf_x)

    self.kf_x = self.kf_x + K @ (z - H @ self.kf_x)
    self.kf_P = (np.eye(5) - K @ H) @ self.kf_P

    # x_pos, y_pos, theta, variance S[0,0], residual
    return(self.kf_x[0, 0], self.kf_x[1, 0], self.kf_x[2, 0], variance, residual, self.kf_P)
        

