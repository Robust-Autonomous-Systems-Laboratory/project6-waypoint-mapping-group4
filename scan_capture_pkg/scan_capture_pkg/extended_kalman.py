import numpy as np

# function takes all ROS topic inputs and updates KF prediction
# v = twist.linear.x
# w = twist.angular.z
# l_pos/r_pos = self.joint_state.position[x]
# imu_a = imu.linear_acceleration.x
# imu_omega = imu.angular_velocity.z

def ekf(self, v, w, l_pos, r_pos, prev_l_pos, prev_r_pos, imu_a, imu_omega):

    if self.ekf_prev_t.sec == 0 and self.ekf_prev_t.nanosec == 0:
        delta_t = 1e-9
    else:
        prev_t = self.ekf_prev_t.sec + (self.ekf_prev_t.nanosec * 1e-9)
        curr_t_msg = self.get_clock().now().to_msg()
        curr_t = curr_t_msg.sec + (curr_t_msg.nanosec * 1e-9)
        delta_t = curr_t - prev_t

    self.ekf_prev_t = self.get_clock().now().to_msg()

    #Propogate
    self.ekf_x[2, 0] = self.ekf_x[2, 0] + w * delta_t

    F = np.array([
        [1, 0, -v*np.sin(self.ekf_x[2, 0])*delta_t, np.cos(self.ekf_x[2, 0])*delta_t, 0],
        [0, 1,  v*np.cos(self.ekf_x[2, 0])*delta_t, np.sin(self.ekf_x[2, 0])*delta_t, 0],
        [0, 0, 1, 0, delta_t],
        [0, 0, 0, 1, 0],
        [0, 0, 0, 0, 1]
    ])

    self.ekf_x[0, 0] = self.ekf_x[0, 0] + v * np.cos(self.ekf_x[2, 0]) * delta_t
    self.ekf_x[1, 0] = self.ekf_x[1, 0] + v * np.sin(self.ekf_x[2, 0]) * delta_t

    self.ekf_x[3, 0] = v
    self.ekf_x[4, 0] = w

    self.ekf_P = F @ self.ekf_P @ F.T + self.Q

    if not prev_l_pos:
        v_left = (l_pos / delta_t) * 0.033 #0.033 is the wheel radius on the turtlebot
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

    # Update

    H = np.array([
        [0, 0, 0, 1, -self.L/2],
        [0, 0, 0, 1, self.L/2],
        [0, 0, 0, 1/delta_t, 0],
        [0, 0, 0, 0, 1]
    ])

    S = H @ self.ekf_P @ H.T + self.R
    K = self.ekf_P @ H.T @ np.linalg.inv(S)

    variance = S[0,0]
    residual = (z - H @ self.ekf_x)

    self.ekf_x = self.ekf_x + K @ (z - H @ self.ekf_x)
    
    self.ekf_P = (np.eye(5) - K @ H) @ self.ekf_P

    # x_pos, y_pos, theta, variance S[0,0], residual
    return(self.ekf_x[0, 0], self.ekf_x[1, 0], self.ekf_x[2, 0], variance, residual, self.ekf_P)