import numpy as np

# generate sigma points
def generate_sigma_points(n, lambd, x, P):
    # matrix square root using Cholesky decomposition

    # componsate for non-positive definiteness by adding a super tiny value
    P_new = P + np.eye(n) * 1e-9

    U = np.linalg.cholesky((n + lambd) * P_new)
    sigmas = np.zeros((n, 2 * n + 1))
    sigmas[:, 0] = x.flatten()
    for k in range(n):
        sigmas[:, k + 1] = x.flatten() + U[k]
        sigmas[:, k + n + 1] = x.flatten() - U[k]
    return sigmas



# function takes all ROS topic inputs and updates KF prediction
# v = twist.linear.x
# w = twist.angular.z
# l_pos/r_pos = self.joint_state.position[x]
# imu_a = imu.linear_acceleration.x
# imu_omega = imu.angular_velocity.z

def ukf(self, v, w, l_pos, r_pos, prev_l_pos, prev_r_pos, imu_a, imu_omega):
    
    if self.ukf_prev_t.sec == 0 and self.ukf_prev_t.nanosec == 0:
        delta_t = 1e-9
    else:
        prev_t = self.ukf_prev_t.sec + (self.ukf_prev_t.nanosec * 1e-9)
        curr_t_msg = self.get_clock().now().to_msg()
        curr_t = curr_t_msg.sec + (curr_t_msg.nanosec * 1e-9)
        delta_t = curr_t - prev_t
        
    self.ukf_prev_t = self.get_clock().now().to_msg()    
    
    # Weights for mean and covariance
    self.weights_m[0] = self.lambd / (self.ukf_n + self.lambd)
    self.weights_c[0] = self.weights_m[0] + (1 - self.alpha**2 + self.beta)

    # predict 
    sigmas = generate_sigma_points(self.ukf_n, self.lambd, self.ukf_x, self.ukf_P)
    
    # Pass sigmas through motion model
    for i in range(2 * self.ukf_n + 1):
        theta = sigmas[2, i]
        v_s = sigmas[3, i]
        sigmas[0, i] += v_s * np.cos(theta) * delta_t
        sigmas[1, i] += v_s * np.sin(theta) * delta_t
        sigmas[2, i] += sigmas[4, i] * delta_t
        sigmas[3, i] = v
        sigmas[4, i] = w

    # Predicted Mean
    self.ukf_x = (sigmas @ self.weights_m).reshape(-1, 1)
    
    # Predicted Covariance
    P_prior = np.zeros((self.ukf_n, self.ukf_n))
    for i in range(2 * self.ukf_n + 1):
        diff = (sigmas[:, i:i+1] - self.ukf_x)
        P_prior += self.weights_c[i] * (diff @ diff.T)
    self.ukf_P = P_prior + self.Q

    sigmas = generate_sigma_points(self.ukf_n, self.lambd, self.ukf_x, self.ukf_P)
    z_dim = 4

    # This is the H matrix from the ekf transformed with sigmas
    Z_sigmas = np.zeros((z_dim, 2 * self.ukf_n + 1))
    for i in range(2 * self.ukf_n + 1):
        v_z, w_z = sigmas[3, i], sigmas[4, i]
        Z_sigmas[0, i] = v_z - w_z * self.L / 2 # v_left wheel ticks
        Z_sigmas[1, i] = v_z + w_z * self.L / 2 # v_right ticks
        Z_sigmas[2, i] = v_z / delta_t        # a_imu (approx)
        Z_sigmas[3, i] = w_z                  # w_imu

    if not prev_l_pos:
        v_left = (l_pos / delta_t) * 0.033  #0.033 [m] is the wheel radius on the turtlebot
        v_right = (r_pos / delta_t) * 0.033
    else:
        v_left = ((l_pos - prev_l_pos) / delta_t) * 0.033
        v_right = ((r_pos - prev_r_pos) / delta_t) * 0.033

    z_mean = (Z_sigmas @ self.weights_m).reshape(-1, 1)
    z_actual = np.array([[v_left], [v_right], [imu_a], [imu_omega]])

    # Calculate S and Cross-Covariance
    S = np.zeros((z_dim, z_dim))
    T = np.zeros((self.ukf_n, z_dim))
    for i in range(2 * self.ukf_n + 1):
        z_diff = Z_sigmas[:, i:i+1] - z_mean
        x_diff = sigmas[:, i:i+1] - self.ukf_x
        S += self.weights_c[i] * (z_diff @ z_diff.T)
        T += self.weights_c[i] * (x_diff @ z_diff.T)
    
    S += self.R
    K = T @ np.linalg.inv(S)

    variance = S[0,0]
    residual = (z_actual - z_mean)
    
    self.ukf_x = self.ukf_x + K @ (z_actual - z_mean)
    self.ukf_P = self.ukf_P - K @ S @ K.T


     # x_pos, y_pos, theta, variance S[0,0], residual
    return(self.ukf_x[0, 0], self.ukf_x[1, 0], self.ukf_x[2, 0], variance, residual, self.ukf_P)



