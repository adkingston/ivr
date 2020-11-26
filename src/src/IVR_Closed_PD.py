import numpy as np


def control_closed(self, image):
    # P gain
    K_p = np.array([[10.0, 0.0, 0.0], [0.0, 10.0, 0.0], [0.0, 0.0, 10.0]])
    # D gain
    K_d = np.array([[0.1, 0.0, 0.0], [0, 0.1, 0.0], [0.0, 0.0, 0.1]])
    # estimate time step
    cur_time = np.array([rospy.get_time()])
    dt = cur_time - self.time_previous_step
    self.time_previous_step = cur_time
    # robot end-effector position
    pos = self.detect_end_effector(image)
    # desired trajectory
    pos_d = self.trajectory()
    # estimate derivative of error
    self.error_d = ((pos_d - pos) - self.error) / dt
    # estimate error
    self.error = pos_d - pos
    q = self.detect_joint_angles(image)  # estimate initial value of joints'
    J_inv = np.linalg.pinv(self.calculate_jacobian(image))  # calculating the psudeo inverse of Jacobian
    dq_d = np.dot(J_inv, (np.dot(K_d, self.error_d.transpose()) + np.dot(K_p,
                                                                         self.error.transpose())))  # control input (angular velocity of joints)
    q_d = q + (dt * dq_d)  # control input (angular position of joints)
    return q_d