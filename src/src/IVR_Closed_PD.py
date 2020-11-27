import numpy as np


# needed inputs:
# previous timestamp
# current timestamp
# end effector POSITION (not angle)
# target position
# joint angles 
# jabobian
def control_closed(dt, ee_pos, target_pos, error, angles, jacobian):
    # P gain
    K_p = np.array([[-0.5, 0.0, 0.0], [0.0, -0.5, 0.0], [0.0, 0.0, -0.5]])
    # D gain
    K_d = np.array([[-0.1, 0.0, 0.0], [0, 0.1, 0.0], [0.0, 0.0, -0.1]])
    # estimate derivative of error
    error_d = (np.abs((target_pos - ee_pos)) - error) / dt
    # estimate error
    error = target_pos - ee_pos
    
    J_inv = np.linalg.pinv(jacobian)  # calculating the psudeo inverse of Jacobian
    dq_d = np.dot(J_inv, (np.dot(K_d, error_d.transpose()) + np.dot(K_p, error.transpose())))  # control input (angular velocity of joints)
    q_d = angles + (dt * dq_d)  # control input (angular position of joints)
    return error, q_d
