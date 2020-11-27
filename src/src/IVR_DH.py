import numpy as np
from numpy import sin as s
from numpy import cos as c

def get_htm(angles):
    t1, t2, t3, t4 = angles

    # Link lengths
    l1 = 2.5
    l3 = 3.5
    l4 = 3.0

    # Include the initial angles (in radians)
    t1 = t1 + 1.5708
    t2 = t2 + 1.5708

    R11 = np.around(c(t1)*c(t2)*c(t3)*c(t4) + s(t1)*s(t3)*c(t4) - c(t1)*s(t2)*s(t4), 2)
    R21 = np.around(s(t1)*c(t2)*c(t3)*c(t4) - c(t1)*s(t3)*c(t4) - s(t1)*s(t2)*s(t4), 2)
    R31 = np.around(s(t2)*c(t3)*c(t4) + c(t2)*s(t4), 2)
    R41 = 0.0

    R12 = np.around(-c(t1)*c(t2)*c(t3)*s(t4) - s(t1)*s(t3)*s(t4) - c(t1)*s(t2)*c(t4), 2)
    R22 = np.around(-s(t1)*c(t2)*c(t3)*s(t4) + c(t1)*s(t3)*s(t4) - s(t1)*s(t2)*c(t4), 2)
    R32 = np.around(-s(t2)*c(t3)*s(t4) + c(t2)*c(t4), 2)
    R42 = 0.0

    R13 = np.around(-c(t1)*c(t2)*s(t3) + s(t1)*c(t3), 2)
    R23 = np.around(-s(t1)*c(t2)*s(t3) - c(t1)*c(t3), 2)
    R33 = np.around(-s(t2)*s(t3), 2)
    R43 = 0.0

    R14 = np.around(c(t1)*c(t2)*c(t3)*c(t4)*l4 + s(t1)*s(t3)*c(t4)*l4 - c(t1)*s(t2)*s(t4)*l4 + c(t1)*c(t2)*c(t3)*l3 + s(t1)*s(t3)*l3, 2)
    R24 = np.around(s(t1)*c(t2)*c(t3)*c(t4)*l4 - c(t1)*s(t3)*c(t4)*l4 - s(t1)*s(t2)*s(t4)*l4 + s(t1)*c(t2)*c(t3)*l3 - c(t1)*s(t3)*l3, 2)
    R34 = np.around(s(t2)*c(t3)*c(t4)*l4 + c(t2)*s(t4)*l4 + s(t2)*c(t3)*l3 + l1, 2)
    R44 = 1.0

    # Return the HTM
    return np.array([np.array([R11, R12, R13, R14]), np.array([R21, R22, R23, R24]),
                    np.array([R31, R32, R33, R34]), np.array([R41, R42, R43, R44])])

# Test
#print(get_htm(0.0,1.5708,0.0,0.0))







