import numpy as np
from numpy import sin as s
from numpy import cos as c

def get_jacobian(angles):
    t1, t2, t3, t4 = angles

    # Link lengths
    l1 = 2.5
    l3 = 3.5
    l4 = 3.0

    # Include the initial angles (in radians)
    t1 = t1 + 1.5708
    t2 = t2 + 1.5708

    J11 = np.around(-s(t1)*c(t2)*c(t3)*c(t4)*l4 + c(t1)*s(t3)*c(t4)*l4 + s(t1)*s(t2)*s(t4)*l4 - s(t1)*c(t2)*c(t3)*l3 + c(t1)*s(t3)*l3, 2)
    J21 = np.around(c(t1)*c(t2)*c(t3)*c(t4)*l4 + s(t1)*s(t3)*c(t4)*l4 - c(t1)*s(t2)*s(t4)*l4 + c(t1)*c(t2)*c(t3)*l3 + s(t1)*s(t3)*l3, 2)
    J31 = 0.0

    J12 = np.around(-c(t1)*s(t2)*c(t3)*c(t4)*l4 - c(t1)*c(t2)*s(t4)*l4 - c(t1)*s(t2)*c(t3)*l3, 2)
    J22 = np.around(-s(t1)*s(t2)*c(t3)*c(t4)*l4 - s(t1)*c(t2)*s(t4)*l4 - s(t1)*s(t2)*c(t3)*l3, 2)
    J32 = np.around(c(t2)*c(t3)*c(t4)*l4 - s(t2)*s(t4)*l4 + c(t2)*c(t3)*l3, 2)

    J13 = np.around(-c(t1)*c(t2)*s(t3)*c(t4)*l4 + s(t1)*c(t3)*c(t4)*l4 - c(t1)*c(t2)*s(t3)*l3 + s(t1)*c(t3)*l3, 2)
    J23 = np.around(-s(t1)*c(t2)*s(t3)*c(t4)*l4 - c(t1)*c(t3)*c(t4)*l4 - s(t1)*c(t2)*s(t3)*l3 - c(t1)*c(t3)*l3, 2)
    J33 = np.around(-s(t2)*s(t3)*c(t4)*l4 - s(t2)*s(t3)*l3, 2)

    J14 = np.around(-c(t1)*c(t2)*c(t3)*s(t4)*l4 - s(t1)*s(t3)*s(t4)*l4 - c(t1)*s(t2)*c(t4)*l4, 2)
    J24 = np.around(-s(t1)*c(t2)*c(t3)*s(t4)*l4 + c(t1)*s(t3)*s(t4)*l4 - s(t1)*s(t2)*c(t4)*l4, 2)
    J34 = np.around(-s(t2)*c(t3)*s(t4)*l4 + c(t2)*c(t4)*l4, 2)

    return (np.array([np.array([J11, J12, J13, J14]), np.array([J21, J22, J23, J24]), np.array([J31, J32, J33, J34])]))

#print(get_jacobian(0.0,0.0,0.0,1.5708))













