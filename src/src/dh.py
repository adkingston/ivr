#!/usr/bin/env python3

from numpy import prod, cos, sin

# globals
## link lengths
L1 = 2.5
L2 = 3.5
L3 = 3.0

def get_htm(T):
    # we expect T to be a 1x4 array of the joint positions 
    # initial angles
    T[0] += 1.5708
    T[1] += 1.5708

    r11 = prod(cos(T)) + sin(T[0])*sin(T[2])*cos(T[3]) - cos(T[0])*sin(T[2])*sin(T[3])
    r21 = prod(cos(T)) + sin(T[0])*sin(T[2])*cos(T[3]) - cos(T[0])*sin(T[2])*sin(T[3])
    r31 = prod(cos(T)) + sin(T[0])*sin(T[2])*cos(T[3]) - cos(T[0])*sin(T[2])*sin(T[3])
    r41 = prod(cos(T)) + sin(T[0])*sin(T[2])*cos(T[3]) - cos(T[0])*sin(T[2])*sin(T[3])


