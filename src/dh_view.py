# -*- coding: utf-8 -*-

import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.axes3d import Axes3D

np.set_printoptions(suppress=True)

TO_DEG = 180/math.pi
TO_RAD = math.pi/180

def cos(deg):
    return math.cos(deg*TO_RAD)

def sin(deg):
    return math.sin(deg*TO_RAD)

def prase_dh(dh_list):
    dh_list = np.array(dh_list)
    T_list = np.zeros((dh_list.shape[0]+1, 4, 4))
    T_list[0] = np.eye(4)
    for i in range(dh_list.shape[0]):
        a, ap, d, t = dh_list[i,:]
        T_trans = [[cos(t),   -sin(t)*cos(ap),  sin(t)*sin(ap),  a*cos(t)],
                   [sin(t),    cos(t)*cos(ap), -cos(t)*sin(ap),  a*sin(t)],
                   [0,         sin(ap),         cos(ap),         d       ],
                   [0,         0,               0,               1       ]]
        T_list[i+1] = np.dot(T_list[i], T_trans)

    return T_list

def prase_mdh(mdh_list):
    mdh_list = np.array(mdh_list)
    T_list = np.zeros((mdh_list.shape[0]+1, 4, 4))
    T_list[0] = np.eye(4)
    for i in range(mdh_list.shape[0]):
        a, ap, d, t = mdh_list[i,:]
        T_trans = [[cos(t),        -sin(t),          0,        a        ],
                   [sin(t)*cos(ap), cos(t)*cos(ap), -sin(ap), -d*sin(ap)],
                   [sin(t)*sin(ap), cos(t)*sin(ap),  cos(ap),  d*cos(ap)],
                   [0,              0,               0,        1        ]]
        T_list[i+1] = np.dot(T_list[i], T_trans)

    return T_list

def dh_view(dh_list, dh_type="mdh"):
    if("mdh" == dh_type):
        T_list = prase_mdh(dh_list)
    else:
        T_list = prase_dh(dh_list)
    
    fig = plt.figure(figsize=(6,6))
    ax = fig.add_subplot(111, projection='3d')
    ax.view_init(elev=20., azim=45.)
    ax.set_xlabel("x(m)")
    ax.set_ylabel("y(m)")
    ax.set_zlabel("z(m)")
    ax.set_xlim3d(-100, 100)
    ax.set_ylim3d(-100, 100)
    ax.set_zlim3d(0, 200)
    
    scale = 30
    
    for i in range(T_list.shape[0]-1):
        ax.plot([T_list[i, 0, 3], T_list[i+1, 0, 3]],
                [T_list[i, 1, 3], T_list[i+1, 1, 3]],
                [T_list[i, 2, 3], T_list[i+1, 2, 3]], 
                color="#555", linewidth=3)
    for i in range(T_list.shape[0]): 
        ax.plot([T_list[i, 0, 3], T_list[i, 0, 3]+T_list[i, 0, 0]*scale],
                [T_list[i, 1, 3], T_list[i, 1, 3]+T_list[i, 1, 0]*scale],
                [T_list[i, 2, 3], T_list[i, 2, 3]+T_list[i, 2, 0]*scale], 
                color="#b85450", linewidth=3)
        ax.plot([T_list[i, 0, 3], T_list[i, 0, 3]+T_list[i, 0, 1]*scale],
                [T_list[i, 1, 3], T_list[i, 1, 3]+T_list[i, 1, 1]*scale],
                [T_list[i, 2, 3], T_list[i, 2, 3]+T_list[i, 2, 1]*scale], 
                color="#008000", linewidth=3)
        ax.plot([T_list[i, 0, 3], T_list[i, 0, 3]+T_list[i, 0, 2]*scale],
                [T_list[i, 1, 3], T_list[i, 1, 3]+T_list[i, 1, 2]*scale],
                [T_list[i, 2, 3], T_list[i, 2, 3]+T_list[i, 2, 2]*scale], 
                color="#9cdcfe", linewidth=3)
    plt.show()

if __name__ == "__main__":
    # ## Scara
    # # MDH参数表  a     alpha       d       theta
    # dh_list = [[ 0.0,    0.0,    250.0,  20.0],
    #             [ 150.0,  0,    0.0,    -20.0],
    #             [ 150.0, 180,    150.0,   0.0],
    #             [ 0.0,   0.0,    0.0,    20.0]]
    # dh_view(dh_list, dh_type="mdh")
    
    # # DH参数表   a     alpha       d       theta
    # dh_list = [[ 150,    0,    250,    20.0],
    #            [ 150,    180,    0.0,    -20.0],
    #            [ 0.0,    0.0,    150.0,   0.0],
    #            [ 0.0,    0.0,    0.0,    20.0]]
    # dh_view(dh_list, dh_type="dh")

    ## SixAxis
    # MDH参数表  a     alpha       d       theta
    dh_list = [[ 0.0,    0,      0,       0.0],
               [ 10.0,  -90,     0.0,    -90.0],
               [ 20.0,  0.0,     0.0,     0.0],
               [ 30.0,  -90.0,   40.0,    0.0],
               [ 0.0,    90.0,    0.0,    0.0],
               [ 0.0,   -90.0,    50.0,   30.0]]
    dh_view(dh_list, dh_type="mdh")
    
    # # DH参数表   a      alpha       d       theta
    # dh_list = [[ 50.0,   -90.0,    242,     0.0],
    #            [ 225.0,   0.0,     0.0,    -90.0],
    #            [ 50.0,   -90.0,    0.0,     0.0],
    #            [ 0.0,     90.0,  228.86,   0.0],
    #            [ 0.0,    -90.0,    0.0,     0.0],
    #            [ 0.0,     0.0,    50.0,    0.0]]
    # dh_view(dh_list, dh_type="dh")