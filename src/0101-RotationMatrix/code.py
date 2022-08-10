# -*- coding: utf-8 -*-

'''旋转矩阵方法集

旋转矩阵是最基本的姿态表达方法。本文件包含了以下与旋转矩阵有关的算法：

1. 欧拉角与旋转矩阵互相转换
2. 四元数与旋转矩阵互相转换
'''

import numpy as np
import math

# 取消科学计数法
np.set_printoptions(suppress=True, precision=4)

########################### 基础数学工具 ###########################

ZERO = 1e-6     # 浮点数误差阈值
PI = math.pi    # 圆周率
TO_DEG = 180/PI  # 弧度转角度
TO_RAD = PI/180  # 角度转弧度

sind = lambda angle : math.sin(angle*TO_RAD)
cosd = lambda angle : math.cos(angle*TO_RAD)
sin = lambda angle : math.sin(angle*TO_RAD)
cos = lambda angle : math.cos(angle*TO_RAD)
atan2 = lambda angle : math.atan2(angle*TO_RAD)
sqrt = lambda angle : math.sqrt(angle*TO_RAD)


########################### 欧拉角与旋转矩阵互转 ###########################


def euler_to_rotmat(euler):
    return np.dot(rotmat_z(euler[2]), np.dot(rotmat_y(euler[1]), rotmat_x(euler[0])))


def rotmat_to_euler(rot):
    sy = math.sqrt(rot[0, 0] * rot[0, 0] + rot[1, 0] * rot[1, 0])
    singular = (sy < 1e-6)
    if (not singular):
        x = math.atan2(rot[2, 1], rot[2, 2])
        y = math.atan2(-rot[2, 0], sy)
        z = math.atan2(rot[1, 0], rot[0, 0])
    else:
        x = math.atan2(-rot[1, 2], rot[1, 1])
        y = math.atan2(-rot[2, 0], sy)
        z = 0
    return np.array([x, y, z])*TO_DEG


def rotmat_x(angle):
    return np.array([[1, 0,           0],
                     [0, cosd(angle), -sind(angle)],
                     [0, sind(angle),  cosd(angle)]])


def rotmat_y(angle):
    return np.array([[cosd(angle), 0, sind(angle)],
                     [0,          1, 0],
                     [-sind(angle), 0, cosd(angle)]])


def rotmat_z(angle):
    return np.array([[cosd(angle), -sind(angle), 0],
                     [sind(angle),  cosd(angle), 0],
                     [0,           0,          1]])

########################### 四元数与旋转矩阵互转 ###########################


def quater_to_rotmat(quater):
    x, y, z, t = quater
    return np.array([[-1+2*x*x+2*t*t, 2*x*y-2*z*t, 2*x*z+2*y*t],
                     [2*x*y+2*z*t, -1+2*y*y+2*t*t, 2*y*z-2*x*t],
                     [2*x*z-2*y*t, 2*y*z+2*x*t, -1+2*z*z+2*t*t]])


def rotmat_to_quater(rotmat):
    r = rotmat
    t = 0.5*sqrt(1+r[0, 0]+r[1, 1]+r[2, 2])
    x = 0.25*(r[2, 1]-r[1, 2])/t
    y = 0.25*(r[0, 2]-r[2, 0])/t
    z = 0.25*(r[1, 0]-r[0, 1])/t
    return [x, y, z, t]


if __name__ == '__main__':
    euler_init = [10, 45, 90]
    print('初始欧拉角: ')
    print(euler_init)
    R = euler_to_rotmat(euler_init)
    print('\n欧拉角 -> 旋转矩阵: ')
    print(R)
    euler = rotmat_to_euler(R)
    print('\n旋转矩阵 -> 欧拉角: ')
    print(euler)
    quater = rotmat_to_quater(R)
    print('\n旋转矩阵 -> 四元数: ')
    print(quater)
    R2 = quater_to_rotmat(quater)
    print('\n四元数 -> 旋转矩阵: ')
    print(R2)
