import numpy as np
import math
np.set_printoptions(suppress=True, precision=4)

ZERO = 1e-6     # 浮点数误差阈值
PI = math.pi    # 圆周率
TO_DEG = 180/PI  # 弧度转角度
TO_RAD = PI/180  # 角度转弧度
def sind(angle): return math.sin(angle*TO_RAD)
def cosd(angle): return math.cos(angle*TO_RAD)
def sin(angle): return math.sin(angle*TO_RAD)
def cos(angle): return math.cos(angle*TO_RAD)
def atan2(angle): return math.atan2(angle*TO_RAD)
def sqrt(angle): return math.sqrt(angle*TO_RAD)

def rot(dir, angle):
    if dir == 'x':
        return np.array([[1, 0,            0,            0],
                         [0, cosd(angle), -sind(angle),  0],
                         [0, sind(angle),  cosd(angle), 0],
                         [0, 0,            0,            1]])
    elif dir == 'y':
        return np.array([[cosd(angle),  0, sind(angle), 0],
                         [0,            1, 0,           0],
                         [-sind(angle), 0, cosd(angle), 0],
                         [0,            0, 0,           1]])
    elif dir == 'z':
        return np.array([[cosd(angle), -sind(angle), 0, 0],
                         [sind(angle),  cosd(angle), 0, 0],
                         [0,            0,           1, 0],
                         [0,            0,           0, 1]])


def trans(dir, dist):
    if dir == 'x':
        return np.array([[1, 0, 0, dist],
                         [0, 1, 0, 0],
                         [0, 0, 1, 0],
                         [0, 0, 0, 1]])
    elif dir == 'y':
        return np.array([[1, 0, 0, 0],
                         [0, 1, 0, dist],
                         [0, 0, 1, 0],
                         [0, 0, 0, 1]])
    elif dir == 'z':
        return np.array([[1, 0, 0, 0],
                         [0, 1, 0, 0],
                         [0, 0, 1, dist],
                         [0, 0, 0, 1]])