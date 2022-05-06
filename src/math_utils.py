'''常用数学函数

这里包含了一些常用数学函数及其变形，旨在使一些较为复杂的算法表达起来更加简洁

在其它文件中这样引用：

from math_utils import *
'''

import math

ZERO = 1e-6     # 浮点数误差阈值
PI = math.pi    # 圆周率
TO_DEG = 180/PI # 弧度转角度
TO_RAD = PI/180 # 角度转弧度


def sind(angle):
    # 角度正弦函数
    return math.sin(angle*TO_RAD)


def cosd(angle):
    # 角度余弦函数
    return math.cos(angle*TO_RAD)


def sin(angle):
    # 弧度正弦函数
    return math.sin(angle)


def cos(angle):
    # 弧度正弦函数
    return math.cos(angle)


def atan2(y, x):
    # 反正切函数
    return math.atan2(y, x)


def sqrt(x):
    # 开平方函数
    return math.sqrt(x)
