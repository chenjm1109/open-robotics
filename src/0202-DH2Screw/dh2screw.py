import modern_robotics as mr
import numpy as np
import math

# 取消科学计数法
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

def dh_to_screw(dh):
    Mlist = []
    Slist = []
    MT = np.eye(4)
    freedom = len(dh)
    for i in range(freedom):
        if dh[i][0] == 0:
            M_i = np.dot(rot('x', dh[i][2]), np.dot(
                trans('x', dh[i][1]), trans('z', dh[i][3])))
            A_i = mr.VecTose3([0, 0, 1, 0, 0, 0])
        else:
            M_i = np.dot(rot('x', dh[i][2]), np.dot(
                trans('x', dh[i][1]), rot('z', dh[i][3])))
            A_i = mr.VecTose3([0, 0, 0, 0, 0, 1])
        MT = np.dot(MT, M_i)
        Mlist.append(MT)
        MTinv = mr.TransInv(MT)
        S_i = mr.se3ToVec(np.dot(MT, np.dot(A_i, MTinv)))
        Slist.append(S_i.tolist())
    return Slist, Mlist, MT.tolist()

def calcAxisAngle(v1, v2):
    """求两个三维向量的旋转轴和旋转角"""
    v3 = np.cross(v1, v2)
    n3 = np.linalg.norm(v3)
    if n3>0:
        u3 = (v3/np.linalg.norm(v3)).tolist()
    else:
        u3 = [0,0,0]
    n1 = np.linalg.norm(v1)
    n2 = np.linalg.norm(v2)
    angle = math.acos(np.dot(v1, v2)/(n1*n2))
    
    return u3, angle


if __name__ == "__main__":
    #     0-type  1-a  2-alpha  3-d/theta
    # dh = [[0, 0.0,    0.0,   250.0],
    #     [0, 150.0,  0.0,     0.0],
    #     [1, 150.0,  180.0,   0.0],
    #     [0, 0.0,    0.0,   0.0]]
    # Slist, Mlist, MT = dh_to_screw(dh)
    # joint = [100, 200, 50, -20]
    # point = mr.FKinSpace(MT, np.array(Slist).T, joint)
    # joint2 = mr.IKinSpace(np.array(Slist).T, MT, point, joint, 0.01, 0.001)
    # print(point)
    # print(joint2)
    
    print(calcAxisAngle([150, 0, 0], [0, 0, 150]))

