import numpy as np
import libs.modern_robotics as mr
from transform_matrix import trans, rot


def dh_to_screw(dh):
    """DH表示法转旋量表示法

    Args:
        dh (list): 机器人连杆结构的DH表示法, 其中包含n个DH参数, n是机器人自由度

    Returns:
        list: 处于初始位姿时, 空间坐标系下的关节旋量
        list: 处于初始位姿时, 各关节处的位姿矩阵
        list: 末端执行器的初始位姿矩阵


    Examples:
    >>> dh = [[0, 0.0, 0.0, 250.0, 0.0], [0, 150.0, 0.0, 0.0, 0.0],[1, 150.0, 180.0, 0.0, 0.0], [0, 0.0, 0.0, 0.0, 0.0]]
    >>> Slist, Mlist, MT = dh_to_screw(dh)
    """
    Mlist = []
    Slist = []
    MT = np.eye(4)
    freedom = len(dh)
    for i in range(freedom):
        if dh[i][0] == 0:
            M_i = rot('x', dh[i][2])@trans('x', dh[i][1])@trans('z', dh[i][3])
            A_i = mr.VecTose3([0, 0, 1, 0, 0, 0])
        else:
            M_i = rot('x', dh[i][2])@trans('x', dh[i][1])@rot('z', dh[i][4])
            A_i = mr.VecTose3([0, 0, 0, 0, 0, 1])
        MT = MT@M_i
        Mlist.append(MT.tolist())
        MTinv = mr.TransInv(MT)
        S_i = mr.se3ToVec(MT@A_i@MTinv)
        Slist.append(S_i.tolist())
    return Slist, Mlist
