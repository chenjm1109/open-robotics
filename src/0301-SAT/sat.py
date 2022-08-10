import numpy as np
import math
import random
import matplotlib.pyplot as plt

# 取消科学计数法
np.set_printoptions(suppress=True, precision=4)

########################### 基础数学工具 ###########################

ZERO = 1e-6     # 浮点数误差阈值
PI = math.pi    # 圆周率
TO_DEG = 180/PI  # 弧度转角度
TO_RAD = PI/180  # 角度转弧度


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


def pose_to_transmat(pose):
    p = np.array([pose[0], pose[1], pose[2]])
    R = euler_to_rotmat([pose[3], pose[4], pose[5]])
    return np.r_[np.c_[R, p], [[0, 0, 0, 1]]]


def transmat_to_pose(transmat):
    r, p = transmat_to_rp(transmat)
    euler = rotmat_to_euler(r)
    return np.array([p[0], p[1], p[2], euler[0], euler[1], euler[2]])


def transmat_to_rp(transmat):
    transmat = np.array(transmat)
    return transmat[0:3, 0:3], transmat[0:3, 3]


def transmat_inv(transmat):
    r, p = transmat_to_rp(transmat)
    rt = np.array(r).T
    return np.r_[np.c_[rt, -np.dot(rt, p)], [[0, 0, 0, 1]]]

# 判断点是否在给定凸多边形区域内


def point_is_in_area(point, area_p_list):
    area_vec_list = []
    size = len(area_p_list)
    for i in range(size):
        area_vec_list.append(area_p_list[(i+1) % size]-area_p_list[i])
    # 对于边界的每条边
    for i in range(size):
        ws_p_vec = point-area_p_list[i]
        if_pass = np.cross(area_vec_list[i][0:2], ws_p_vec[0:2])
        # 倘若点位于某条边的外侧，说明不在区域内
        if if_pass <= 0:
            return False
    return True

# 判断多边形1是否在多边形2内部


def area1_is_in_area2(area_p_list_1, area_p_list_2):
    # area_vec_list_1 = []
    size_1 = len(area_p_list_1)
    size_2 = len(area_p_list_2)
    # for i in range(size_1):
    #     area_vec_list_1.append(area_p_list_1[(i+1)%size_1]-area_p_list_1[i])
    for i in range(len(area_p_list_1)):
        if not point_is_in_area(area_p_list_1[i], area_p_list_2):
            return False
    return True

# 分离轴测试——判断两个多边形是否相交


def separating_axis_test(area_p_list_1, area_p_list_2):
    area_vec_list_1 = []
    size_1 = len(area_p_list_1)
    for i in range(size_1):
        area_vec_list_1.append(area_p_list_1[(i+1) % size_1]-area_p_list_1[i])

    area_vec_list_2 = []
    size_2 = len(area_p_list_2)
    for i in range(size_2):
        area_vec_list_2.append(area_p_list_2[(i+1) % size_2]-area_p_list_2[i])

    for i in range(size_1):
        for j in range(size_2):
            area_vec_list_2_1 = area_p_list_2[j]-area_p_list_1[i]
            if_pass = np.cross(area_vec_list_2_1[0:2], area_vec_list_1[i][0:2])
            if if_pass < 0:
                break
            if j == size_2-1:
                return False

    for i in range(size_2):
        for j in range(size_1):
            area_vec_list_1_2 = area_p_list_1[j]-area_p_list_2[i]
            if_pass = np.cross(area_vec_list_1_2[0:2], area_vec_list_2[i][0:2])
            if if_pass < 0:
                break
            if j == size_1-1:
                return False

    return True

########################### main ###########################


def create_new_object():
    return [
        random.random()*140-20,
        random.random()*70-10,
        0,
        0,
        0,
        random.random()*360-180
    ]


if __name__ == '__main__':

    # area_p_list_1 = [np.array([0,0,0]),
    #                 np.array([2,0,0]),
    #                 np.array([2,2,0]),
    #                 np.array([0,2,0])]
    # area_p_list_2 = [np.array([3,0,0]),
    #                 np.array([6,0,0]),
    #                 np.array([6,3,0]),
    #                 np.array([3,3,0])]

    # print(separating_axis_test(area_p_list_1, area_p_list_2))
    # pass
    # 工作空间矩形
    ws_length = 100
    ws_width = 50
    ws_A = np.array([0, 0, 0, 1])
    ws_B = np.array([ws_length, 0, 0, 1])
    ws_C = np.array([ws_length, ws_width, 0, 1])
    ws_D = np.array([0, ws_width, 0, 1])

    # 空间顶点集
    ws_p_list = [ws_A, ws_B, ws_C, ws_D]
    # 空间边缘集
    ws_vec_list = [ws_B-ws_A, ws_C-ws_B, ws_D-ws_C, ws_A-ws_D]

    # 工具矩形
    t_length = 20
    t_width = 5
    t_A = np.array([0, 0, 0, 1])
    t_B = np.array([t_length, 0, 0, 1])
    t_C = np.array([t_length, t_width, 0, 1])
    t_D = np.array([0, t_width, 0, 1])

    # 工具顶点集
    t_p_list = [t_A, t_B, t_C, t_D]

    fig = plt.figure(figsize=(8, 4.5), dpi=300)
    plt.xlim(-30, 130)
    plt.ylim(-50, 110)
    ax = plt.gca()
    ax.set_aspect(1)
    # 创建工件
    for k in range(20):
        # 工具末端在工作空间的位置
        ws_t = create_new_object()
        trans_ws_t = pose_to_transmat(ws_t)
        ws_t_p_list = []
        for i in range(len(t_p_list)):
            ws_t_p = np.dot(trans_ws_t, t_p_list[i])
            ws_t_p_list.append(ws_t_p)
        collision = False
        if 0:
            if not area1_is_in_area2(ws_t_p_list, ws_p_list):
                collision = True
        else:
            if separating_axis_test(ws_p_list, ws_t_p_list):
                collision = True

        for i in range(len(ws_p_list)-1):
            plt.plot([ws_p_list[i][0], ws_p_list[i+1][0]],
                     [ws_p_list[i][1], ws_p_list[i+1][1]],
                     linestyle='--', color='#98c379')
        plt.plot([ws_p_list[i+1][0], ws_p_list[0][0]],
                 [ws_p_list[i+1][1], ws_p_list[0][1]],
                 linestyle='--', color='#98c379')

        if collision:
            COLOR_OBJ = '#d46c75'
        else:
            COLOR_OBJ = '#61afef'

        for i in range(len(ws_t_p_list)-1):
            plt.plot([ws_t_p_list[i][0], ws_t_p_list[i+1][0]],
                     [ws_t_p_list[i][1], ws_t_p_list[i+1][1]],
                     linestyle='-', color=COLOR_OBJ)
        plt.plot([ws_t_p_list[i+1][0], ws_t_p_list[0][0]],
                 [ws_t_p_list[i+1][1], ws_t_p_list[0][1]],
                 linestyle='-', color=COLOR_OBJ)

    plt.savefig("src/collision_detection/collision_detection.png")
