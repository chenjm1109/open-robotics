import numpy as np
import libs.modern_robotics_plus as mr

def trans_to_point(trans):
    trans = np.array(trans)
    rotmat, posvec = mr.TransToRp(trans)
    euler = matrix_to_euler(rotmat)
    return np.r_[posvec, euler]

def point_to_trans(point):
    point = np.array(point.copy())
    posvec = point[0:3]
    rotmat = euler_to_matrix(point[3:6])
    return mr.RpToTrans(rotmat, posvec).copy()


def matrix_to_euler(matrix: np.ndarray) -> np.ndarray:
    """
    将旋转矩阵转换为欧拉角.

    参数:
        matrix (np.ndarray): 3x3 旋转矩阵

    返回:
        np.ndarray: 欧拉角（x, y, z），单位为角度
    """
    sy = np.sqrt(matrix[0, 0] * matrix[0, 0] + matrix[1, 0] * matrix[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = np.arctan2(matrix[2, 1], matrix[2, 2])
        y = np.arctan2(-matrix[2, 0], sy)
        z = np.arctan2(matrix[1, 0], matrix[0, 0])
    else:
        x = np.arctan2(-matrix[1, 2], matrix[1, 1])
        y = np.arctan2(-matrix[2, 0], sy)
        z = 0

    return np.degrees(np.array([x, y, z]))


def euler_to_matrix(euler: np.ndarray) -> np.ndarray:
    """
    将欧拉角转换为旋转矩阵

    参数:
        euler (np.ndarray): 欧拉角（x, y, z），单位为角度

    返回:
        np.ndarray: 3x3 旋转矩阵
    """
    x, y, z = np.radians(euler)
    c_x, s_x = np.cos(x), np.sin(x)
    c_y, s_y = np.cos(y), np.sin(y)
    c_z, s_z = np.cos(z), np.sin(z)
    matrix = np.array([
        [c_y*c_z, -c_x*s_z+s_x*s_y*c_z, s_x*s_z+c_x*s_y*c_z],
        [c_y*s_z, c_x*c_z+s_x*s_y*s_z, -s_x*c_z+c_x*s_y*s_z],
        [-s_y, s_x*c_y, c_x*c_y]
    ])
    return matrix


def matrix_to_quaternion(matrix):
    """
    将旋转矩阵转换为四元数
    
    参数:
        matrix (np.ndarray): 3x3 旋转矩阵
    
    返回:
        np.ndarray: 四元数（w, x, y, z）
    """
    q = np.empty((4, ))
    m00 = matrix[0, 0]
    m01 = matrix[0, 1]
    m02 = matrix[0, 2]
    m10 = matrix[1, 0]
    m11 = matrix[1, 1]
    m12 = matrix[1, 2]
    m20 = matrix[2, 0]
    m21 = matrix[2, 1]
    m22 = matrix[2, 2]
    t = m00 + m11 + m22
    if t > 0.0:
        s = np.sqrt(t + 1.0) * 2
        q[3] = 0.25 * s
        q[0] = (m21 - m12) / s
        q[1] = (m02 - m20) / s
        q[2] = (m10 - m01) / s
    elif (m00 > m11) and (m00 > m22):
        s = np.sqrt(1.0 + m00 - m11 - m22) * 2
        q[3] = (m21 - m12) / s
        q[0] = 0.25 * s
        q[1] = (m01 + m10) / s
        q[2] = (m02 + m20) / s
    elif m11 > m22:
        s = np.sqrt(1.0 + m11 - m00 - m22) * 2
        q[3] = (m02 - m20) / s
        q[0] = (m01 + m10) / s
        q[1] = 0.25 * s
        q[2] = (m12 + m21) / s
    else:
        s = np.sqrt(1.0 + m22 - m00 - m11) * 2
        q[3] = (m10 - m01) / s
        q[0] = (m02 + m20) / s
        q[1] = (m12 + m21) / s
        q[2] = 0.25 * s

    return q

if __name__ == '__main__':
    matrix = euler_to_matrix([45, 30, -20])
    euler = matrix_to_euler(matrix)
    print(matrix)
    print(euler)
