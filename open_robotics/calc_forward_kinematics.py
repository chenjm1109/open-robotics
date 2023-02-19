import numpy as np

def dh_transform(a, alpha, d, theta):
    """
    DH参数转换矩阵
    """
    ct = np.cos(theta)
    st = np.sin(theta)
    ca = np.cos(alpha)
    sa = np.sin(alpha)

    T = np.array([
        [ct, -st * ca, st * sa, a * ct],
        [st, ct * ca, -ct * sa, a * st],
        [0, sa, ca, d],
        [0, 0, 0, 1]
    ])

    return T

def calc_forward_kinematics(dh_params, joint_angles):
    """
    正向运动学计算
    """
    T = np.eye(4)
    for i in range(len(joint_angles)):
        a, alpha, d, theta = dh_params[i]
        T = np.dot(T, dh_transform(a, alpha, d, theta + joint_angles[i]))

    pos = T[:3, 3]
    rot = T[:3, :3]
    euler = np.array([
        np.arctan2(rot[2, 1], rot[2, 2]),
        np.arctan2(-rot[2, 0], np.sqrt(rot[2, 1]**2 + rot[2, 2]**2)),
        np.arctan2(rot[1, 0], rot[0, 0])
    ])

    return np.concatenate([pos, euler])

# 示例DH参数和关节角
dh_params = [
    [0, 0, 0.1, 0],
    [0, np.pi/2, 0, 0],
    [0, 0, 0.2, 0],
    [0, np.pi/2, 0, 0],
    [0, 0, 0.2, 0],
    [0, np.pi/2, 0, 0]
]

joint_angles = [0, 0, 0, 0, 0, 0]

# 计算机械臂末端位置
end_effector_pose = calc_forward_kinematics(dh_params, joint_angles)
print(end_effector_pose)