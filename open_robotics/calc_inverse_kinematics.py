import numpy as np
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation as R

def fk(dh_params, q):
    """
    正向运动学计算
    """
    T = np.eye(4)
    for i in range(6):
        a, alpha, d, theta = dh_params[i]
        ct, st = np.cos(q[i]), np.sin(q[i])
        ca, sa = np.cos(alpha), np.sin(alpha)
        T_i = np.array([
            [ct, -st*ca, st*sa, a*ct],
            [st, ct*ca, -ct*sa, a*st],
            [0, sa, ca, d],
            [0, 0, 0, 1]
        ])
        T = np.dot(T, T_i)
    return T

def objective(q, dh_params, target_pose):
    """
    目标函数
    """
    T = fk(dh_params, q)
    p, R = T[:3,3], T[:3,:3]
    r = R.as_euler('zyx')
    err_p = np.linalg.norm(target_pose[:3] - p)
    err_r = np.linalg.norm(target_pose[3:] - r)
    return err_p + err_r

def calc_inverse_kinematics(dh_params, target_pose, q_init=None, method='SLSQP', bounds=None):
    """
    逆向运动学计算
    """
    if q_init is None:
        q_init = np.zeros(6)

    if bounds is None:
        bounds = [(None, None)] * 6

    # 优化器
    res = minimize(objective, q_init, args=(dh_params, target_pose), method=method, bounds=bounds)
    if not res.success:
        return None
    
    # 返回关节角
    return res.x