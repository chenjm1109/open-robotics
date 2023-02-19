import numpy as np
from src.calc_forward_kinematics import calc_forward_kinematics
from src.calc_inverse_kinematics import calc_inverse_kinematics


def test_kinematics_consistency():
    # 测试正逆运动学是否一致
    dh_params = [
        [0, 0, 1, 0],
        [0, -np.pi/2, 0, 0],
        [0, np.pi/2, 0, 0],
        [0, -np.pi/2, 1, 0],
        [0, np.pi/2, 0, 0],
        [0, 0, 1, 0]
    ]
    q = [0, 0, 0, 0, 0, 0]
    expected_pose = np.array([
        [0, 1, 0, 0],
        [-1, 0, 0, 1],
        [0, 0, 1, 3],
        [0, 0, 0, 1]
    ])
    np.testing.assert_allclose(forward_kinematics(dh_params, q), expected_pose, rtol=1e-4)
    
    # 测试逆向运动学是否一致
    target_pose = expected_pose
    expected_q = q
    np.testing.assert_allclose(inverse_kinematics(dh_params, target_pose), expected_q, rtol=1e-4)
    
    # 测试正逆运动学是否一致
    q_ik = inverse_kinematics(dh_params, target_pose)
    np.testing.assert_allclose(forward_kinematics(dh_params, q_ik), target_pose, rtol=1e-4)