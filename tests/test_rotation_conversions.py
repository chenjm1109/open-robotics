# test_my_algorithm.py
import config
import numpy as np
from src.rotation_conversions import matrix_to_euler

def test_matrix_to_euler():
    # 测试用例1: 单位矩阵
    matrix = np.identity(3)
    expected_euler = np.array([0, 0, 0])
    result = matrix_to_euler(matrix)
    assert np.allclose(result, expected_euler), f'错误: {result}'

    # 测试用例2: 绕x轴旋转90度的矩阵
    matrix = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]])
    expected_euler = np.array([90, 0, 0])
    result = matrix_to_euler(matrix)
    assert np.allclose(result, expected_euler), f'错误: {result}'
    
    # 测试用例3: 绕y轴旋转45度的矩阵
    matrix = np.array([[np.sqrt(2)/2, 0, np.sqrt(2)/2], [0, 1, 0], [-np.sqrt(2)/2, 0, np.sqrt(2)/2]])
    expected_euler = np.array([0, 45, 0])
    result = matrix_to_euler(matrix)
    assert np.allclose(result, expected_euler), f'错误: {result}'