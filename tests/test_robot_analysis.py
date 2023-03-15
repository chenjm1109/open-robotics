import open_robotics as orb
import numpy as np
import config

def test_initDefaultData():
    # 测试用例1: 单位矩阵
    ra = orb.RobotAnalysis()
    ra.initDefaultData()
    assert ra.data.shape == (1000, 2)
    assert ra.data[0, 0] == 0.0
    assert ra.data[-1, 0] == 9.99
    assert np.allclose(np.diff(ra.data[:, 0]), 0.01)
