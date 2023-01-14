from mpl_toolkits import mplot3d
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import rcParams
rcParams['font.family'] = 'Microsoft YaHei'



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

def plot_coordinate_frame(input_value, input_type='pose'):
    if input_type =='pose':
        x, y, z, roll, pitch, yaw = input_value
        T = np.eye(4)
        T[:3,3] = [x, y, z]
        T[:3,:3] = euler_to_matrix([roll, pitch, yaw])
    elif input_type =='trans_matrix':
        T = np.array(input_value)
    else:
        raise TypeError("输入参数类型错误，请输入位置坐标或变换矩阵")
    # 创建 3D 图形并设置窗口全屏

    fig = plt.figure(dpi=100, figsize=(16,9))
    ax = plt.axes(projection='3d')
    ax.view_init(30, -45)

    # 基座标系
    ax.quiver(0, 0, 0, 1, 0, 0, color='r', arrow_length_ratio=0.2)
    ax.quiver(0, 0, 0, 0, 1, 0, color='g', arrow_length_ratio=0.2)
    ax.quiver(0, 0, 0, 0, 0, 1, color='b', arrow_length_ratio=0.2)
    ax.text(0,0,0, '基坐标系', color='black')

    # 输入矩阵的坐标系
    x, y, z = T[:3, 0], T[:3, 1], T[:3, 2]
    ax.quiver(T[0, 3], T[1, 3], T[2, 3], x[0], x[1], x[2], color='r', arrow_length_ratio=0.2)
    ax.quiver(T[0, 3], T[1, 3], T[2, 3], y[0], y[1], y[2], color='g', arrow_length_ratio=0.2)
    ax.quiver(T[0, 3], T[1, 3], T[2, 3], z[0], z[1], z[2], color='b', arrow_length_ratio=0.2)
    ax.text(T[0, 3], T[1, 3], T[2, 3], '输入坐标系', color='black')
    
    # 自动调整坐标轴范围
    ax.autoscale_view(tight=True)
    
    # 设置坐标轴比例
    ax.set_aspect('equal', 'box')

    # # 设置坐标轴范围
    # ax.set_xlim([-1, 1])
    # ax.set_ylim([-1, 1])
    # ax.set_zlim([-1, 1])

    # 显示图像
    plt.show()
    
if __name__ == '__main__':
    # 定义变换矩阵
    T = np.array([[1, 0, 0, 1],
                    [0, 1, 0, 2],
                    [0, 0, 1, 3],
                    [0, 0, 0, 1]])
    # 绘制坐标系
    plot_coordinate_frame([1,2,3,0,0,45])