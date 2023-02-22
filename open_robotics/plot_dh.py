# -*- coding: utf-8 -*-

import numpy as np
import math
import matplotlib
import matplotlib.pyplot as plt

from mpl_toolkits.mplot3d.axes3d import Axes3D

matplotlib.rcParams['font.family'] = 'Microsoft YaHei'

np.set_printoptions(suppress=True)

TO_DEG = 180/math.pi
TO_RAD = math.pi/180


def cos(deg):
    return math.cos(deg*TO_RAD)


def sin(deg):
    return math.sin(deg*TO_RAD)


def prase_dh(dh_list):
    dh_list = np.array(dh_list)
    T_list = np.zeros((dh_list.shape[0]+1, 4, 4))
    T_list[0] = np.eye(4)
    for i in range(dh_list.shape[0]):
        a, ap, d, t = dh_list[i, :]
        T_trans = [[cos(t),   -sin(t)*cos(ap),  sin(t)*sin(ap),  a*cos(t)],
                   [sin(t),    cos(t)*cos(ap), -cos(t)*sin(ap),  a*sin(t)],
                   [0,         sin(ap),         cos(ap),         d],
                   [0,         0,               0,               1]]
        T_list[i+1] = np.dot(T_list[i], T_trans)

    return T_list


def prase_mdh(mdh_list, joint):
    mdh_list = np.array(mdh_list)
    T_list = np.zeros((mdh_list.shape[0]+1, 4, 4))
    T_list[0] = np.eye(4)
    for i in range(mdh_list.shape[0]):
        joint_type, a, ap, d, t = mdh_list[i, :]
        if joint_type == 0:
            t += joint[i]
        elif joint_type == 1:
            d += joint[i]
        T_trans = [[cos(t),        -sin(t),          0,        a],
                   [sin(t)*cos(ap), cos(t)*cos(ap), -sin(ap), -d*sin(ap)],
                   [sin(t)*sin(ap), cos(t)*sin(ap),  cos(ap),  d*cos(ap)],
                   [0,              0,               0,        1]]
        T_list[i+1] = np.dot(T_list[i], T_trans)
    return T_list


LIGHT_MODE = True

OR_RED = "#EF6665" if LIGHT_MODE else "#B8515F"
OR_GREEN = "#91CD76" if LIGHT_MODE else "#5CB789"
OR_BLUE = "#546FC6" if LIGHT_MODE else "#4382D4"
OR_ORANGE = "#fac958" if LIGHT_MODE else "#B69F50"
OR_BORDER = "#B5BCCF"
OR_TEXT = "#455377" if LIGHT_MODE else "#B5BCCF"
OR_BACKGROUND = "#f3f4fa" if LIGHT_MODE else "#100C2A"


def plot_cylinder(x, y, z, r, h, ax):
    # 创建圆柱体
    z_cylinder = np.linspace(z-h/2, z+h/2, 100)
    theta = np.linspace(0, 2*np.pi, 100)
    theta_grid, z_grid = np.meshgrid(theta, z_cylinder)
    x_grid = x + r*np.cos(theta_grid)
    y_grid = y + r*np.sin(theta_grid)
    ax.plot_surface(x_grid, y_grid, z_grid, color='r')

def plot_joint_traj_animation(dh_list, joint_traj, time_list):
    node_num = len(joint_traj)
    fig = plt.figure(dpi=100, figsize=(16, 9), facecolor=OR_BACKGROUND)
    ax = plt.axes(projection='3d')
    ax.view_init(elev=20., azim=45.)
    time_interval = 0.0
    for i in range(node_num):
        if i < node_num-1:
            time_interval += time_list[i+1]-time_list[i]
        else:
            time_interval = time_list[-1]
        if time_interval < 0.05:
            continue
        ax.clear()
        update_robot_pose(dh_list, joint_traj[i], ax)
        plt.pause(time_interval)
        time_interval = 0.0
    plt.show()
    
def plot_robot_pose(dh_list, joint):
    fig = plt.figure(dpi=100, figsize=(16, 9), facecolor=OR_BACKGROUND)
    ax = plt.axes(projection='3d')
    ax.view_init(elev=20., azim=45.)
    update_robot_pose(dh_list, joint, ax)
    plt.show()
    
def update_robot_pose(dh_list, joint, ax):
    dh_list = np.array(dh_list)
    T_list = prase_mdh(dh_list, joint)

    ax.set_xlabel("x(m)")
    ax.set_ylabel("y(m)")
    ax.set_zlabel("z(m)")
    max_size = (dh_list[:, 1].sum()+dh_list[:, 3].sum())*0.75
    ax.set_xlim([-max_size, max_size])
    ax.set_ylim([-max_size, max_size])
    ax.set_zlim([-max_size, max_size])

    ax.patch.set_facecolor(OR_BACKGROUND)

    # 坐标轴线
    ax.w_xaxis.line.set_color(OR_BORDER)
    ax.w_yaxis.line.set_color(OR_BORDER)
    ax.w_zaxis.line.set_color(OR_BORDER)

    # 坐标轴标签和刻度
    ax.set_xlabel('X 轴', color=OR_TEXT)
    ax.set_ylabel('Y 轴', color=OR_TEXT)
    ax.set_zlabel('Z 轴', color=OR_TEXT)
    ax.tick_params(axis='x', colors=OR_TEXT)
    ax.tick_params(axis='y', colors=OR_TEXT)
    ax.tick_params(axis='z', colors=OR_TEXT)

    # 坐标平面
    ax.xaxis.pane.set_facecolor(OR_BACKGROUND)
    ax.yaxis.pane.set_facecolor(OR_BACKGROUND)
    ax.zaxis.pane.set_facecolor(OR_BACKGROUND)

    # 网格
    ax.xaxis._axinfo["grid"]['color'] = OR_BORDER
    ax.yaxis._axinfo["grid"]['color'] = OR_BORDER
    ax.zaxis._axinfo["grid"]['color'] = OR_BORDER

    xs = T_list[:, 0, 3]
    ys = T_list[:, 1, 3]
    zs = T_list[:, 2, 3]
    scale = max_size/5
    for i in range(T_list.shape[0]-1):
        ax.plot([T_list[i, 0, 3], T_list[i+1, 0, 3]],
                [T_list[i, 1, 3], T_list[i+1, 1, 3]],
                [T_list[i, 2, 3], T_list[i+1, 2, 3]],
                color=OR_ORANGE, linewidth=5)
    for i in range(T_list.shape[0]):
        if i != T_list.shape[0]-1:
            actual_scale = scale*0.5
        else:
            actual_scale = scale
        ax.quiver(xs[i], ys[i], zs[i],
                  T_list[i, 0, 0]*actual_scale,
                  T_list[i, 1, 0]*actual_scale,
                  T_list[i, 2, 0]*actual_scale,
                  color=OR_RED, arrow_length_ratio=0.5)
        ax.quiver(xs[i], ys[i], zs[i],
                  T_list[i, 0, 1]*actual_scale,
                  T_list[i, 1, 1]*actual_scale,
                  T_list[i, 2, 1]*actual_scale,
                  color=OR_GREEN, arrow_length_ratio=0.5)
        ax.quiver(xs[i], ys[i], zs[i],
                  T_list[i, 0, 2]*actual_scale,
                  T_list[i, 1, 2]*actual_scale,
                  T_list[i, 2, 2]*actual_scale,
                  color=OR_BLUE, arrow_length_ratio=0.5)

    # # 自动调整坐标轴范围
    # ax.autoscale_view(tight=True)

    # 设置坐标轴比例
    ax.set_aspect('equal', 'box')


def plot_traj_curve(joint_traj:np.ndarray, time_list:np.ndarray):
    freedom = joint_traj.shape[1]
    # 计算速度和加速度
    velocity = np.diff(joint_traj, axis=0) / np.diff(time_list)[:, None]
    acceleration = np.diff(velocity, axis=0) / \
        np.diff(time_list[:-1])[:, None]

    # 绘制坐标、速度和加速度曲线
    fig, axs = plt.subplots(3, 1, figsize=(8, 8))
    for i in range(freedom):
        axs[0].plot(time_list, joint_traj[:, i], label=f"pos {i}")
        axs[1].plot(time_list[:-1], velocity[:, i],
                    label=f"vel {i}")
        axs[2].plot(time_list[:-2], acceleration[:, i],
                    label=f"acc {i}")
    axs[0].set_xlabel("时间(s)")
    axs[0].set_ylabel("关节位置")
    axs[0].legend(loc='upper left', bbox_to_anchor=(1.0, 1.0))
    axs[0].grid()
    axs[1].set_xlabel("时间(s)")
    axs[1].set_ylabel("关节速度")
    axs[1].legend(loc='upper left', bbox_to_anchor=(1.0, 1.0))
    axs[1].grid()
    axs[2].set_xlabel("时间(s)")
    axs[2].set_ylabel("关节加速度")
    axs[2].legend(loc='upper left', bbox_to_anchor=(1.0, 1.0))
    axs[2].grid()

    fig.subplots_adjust(hspace=0.4)
    fig.tight_layout()
    plt.show()

