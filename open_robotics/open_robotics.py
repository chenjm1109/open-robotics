import numpy as np
import libs.modern_robotics as mr
from constant import TO_DEG, TO_RAD
from dh_and_screw import dh_to_screw
from pose_conversion import trans_to_point, point_to_trans
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.axes3d import Axes3D
from plot_dh import plot_robot_pose, plot_joint_traj_animation, plot_traj_curve

np.set_printoptions(precision=3, suppress=True)
# 使用自定义主题
plt.style.use('open_robotics/orlight.mplstyle')

DH_TABLE = {
    "SCARA": [[0, 0.0,    0.0,     0.0, 0.0],
              [0, 200.0,  0.0,     0.0, 0.0],
              [1, 200.0,  180.0,   0.0, 0.0],
              [0, 0.0,    0.0,     0.0, 0.0]],
    "CR6": [[0, 0,      0,   0,     0],
            [0, 0,    -90,   0,   -90],
            [0, 809,    0,   0,     0],
            [0, 720.5,  0,   202, -90],
            [0, 0,    -90,  120,   0],
            [0, 0,     90,   120,   0]],
    "CR5": [[0, 0,     0,     0,   0],
            [0, 0,   -90,    0,  -90],
            [0, 809,   0,     0,   0],
            [0, 720.5,  0,    181, 0],
            [0, 0,    -90,     120,   0]],
    "R6": [[0, 0,     0,     0, 0],
           [0, 25,   -90,    0, -90],
           [0, 463,   0,     0, 0],
           [0, 35,   -90,    432, 0],
           [0, 0,    90,     0, 0],
           [0, 0,    -90,    0, 0]]
}


class OpenRobotics(object):
    def __init__(self, define, method="name"):
        if method == "mdh":
            self.parse_dh_list(define)
        elif method == "screw":
            self.screw_list, self.matrix_list = define
        elif method == "name":
            self.dh_list = np.array(DH_TABLE[define])
            self.parse_dh_list(self.dh_list)
        else:
            raise ValueError("无效的method名称")
        
    def show_info(self):
        print("DH参数表: \n[type  a  alpha  d  theta]")
        print(self.dh_list)

    def get_freedom(self):
        return len(self.screw_list)

    def parse_dh_list(self, dh_list):
        self.joint_type = dh_list[:, 0]
        self.screw_list, self.matrix_list = dh_to_screw(dh_list)
        self.joint_offset = []
        freedom = self.get_freedom()
        for i in range(freedom):
            if self.joint_type[i] == 0:
                self.joint_offset.append(dh_list[i, 4])
            elif self.joint_type[i] == 1:
                self.joint_offset.append(dh_list[i, 3])
            else:
                raise ValueError("非法的关节类型")
        self.joint_offset = np.array(self.joint_offset)

    def convert_joint_unit_to_view(self, joint):
        result = []
        for i in range(self.get_freedom()):
            if self.joint_type[i] == 0:
                result.append(joint[i]*TO_DEG)
            else:
                result.append(joint[i])
        return result

    def convert_joint_unit_to_calc(self, joint):
        result = []
        for i in range(self.get_freedom()):
            if self.joint_type[i] == 0:
                result.append(joint[i]*TO_RAD)
            else:
                result.append(joint[i])
        return result

    def calc_fkine(self, joint):
        joint = np.array(joint, dtype=float)
        screw_list = np.array(self.screw_list)
        matrix_home = self.matrix_list[-1]
        joint += self.joint_offset
        joint_calc = self.convert_joint_unit_to_calc(joint)
        matrix_end = mr.FKinSpace(matrix_home, screw_list.T, joint_calc)
        point = trans_to_point(matrix_end)
        return point

    def calc_ikine(self, point, joint_init=None):
        screw_list = np.array(self.screw_list)
        matrix_end = point_to_trans(point)
        matrix_home = self.matrix_list[-1]
        joint_init = joint_init or np.zeros(self.get_freedom())
        joint_init += self.joint_offset
        joint_init_calc = self.convert_joint_unit_to_calc(joint_init)
        res = mr.IKinSpace(screw_list.T, matrix_home,
                           matrix_end, joint_init_calc, 1e-2, 1e-4)
        if res[1]:
            joint_view = self.convert_joint_unit_to_view(res[0])
            joint_without_offset = joint_view - self.joint_offset
            return np.array(joint_without_offset)
        else:
            return False

    # *********************************************
    # * 轨迹规划
    # *********************************************

    def plan_joint_traj(self, joint_start, joint_end, vel_max, acc_max, method="t", interval=0.001):
        """
        用于规划关节空间中的轨迹
        
        参数：
            - joint_start: 起始关节角度
            - joint_end: 终止关节角度
            - vel_max: 最大关节速度
            - acc_max: 最大关节加速度
            - method: 规划方法，取值可以为 "3", "5", 或 "t"，分别表示三次多项式规划、五次多项式规划和时间优化规划
            - interval: 时间间隔，默认为 0.001 秒
        
        返回：
            - joint_traj: 一个 JointTrajectory 类型的对象，表示规划出的关节轨迹
            - time_list: 一个一维数组，表示关节轨迹对应的时间刻度
        """
        joint_start, joint_end = np.array(joint_start), np.array(joint_end)
        vel_max, acc_max = np.array(vel_max), np.array(acc_max)
        delta_joint = abs(joint_end-joint_start)
        if method == "3" or method == 3:
            # 三次多项式规划
            time_max_vel = 1.5*delta_joint/vel_max
            time_max_acc = np.sqrt(6*delta_joint/acc_max)
            max_time = np.max([np.max(time_max_vel), np.max(time_max_acc)])
            max_time = np.ceil(max_time / interval) * interval
            node_num = int(max_time/interval)+1
            joint_traj = mr.JointTrajectory(
                joint_start, joint_end, max_time, node_num, 3)
        elif method == "5" or method == 5:
            # 五次多项式规划
            time_max_vel = 15*delta_joint/(8*vel_max)
            time_max_acc = np.sqrt(10*delta_joint/(3**0.5*acc_max))
            max_time = np.max([np.max(time_max_vel), np.max(time_max_acc)])
            max_time = np.ceil(max_time / interval) * interval
            node_num = int(max_time/interval)+1
            joint_traj = mr.JointTrajectory(
                joint_start, joint_end, max_time, node_num, 5)
        elif method == "t":
            # 梯形速度规划
            max_time = 0
            for i in range(self.get_freedom()):
                # 遍历每个关节
                if delta_joint[i] < 1e-6:
                    # 如果关节角度变化量过小，则跳过
                    continue
                # 计算加速度和速度的缩放因子
                temp_a = acc_max[i]/delta_joint[i]
                temp_v = vel_max[i]/delta_joint[i]
                if temp_v**2 <= temp_a:
                    temp_max_time = (temp_a+temp_v**2)/(temp_a*temp_v)
                else:
                    temp_max_time = (4/temp_a)**0.5
                # 找到最大耗时关节对应的时间、加速度和速度缩放因子
                if temp_max_time > max_time:
                    max_time = temp_max_time
                    acc_scale = temp_a
                    vel_scale = temp_v
            # 对最长时间进行向上取整，并且根据需要重新计算速度缩放因子
            max_time = np.ceil(max_time / interval) * interval
            if acc_scale*max_time**2 >= 4:
                vel_scale = 0.5*(acc_scale*max_time-acc_scale **
                                 0.5*(acc_scale*max_time**2-4)**0.5)
            # 计算关节轨迹的节点数，并使用梯形速度进行轨迹规划
            node_num = int(max_time/interval)+1
            joint_traj = mr.JointTrajectory(
                joint_start, joint_end, max_time, node_num, "t", vel_scale, acc_scale)
        # 计算时间刻度
        time_list = np.linspace(0, max_time, node_num)
        return joint_traj, time_list

    def plot_joint_traj_curve(self, joint_traj, time_list):
        plot_traj_curve(joint_traj, time_list)

    def plot_joint(self, joint):
        plot_robot_pose(self.dh_list, joint)
        
    def plot_joint_traj_animation(self, joint_traj, time_list):
        plot_joint_traj_animation(self.dh_list, joint_traj, time_list)


if __name__ == "__main__":
    cr6 = OpenRobotics('CR6')
    cr6.show_info()
    point = cr6.calc_fkine([90, 45, 0, 0, 0, 0])
    joint = cr6.calc_ikine(point, [80, 50, 0, 0, 0, 20])
    joint_traj, time_list = cr6.plan_joint_traj(
        [-100, 80, -70, 60, -50, 0],
        [100, -80, 30, 40, 90, 30],
        [10]*6,
        [4]*6, "t")
    # cr6.view_joint_traj(joint_traj, time_list)
    cr6.plot_joint_traj_animation(joint_traj, time_list)
