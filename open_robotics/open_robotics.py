import numpy as np
import libs.modern_robotics as mr
from constant import TO_DEG, TO_RAD
from dh_and_screw import dh_to_screw
from pose_conversion import trans_to_point, point_to_trans
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.axes3d import Axes3D
from plot_dh import dh_view, OR_BACKGROUND
np.set_printoptions(precision=3, suppress=True)


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
        
    def view_joint(self, joint):
        fig = plt.figure(dpi=100, figsize=(16, 9), facecolor=OR_BACKGROUND)
        ax = plt.axes(projection='3d')
        ax.view_init(elev=20., azim=45.)
        dh_view(self.dh_list, joint, ax)
        plt.show()
        
    def view_plan_joint_traj(self, joint_start, joint_end):
        total_time = 5
        node_num = 101
        joint_traj = mr.JointTrajectory(joint_start, joint_end, total_time, node_num, 3)
        fig = plt.figure(dpi=100, figsize=(16, 9), facecolor=OR_BACKGROUND)
        ax = plt.axes(projection='3d')
        ax.view_init(elev=20., azim=45.)
        import time
        elapsed_time = 0
        for i in range(node_num):
            start = time.time()
            ax.clear()
            dh_view(self.dh_list, joint_traj[i], ax)
            end = time.time()
            elapsed_time = (end-start)
            pause_time = total_time/(node_num-1)-elapsed_time
            if pause_time < 0:
                pause_time = 0.1
            plt.pause(pause_time)
            
        plt.show()

if __name__ == "__main__":
    cr6 = OpenRobotics('CR5')
    point = cr6.calc_fkine([90, 45, 0, 0, 0])
    joint = cr6.calc_ikine(point, [80, 50, 0, 0, 0])
    cr6.view_plan_joint_traj([0, 0, 0, 0, 0], [20, 0, 90, 0, 90])
    