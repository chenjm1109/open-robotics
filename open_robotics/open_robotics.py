import numpy as np
import libs.modern_robotics as mr
from constant import TO_DEG, TO_RAD
from dh_and_screw import dh_to_screw
from pose_conversion import trans_to_point, point_to_trans
np.set_printoptions(precision=4, suppress=True)



DH_TABLE = {
    "CR6": [[0, 0,     0,     0, 0],
            [0, 0,   -90,    0, -90],
            [0, 809,   0,     0, 0],
            [0, 720.5,   0,    202, -90],
            [0, 0,    -90,     120, 0],
            [0, 0,    90,    120, 0]]
}


class OpenRobotics(object):
    def __init__(self, define, method="name"):
        if method == "mdh":
            self.parse_dh_list(define)
        elif method == "screw":
            self.screw_list, self.matrix_list = define
        elif method == "name":
            dh_list = np.array(DH_TABLE[define])
            self.parse_dh_list(dh_list)

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
    
    # def convert_point_unit_to_(self):
        
    def calc_fkine(self, joint):
        joint = np.array(joint, dtype=float)
        screw_list = self.screw_list
        matrix_home = self.matrix_list[-1]
        joint += self.joint_offset
        joint_calc = self.convert_joint_unit_to_calc(joint)
        matrix_end = mr.FKinSpace(matrix_home, screw_list, joint_calc)
        point = trans_to_point(matrix_end)
        return point

    def calc_ikine(self, point, joint_init=None):
        screw_list = self.screw_list
        matrix_end = point_to_trans(point)
        matrix_home = self.matrix_list[-1]
        joint_init = joint_init or np.zeros(self.get_freedom())
        joint_init += self.joint_offset
        res = mr.IKinSpace(screw_list, matrix_home, matrix_end, joint_init, 0.01, 0.001)
        if res[1]:
            return res[0] - self.joint_offset
        else:
            return False


if __name__ == "__main__":
    cr6 = OpenRobotics('CR6')
    # print(cr6.convert_joint_unit())
    point = cr6.calc_fkine([0, 0, 0, 0, 0, 0])
    print(point)