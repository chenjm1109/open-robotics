import collision_detection as cd
import numpy as np
import math
import random
import matplotlib.pyplot as plt
import time

class HungarianAlgorithm:
    def __init__(self, line_graph):
        self.line_graph = line_graph
        self.num_obj = len(line_graph)
        self.num_tool = len(line_graph[0])
        self.tool_obj = (np.zeros(self.num_tool)-1).tolist()
        self.find_count = 0
        self.is_found = np.zeros(self.num_tool).tolist()

    def calc(self):
        for i in range(self.num_obj):
            self.is_found = np.zeros(self.num_tool)
            if self._find(i):
                self.find_count += 1

    def _find(self, obj_index):
        for j in range(self.num_tool):
            if self.line_graph[obj_index][j] == 1 and self.is_found[j] == 0:
                self.is_found[j] = 1
                if self.tool_obj[j] < 0 or self._find(self.tool_obj[j]):
                    self.tool_obj[j] = obj_index
                    return True
        return False


num_obj = 4  # 工件数量
num_tool = 4 # 夹爪数量

# 夹爪在TCP的位置
tool_length = 10
tool_width = 5
tool_list = []
tool_list.append(np.array([0, 0, 0, 0, 0, 0]))
tool_list.append(np.array([tool_length, 0, 0, 0,0,0]))
tool_list.append(np.array([tool_length, tool_width, 0, 0,0,0]))
tool_list.append(np.array([0, tool_width, 0, 0,0,0]))

# 工件对夹爪的图连通
line_obj_tool = []

# 工作空间矩形
ws_length = 100
ws_width = 50
ws_A = np.array([0, 0, 0, 1])
ws_B = np.array([ws_length, 0, 0, 1])
ws_C = np.array([ws_length, ws_width, 0, 1])
ws_D = np.array([0, ws_width, 0, 1])

# 空间顶点集
ws_p_list = [ws_A, ws_B, ws_C, ws_D]
# 空间边缘集
ws_vec_list = [ws_B-ws_A, ws_C-ws_B, ws_D-ws_C, ws_A-ws_D]

# 工具矩形
t_length = 5
t_width = 3
t_A = np.array([0, 0, 0, 1])
t_B = np.array([t_length, 0, 0, 1])
t_C = np.array([t_length, t_width, 0, 1])
t_D = np.array([0, t_width, 0, 1])

# 工具顶点集
t_p_list = [t_A, t_B, t_C, t_D]

fig = plt.figure(figsize=(12, 12), dpi=300)

plt.xlim(-30,130)
plt.ylim(-50,110)


# 对于每个工件
for i in range(num_obj):
    line_obj_tool.append([])
    transmat_user_obj = cd.pose_to_transmat(cd.create_new_object()) # 工件在用户坐标系的位置
    plt.text(transmat_user_obj[0, 3], transmat_user_obj[1, 3], str(i), size=20, color='k')
    
    # 对于每个夹爪去抓取该工件时
    for j in range(num_tool):
        # TCP在用户坐标系的位置
        transmat_obj_tcp = cd.transmat_inv(cd.pose_to_transmat(tool_list[j]))
        transmat_user_tcp = np.dot(transmat_user_obj,transmat_obj_tcp)
        ws_t_p_list = []
        collision = False
        for k in range(len(t_p_list)):
            ws_t_p = np.dot(transmat_user_tcp, t_p_list[k])
            ws_t_p_list.append(ws_t_p)
        # 检查TCP是否碰撞
        if 1:
            if not cd.area1_is_in_area2(ws_t_p_list, ws_p_list):
                line_obj_tool[i].append(0)
                collision = True
            else:
                line_obj_tool[i].append(1)
        else:
            
            if cd.separating_axis_test(ws_p_list, ws_t_p_list):
                line_obj_tool[i].append(0)
                collision = True
            else:
                line_obj_tool[i].append(1)
        
        for k in range(len(ws_p_list)-1):
            plt.plot([ws_p_list[k][0], ws_p_list[k+1][0]], 
                    [ws_p_list[k][1], ws_p_list[k+1][1]], 
                    linestyle='--', color='g')
        plt.plot([ws_p_list[k+1][0], ws_p_list[0][0]], 
                    [ws_p_list[k+1][1], ws_p_list[0][1]], 
                    linestyle='--', color='g')

        if collision:
            color_obj = 'r'
        else:
            color_obj = 'b'

        plt.text(transmat_user_tcp[0, 3], transmat_user_tcp[1, 3]+3, str(j), size=16, color='b')
        for k in range(len(ws_t_p_list)-1):        
            plt.plot([ws_t_p_list[k][0], ws_t_p_list[k+1][0]], 
                    [ws_t_p_list[k][1], ws_t_p_list[k+1][1]], 
                    linestyle='-', color=color_obj)
        plt.plot([ws_t_p_list[k+1][0], ws_t_p_list[0][0]], 
                    [ws_t_p_list[k+1][1], ws_t_p_list[0][1]], 
                    linestyle='-', color=color_obj)


print(line_obj_tool)
ha = HungarianAlgorithm(line_obj_tool)
ha.calc()

for i in range(len(ha.tool_obj)):
    if ha.tool_obj[i]<0:
        print('夹爪'+str(i)+'没有合适的物体')
        continue
    print('工件'+str(ha.tool_obj[i])+'分配夹爪:'+str(i))
plt.savefig("hungarian_algorithm.png")

