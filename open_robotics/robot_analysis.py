import numpy as np
from scipy import signal
import matplotlib.pyplot as plt
import libs.modern_robotics_plus as mrp
from matplotlib import rcParams
rcParams['font.family'] = 'Microsoft YaHei'
np.set_printoptions(precision=3, suppress=True)
# 使用自定义主题
plt.style.use('open_robotics/ordark.mplstyle')

"""
# 数据导入方法

- 文件导入
- 接口参数整体输入
- 接口时序输入

# 数据处理方法

- 轨迹数据差分
- 频谱变换
- 数据滤波
- 基于模型的分析方法
    - 笛卡尔轨迹变换
    - 轨迹奇异性分析

# 数据输出方法

- 文件格式化输出
- 函数格式化输出
- 可视化图片导出
"""

class RobotAnalysis(object):
    Y_LABEL = {
        'pos': '位置',
        'vel': '速度',
    }
    
    def __init__(self) -> None:
        self.initDefaultData()
        
    def initDefaultData(self):
        self.data = mrp.JointTrajectoryPlus([0, 50, -60, 40], [50, -30, 40, 70], [10, 10, 6, 20], [5, 5, 6, 3], 't', 0.02)
        self.data_type = 'pos'
    
    def getDataShape(self):
        return self.data[:,1:].shape
    
    def getNoiseData(self, data):
        timestamp = data[:, 0].reshape(-1, 1)
        data_without_timestamp = data[:, 1:]
        noise_without_timestamp = np.random.normal(0, 0.01, data[:,1:].shape)
        result_without_timestamp = data_without_timestamp + noise_without_timestamp
        return np.hstack((timestamp, result_without_timestamp))
    
    def getFiltData(self, data, fs=1000, fc=10, order=4):
        timestamp = data[:, 0].reshape(-1, 1)
        data_without_timestamp = data[:, 1:]
        # 定义滤波器参数
        # fs = 1000  # 采样率
        # fc = 10  # 截止频率
        # order = 4  # 阶数

        # 计算滤波器系数
        b, a = signal.butter(order, 2*fc/fs, 'lowpass')

        # 对数据进行滤波
        filtered_data = np.zeros_like(data_without_timestamp)
        for i in range(data_without_timestamp.shape[1]):
            filtered_data[:, i] = signal.filtfilt(b, a, data_without_timestamp[:, i])
        return np.hstack((timestamp, filtered_data))
    
    def getDiffData(self, data):
        timestamp = data[:, 0]
        dt = np.diff(timestamp)[:, None]
        diff_data = np.diff(data[:, 1:], axis=0)/dt
        return np.concatenate([data[1:, 0].reshape(-1, 1), diff_data], axis=1)
    
    def showTraj(self, noise = [0, 0, 0], filt = [0, 0, 0]):
        datas = []
        if self.data_type == 'pos':
            datas.append(self.data)
            if noise[0]:
                datas[0] = self.getNoiseData(datas[0])
            if filt[0]:
                datas[0] = self.getFiltData(datas[0])
            datas.append(self.getDiffData(datas[0]))
            if noise[1]:
                datas[1] = self.getNoiseData(datas[1])
            if filt[1]:
                datas[1] = self.getFiltData(datas[1])
            datas.append(self.getDiffData(datas[1]))
            if noise[2]:
                datas[2] = self.getNoiseData(datas[2])
            if filt[2]:
                datas[2] = self.getFiltData(datas[2])
                
        fig, axs = plt.subplots(3, 1, dpi=100, figsize=(8,9))
        fig.suptitle('轨迹数据')
        for i in range(len(axs)):
            ax= axs[i]
            data = datas[i]
            timestamp = data[:, 0]
            num_cols = data.shape[1]
            # 遍历数据列，并在同一个图表中绘制
            for i in range(1, num_cols):
                ax.plot(timestamp, data[:, i], label='数据 {}'.format(i))

            # 设置图表属性
            ax.set_xlabel('时间戳')
            ax.set_ylabel(RobotAnalysis.Y_LABEL[self.data_type])
            ax.legend()
            ax.grid()
        # 显示图表
        plt.tight_layout()
        plt.show()
        
            
    def showData(self):
        data = self.data
        # 创建figure
        _, ax = plt.subplots(dpi=100, figsize=(12,6))
        num_cols = data.shape[1]
        timestamp = data[:, 0]
        
        # 遍历数据列，并在同一个图表中绘制
        for i in range(1, num_cols):
            ax.plot(timestamp, data[:, i], label='数据 {}'.format(i))

        # 设置图表属性
        ax.set_xlabel('时间戳')
        ax.set_ylabel(RobotAnalysis.Y_LABEL[self.data_type])
        ax.set_title('原始数据')
        ax.legend()
        ax.grid()

        # 显示图表
        plt.show()
        

# ra = RobotAnalysis()
# ra.initDefaultData()
# ra.showTraj(noise=[1,0,0], filt=[1,1,1])