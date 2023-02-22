import numpy as np

# 生成一些连续数据
x = np.linspace(0, 10, 100)
y = np.sin(x)

# 计算一阶差分
diff_y = np.diff(y)
diff_y_smooth = np.zeros_like(diff_y)

# 定义滑动窗口大小
window_size = 5

# 使用滑动窗口平均值平滑数据
for i in range(window_size // 2, len(y) - window_size // 2):
    diff_y_smooth[i-1] = np.mean(diff_y[i-window_size//2:i+window_size//2])

# 绘制图形
import matplotlib.pyplot as plt

fig, ax = plt.subplots()
ax.plot(x, y, label='Original Data')
# ax.plot(x[:-1], diff_y_smooth, label='Smoothed Diff')
ax.plot(x[:-1], diff_y, label='Diff')
ax.legend()

plt.show()
