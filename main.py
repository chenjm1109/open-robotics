import numpy as np
import matplotlib.pyplot as plt


# 生成电流信号
current_signal = np.sin(2 * np.pi * 10 * np.linspace(0, 1, 1000))

# 定义噪声类型和强度
noise_type = 'gaussian'
noise_amplitude = 0.1

# 生成噪声
if noise_type == 'gaussian':
    noise = np.random.normal(0, noise_amplitude, len(current_signal))
else:
    noise = np.random.uniform(-noise_amplitude, noise_amplitude, len(current_signal))

# 将噪声加入电流信号
noisy_current_signal = current_signal + noise


# 定义滤波器
filter_size = 10
filter_weights = np.ones(filter_size) / filter_size

# 应用滤波器
filtered_signal = np.convolve(noisy_current_signal, filter_weights, mode='same')

# 绘制图像
plt.plot(current_signal, label='Current Signal')
plt.plot(noisy_current_signal, label='Noisy Current Signal')
plt.plot(filtered_signal, label='Filtered Signal')
plt.legend()
plt.show()