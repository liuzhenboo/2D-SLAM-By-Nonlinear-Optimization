import numpy as np
import matplotlib.pyplot as plt
from scipy.linalg import solve_discrete_are

# 参数定义
m = 1.0  # 小车质量
dt = 0.033  # 时间步长
t_max = 10.0  # 模拟总时间
num_steps = int(t_max / dt)  # 时间步数

# 系统矩阵
A = np.array([[0, 1],
              [0, 0]])

B = np.array([[0],
              [1/m]])

C = np.array([[1, 0],[0, 0]])

# 观测器增益矩阵（单位阵）
# # L = np.eye(2)
# L = np.array([[1, 100],[0, 1]])
# 离散化系统矩阵
Ad = np.eye(2) + A * dt
Bd = B * dt

# 定义Riccati方程的权重矩阵
Q = np.eye(2)  # 状态权重
R = np.eye(2)  # 控制权重

# 求解离散Riccati方程
P = solve_discrete_are(Ad.T, C.T, Q, R)
L = np.dot(np.dot(P, C.T), np.linalg.inv(R + np.dot(np.dot(C, P), C.T)))
print("opt L:",L)
# 初始化状态
x = np.array([[0],  # 初始位置
              [0.5]]) # 初始速度

# 观测器初始状态
x_hat = np.array([[0],  # 初始位置估计
                  [0]]) # 初始速度估计

# 记录状态和估计值
x_history = []
x_hat_history = []

# 输入力（这里设为常数，可以修改为任意输入）
F = 0.1

# 噪声参数
noise_mean = 0.01
noise_std = 1

# 模拟过程
for _ in range(num_steps):
    # 真正的状态更新
    x = x + dt * (A @ x + F * B)
    
    # 输出
    y = C @ x + np.random.normal(noise_mean, noise_std)
    
    # 计算误差
    error = y - C @ x_hat
    
    # 观测器状态更新
    x_hat = x_hat + dt * (A @ x_hat + F * B  + L @ error)
    
    # 记录状态和估计值
    x_history.append(x.flatten())
    x_hat_history.append(x_hat.flatten())

# 转换为numpy数组
x_history = np.array(x_history)
x_hat_history = np.array(x_hat_history)

# 绘制结果
time = np.linspace(0, t_max, num_steps)
plt.figure(figsize=(10, 6))

plt.subplot(2, 1, 1)
plt.plot(time, x_history[:, 0], label='True Position', linestyle='--')
plt.plot(time, x_hat_history[:, 0], label='Estimated Position')
plt.xlabel('Time (s)',fontsize=14)
plt.ylabel('Position (normalized)',fontsize=14)
plt.legend(fontsize=14)

plt.subplot(2, 1, 2)
plt.plot(time, x_history[:, 1], label='True Velocity', linestyle='--')
plt.plot(time, x_hat_history[:, 1], label='Estimated Velocity')
plt.xlabel('Time (s)',fontsize=14)
plt.ylabel('Velocity (normalized)',fontsize=14)
plt.legend(fontsize=14)

plt.tight_layout()
plt.show()
