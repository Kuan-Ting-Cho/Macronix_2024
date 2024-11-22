import numpy as np
import matplotlib.pyplot as plt
import math
from scipy.signal import gaussian

global PI
PI = math.pi

# Pressure sensor position
p_sensor_pos = [
    [0.0755, -0.15225], # RFR
    [0.0755, -0.07225], # RFL
    [-0.545, -0.15225], # RBR
    [-0.545, -0.07225], # RBL
    [0.0755, 0.7225],   # LFR
    [0.0755, 0.15225],  # LFL
    [-0.545, 0.7225],   # LBR
    [-0.545, 0.15225]  # LBL
]



def cal_cop(right, left, pos):
    if not any(right) and not any(left):  # fall down
        phase = 3
    elif not any(right):  # single support on left
        phase = 2
    elif not any(left):  # single support on right
        phase = 1
    else:  # double support
        phase = 0

    x_cop = 0
    y_cop = 0
    for i in range(len(right)):
        x_cop += right[i] * pos[i][0] + left[i] * pos[i+4][0]
        y_cop += right[i] * pos[i][1] + left[i] * pos[i+4][1]

    x_cop /= (sum(right) + sum(left))
    y_cop /= (sum(right) + sum(left))

    print(x_cop, y_cop)

def plot():
    # 定义参数
    T = 10  # 示例中的 T 值，可以根据需要修改
    Ampz = 1.0  # 示例中的 Ampz 值，可以根据需要修改
    t1 = 2.0  # 示例中的 t1 值，可以根据需要修改
    T_SSP = 5.0  # 示例中的 T_SSP 值，可以根据需要修改
    dt = 1.0  # 示例中的 dt 值，可以根据需要修改
    t2 = 8.0  # 示例中的 t2 值，可以根据需要修改

    # 创建时间数组
    t_values = np.linspace(0, 12, 1000)  # 时间范围可以根据需要调整

    # 计算函数值
    temp_footz_values = 1.2 / 1000 * np.cos(2 * np.pi * t_values / T)

    temp_alter_z_values = Ampz / (2 * np.pi) * (2 * np.pi * (t_values - t1) / T_SSP - np.sin(2 * np.pi * (t_values - t1) / T_SSP)) * (np.heaviside(t_values - t1 - dt, 1) - np.heaviside(t_values - t2, 1)) \
                        + Ampz * np.heaviside(t_values - t2, 1)

    # 绘制图表
    plt.figure(figsize=(10, 5))

    # 绘制 temp_footz
    plt.plot(t_values, temp_footz_values, label='temp_footz')

    # 绘制 temp_alter_z
    plt.plot(t_values, temp_alter_z_values, label='temp_alter_z')

    # 添加标题和标签
    plt.title('Comparison of temp_footz and temp_alter_z')
    plt.xlabel('Time')
    plt.ylabel('Function Value')

    # 添加图例
    plt.legend()

    # 显示图表
    plt.show()

def alter_hright():

    # 定义余弦函数
    def sinusoidal_wave(t):
        return ((t - np.sin(t))/ 2/PI)
    
    # def sinusoidal_wave(t):
    #     sin = []
    #     phase = 0
    #     for i in range(len(t)):
    #         sin.append((t[i]+phase- np.sin(t[i]+phase))/ 2 / PI)
    #     return sin

    # 定义过渡函数（高斯函数）
    def transition_function(t, t_trigger, sigma):
        return np.exp(-((t - t_trigger) / sigma)**2)

    # 定义参数

    

    
    T = 1  # 余弦函数的周期
    dt=0.01
    t = np.arange(dt, (T)+dt, dt)
    t3 = 0.5*T
    t4 = np.arange(0.4*T,0.6*T,dt)
    t_values = np.linspace(0, 2*PI, len(t4))  # 时间范围
    half = int(len(t_values)/2)

    t2 = np.linspace(0,40,100)
    tmp = []
    tmp2 = []

    sinu = np.concatenate([np.zeros(40),sinusoidal_wave(t_values),np.ones(40)])


    for i in range(len(t)):
        total = len(t)
        R_temp_footz = -1 * math.cos(2 * PI*t[i] / T) + 0.3 * sinu[i]
 
        pp = -1 * math.cos(2 * PI*t[i] / T) + 0.3 * np.heaviside(t[i] - t3, 1)
        tmp.append(R_temp_footz)
        tmp2.append(pp)

    # R_temp_footz = -1 * math.cos(2 * PI*t[i] / T) + 0.3 * np.heaviside(t[i] - t3, 1)


    # 绘制图表
    plt.figure(figsize=(10, 6))

    # 原始余弦函数
    plt.plot(sinu, label='Original Cosine Wave')
    plt.plot(tmp, label='foot')
    # plt.plot(tmp2, label='foot2')

    # 添加标签和标题
    plt.title('Smooth Transition in Cosine Wave')
    plt.xlabel('Time')
    plt.ylabel('Function Value')
    plt.legend()
    plt.grid(True)
    plt.show()

def sigmoid():
    def sig(x):
        return 1 / (1 + np.exp(-0.1*x))

    # 生成 x 值
    x = np.linspace(-100, 100, 200)

    # 计算对应的 Sigmoid 函数值
    y = sig(x)

    # 绘制 Sigmoid 函数图形
    plt.plot(x, y, label='Sigmoid Function')
    plt.title('Sigmoid Function')
    plt.xlabel('x')
    plt.ylabel('S(x)')
    plt.legend()
    plt.grid(True)
    plt.show()

    for i in range(len(x)):
        print(x[i],": ", y[i])


    


if __name__ == "__main__":
    alter_hright()
    sigmoid()
