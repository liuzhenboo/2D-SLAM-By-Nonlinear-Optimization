# coding:utf-8
import numpy as np
import matplotlib.pyplot as plt
from numpy import linspace
import math

class Draw:
    def __init__(self):
        self._fig = plt.figure(figsize=(10,10))

        #设置图形元素
        ## 当前真实位置
        self._x = 0
        self._y = 0
        ## 当前估计位置
        self._x_e = 0
        self._y_e = 0
        ## 圆形扫描圈
        self._circlex = 0
        self._circley = 0
        ## 真实运动轨迹
        self._traj = np.array([[12.0],[3.0]])
        ## 估计轨迹
        self._traj_e = 0


    # 画出当前时刻路标点位置, 真实运动点,估计运动点,扫描半径
    def Show_Result(self, currentpose, r, landmarks, temp, temp1):
        # 真实位置(x,y)
        self._x = [currentpose[0,0]]
        self._y = [currentpose[1, 0]]
        # 真实轨迹
        self._traj = np.append((self._traj),[self._x, self._y], axis=1)
        # 扫描半径
        theta = np.arange(0, 2*np.pi, 0.01)
        self._circlex = (self._x)[0] + r * np.cos(theta)
        self._circley = (self._y)[0] + r * np.sin(theta)        

        # 作图
        plt.clf()
        plt.xlabel('x/m')
        plt.ylabel('y/m')
        plt.title('python_slam')
        plt.axis([-1, 21, -1, 21])
        ax = plt.gca()
        ax.set_aspect(1)
        plt.scatter(temp[0], temp[1], color='b')
        plt.plot(temp1[0], temp1[1], color= 'b')
        plt.plot(self._circlex,self._circley, color='b')
        plt.plot(self._x, self._y, color='b', marker='+')
        plt.scatter(landmarks[0], landmarks[1], color='r', marker='*')
        plt.plot((self._traj)[0], (self._traj)[1], color='r')
        plt.pause(0.01)


# plt.axis('equal')
# plt.set_xlim = (0, 10)
# plt.set_ylim=(0,4)

#clf() # 清图。
#cla() # 清坐标轴。
#close() # 关窗口
#https://www.zhihu.com/question/51213258/answer/124713752
#https://www.zhihu.com/question/353439108
#https://zhuanlan.zhihu.com/p/37595853
