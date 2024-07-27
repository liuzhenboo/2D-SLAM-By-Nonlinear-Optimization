# coding:utf-8
# create by liuzhenbo 2020/8/16 in nwpu

import numpy as np
import matplotlib.pyplot as plt
from numpy import linspace
import math

class Draw:
    def __init__(self, landmarks, slidewindow_graph, movemodel):
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

        self._landmarks = landmarks
        self._graph = slidewindow_graph
        self._movemodel = movemodel

    # 画出当前时刻路标点位置, 真实运动点,估计运动点,扫描半径
    def Show_result(self, r, file):
        # 真实位置(x,y)
        self._x = [self._movemodel._currentpose[0,0]]
        self._y = [self._movemodel._currentpose[1, 0]]
        # file.write(str(self._movemodel._currentpose[0][-1]) + ' ' + str(self._movemodel._currentpose[1][-1]) + '\n')

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
        # 预测轨迹: self._graph._esti_pose[0] --> x序列, self._graph._esti_pose[1] --> y序列
        plt.plot(self._graph._esti_pose[0], self._graph._esti_pose[1], color= 'b')
        if self._graph._esti_pose[0]:
            file.write(str(self._graph._esti_pose[0][-1]) + ' ' + str(self._graph._esti_pose[1][-1]) + '\n')

        # 传感器范围
        plt.plot(self._circlex,self._circley, color='b', linestyle='--')
        # 真实位置
        plt.plot(self._x, self._y, color='b', marker='+',linewidth=2)
        # 真实路标位置
        plt.scatter(self._landmarks._landmarks[0], self._landmarks._landmarks[1], color='r', marker='*')
        # 轨迹真值
        plt.plot((self._traj)[0], (self._traj)[1], color='r')
        # 滑动窗口位姿
        plt.scatter(self._graph._slideframes[0], self._graph._slideframes[1], color='r', marker='s')
        # 滑窗内路标
        plt.scatter(self._graph._slidepoints[0], self._graph._slidepoints[1], color='b', marker='*')
        # 帧间共同约束点
        plt.scatter(self._graph._f2ftrack_show[0], self._graph._f2ftrack_show[1],color='b',marker='s')
        plt.pause(0.01)

    def Save_result(self):
        plt.savefig('result.png')
        plt.show()
        plt.close()

