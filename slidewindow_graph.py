# coding:utf-8

import numpy as np
from math import sin, cos
import math
from scipy import optimize

from frame import Frame
from mappoint import Mappoint
from gauss_newton import Gauss_newton

class Slidewindow_graph:
    def __init__(self):
        # 滑动窗口中的frame集合
        self._frames = []
        # 滑动窗口中mappoint集合，里面元素为字典(描述子：Mappoints类)
        self._mappoints = {}
        self._esti_pose = [[],[]]
        self._track = [[],[]]
        self._lastframe = Frame(0)
        self._coefficient = [[],[]]
    def Init_slidewindow_graph(self, measure_data, frame_id, init_pose):
        newFrame = Frame(frame_id)

        newFrame.set_pose(init_pose)

        for i in range(0, len(measure_data[0])):
            mp = np.array([[measure_data[0][i]], [measure_data[1][i]]])
            mp = np.dot(np.linalg.inv(newFrame._Rbm), mp) + newFrame._tb  
            newmappoint = Mappoint()
            newmappoint.set_descriptor(measure_data[2][i])
            newmappoint.set_pose(mp)
            newFrame.add_mappoint(newmappoint)
            newmappoint.add_frame(newFrame)
            self._mappoints[newmappoint._descriptor] = newmappoint
        self._frames.append(newFrame)
        self._lastframe = newFrame
        #print(self._lastframe._pose)

    def Update_slidewindow_graph(self, measure_data, frame_id, landmark):
        self._track = [[],[]]
        newFrame = Frame(frame_id)
        # 根据前后帧数据关联，求出当前帧位姿的初始估计
        n = 0
        i = 0
        b = np.array([0, 0, 0, 0])
        A = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [1, 0, 0, 0], [0, 1, 0, 0]])
        self._coefficient = [[],[]]

        while n != 5:
            #print(n)
            if measure_data[2][i] in self._lastframe._seeDescriptor:
                self._coefficient[1].append((self._mappoints[measure_data[2][i]])._pose[0][0])
                self._coefficient[1].append((self._mappoints[measure_data[2][i]])._pose[1][0])
                self._coefficient[0].append(measure_data[0][i])
                self._coefficient[0].append(measure_data[1][i])
                n = n + 1
                self._track[0].append(landmark[measure_data[2][i]][0])
                self._track[1].append(landmark[measure_data[2][i]][1])   
            i = i + 1
        #print(self._lastframe._pose)

        init_gs = np.array([[self._lastframe._pose[0][0]], [self._lastframe._pose[1][0]], [cos(self._lastframe._pose[2][0])], [sin(self._lastframe._pose[2][0])]])
        #print(self._lastframe._pose)
        GNsolve = Gauss_newton(self._coefficient, init_gs)
        # x = optimize.fsolve(self.get_initpose, [self._lastframe._pose[0][0], self._lastframe._pose[1][0], sin(self._lastframe._pose[2][0])])
        x = GNsolve.Solve()
        print(x[2][0])
        #x = [reasult[0][0],reasult[1][0], math.acos(reasult[2][0])]
        self._esti_pose[0].append(x[0][0])
        self._esti_pose[1].append(x[1][0])
        newFrame.set_pose(x)
        # 根据当前帧的位置，来估计新增加mappoint的初始位置；老的mappoints位置不变
        for i in range(0, len(measure_data[0])):
            if measure_data[2][i] in self._mappoints:
                newFrame.add_mappoint(self._mappoints[measure_data[2][i]])
                self._mappoints[measure_data[2][i]].add_frame(newFrame)
                continue
            else:
                mp = np.array([[measure_data[0][i]], [measure_data[1][i]]])
                #print(newFrame._Rbm)
                mp = np.dot(np.linalg.inv(newFrame._Rbm), mp) + newFrame._tb  
                newmappoint = Mappoint()
                newmappoint.set_descriptor(measure_data[2][i])
                newmappoint.set_pose(mp)
                newFrame.add_mappoint(newmappoint)
                newmappoint.add_frame(newFrame)
                self._mappoints[newmappoint._descriptor] = newmappoint
        #print(len(newFrame._seeDescriptor))
        self._frames.append(newFrame)
        self._lastframe = newFrame
    # def get_initpose(self, pose):
    #     t_x, t_y, alph = pose.tolist() 
    #     return [self._coefficient[0][0]*cos(alph) - self._coefficient[0][1]*sin(alph) + t_x - self._coefficient[1][0],
    #             self._coefficient[0][0]*sin(alph) + self._coefficient[0][1]*cos(alph) + t_y - self._coefficient[1][1],
    #             #self._coefficient[0][2]*cos(alph) - self._coefficient[0][3]*sin(alph) + t_x - self._coefficient[1][2],
    #             self._coefficient[0][2]*sin(alph) + self._coefficient[0][3]*cos(alph) + t_y - self._coefficient[1][3] ]

