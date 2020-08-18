# coding:utf-8

import numpy as np
from math import sin, cos
import math
from scipy import optimize
from measure import Measure
from frame import Frame
from mappoint import Mappoint
from five_point_tracking import Gauss_newton
from movemodel import MoveModel
class Slidewindow_graph:
    def __init__(self):
        self._max_window = 8
        # 滑动窗口中的frame集合
        self._frames = []
        # 滑动窗口中mappoint集合，里面元素为字典(描述子：Mappoints类)
        self._mappoints = {}
        self._esti_pose = [[],[]]
        self._f2ftrack = []
        self._lastframe = Frame(0)
        self._coefficient = [[], []]

        self._measure = Measure()
    def Init_slidewindow_graph(self, init_pose, measure):
        self._measure = measure
        newFrame = Frame(self._measure._pose_id)

        newFrame.set_pose(init_pose)

        for i in range(0, len(self._measure._data[0])):
            mp = np.array([[self._measure._data[0][i]], [self._measure._data[1][i]]])
            mp = np.dot(np.linalg.inv(newFrame._Rbm), mp) + newFrame._tb  
            newmappoint = Mappoint()
            newmappoint.set_descriptor(self._measure._data[2][i])
            newmappoint.set_pose(mp)
            newFrame.add_mappoint(newmappoint)
            newmappoint.add_frame(newFrame)
            self._mappoints[newmappoint._descriptor] = newmappoint
        self._frames.append(newFrame)
        self._lastframe = newFrame

    def Tracking(self, measure):
        self._measure = measure
        self.Fivepoint_f2f_track()
        self.Optimize_graph()

    def Fivepoint_f2f_track(self):
        self._f2ftrack = [[],[]]
        newFrame = Frame(self._measure._pose_id)
        # 根据前后帧数据关联，求出当前帧位姿的初始估计
        n = 0
        i = 0
        b = np.array([0, 0, 0, 0])
        A = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [1, 0, 0, 0], [0, 1, 0, 0]])
        self._coefficient = [[],[]]

        while n != 5:
            if self._measure._data[2][i] in self._lastframe._seeDescriptor:
                self._coefficient[1].append((self._mappoints[self._measure._data[2][i]])._pose[0][0])
                self._coefficient[1].append((self._mappoints[self._measure._data[2][i]])._pose[1][0])
                self._coefficient[0].append(self._measure._data[0][i])
                self._coefficient[0].append(self._measure._data[1][i])
                n = n + 1
                self._f2ftrack.append(self._measure._data[2][i])
            i = i + 1

        init_gs = np.array([[self._lastframe._pose[0][0]], [self._lastframe._pose[1][0]], [cos(self._lastframe._pose[2][0])], [sin(self._lastframe._pose[2][0])]])
        GNsolve = Gauss_newton(self._coefficient, init_gs)
        x = GNsolve.Solve()
        self._esti_pose[0].append(x[0][0])
        self._esti_pose[1].append(x[1][0])
        newFrame.set_pose(x)
        # 根据当前帧的位置，来估计新增加mappoint的初始位置；老的mappoints位置不变
        for i in range(0, len(self._measure._data[0])):
            if self._measure._data[2][i] in self._mappoints:
                newFrame.add_mappoint(self._mappoints[self._measure._data[2][i]])
                self._mappoints[self._measure._data[2][i]].add_frame(newFrame)
                continue
            else:
                mp = np.array([[self._measure._data[0][i]], [self._measure._data[1][i]]])
                #print(newFrame._Rbm)
                mp = np.dot(np.linalg.inv(newFrame._Rbm), mp) + newFrame._tb  
                newmappoint = Mappoint()
                newmappoint.set_descriptor(self._measure._data[2][i])
                newmappoint.set_pose(mp)
                newFrame.add_mappoint(newmappoint)
                newmappoint.add_frame(newFrame)
                self._mappoints[newmappoint._descriptor] = newmappoint
        #print(len(newFrame._seeDescriptor))
        self._frames.append(newFrame)
        self._lastframe = newFrame

    def Optimize_graph(self):
        pass
