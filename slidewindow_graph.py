# coding:utf-8

import numpy as np

from frame import Frame
from mappoint import Mappoint

class Slidewindow_graph:
    def __init__(self):
        # 滑动窗口中的frame集合
        self._frames = []
        # 滑动窗口中mappoint集合，里面元素为字典(描述子：Mappoints类)
        self._mappoints = {}

        self._lastframe = Frame(0)
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

    def Update_slidewindow_graph(self,measure_data, frame_id):
        newFrame = Frame(frame_id)
        n = 0
        i = 0
        b = np.array([0, 0, 0, 0])
        A = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [1, 0, 0, 0], [0, 1, 0, 0]])
        while n != 2:
            if measure_data[2][i] in self._lastframe._seeMappints:
                n = n + 1


