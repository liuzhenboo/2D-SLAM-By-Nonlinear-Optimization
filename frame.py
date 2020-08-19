# coding:utf-8
# create by liuzhenbo 2020/8/16 in nwpu

import numpy as np

class Frame:
    def __init__(self, id):
        self._id = id
        self._pose = np.array([[0.0], [0.0], [0.0]])
        self._Rbm = np.array([[1.0, 0.0], [0.0, 1.0]])
        self._tb = np.array([[0.0], [0.0]])
        self._seeMappints = []
        self._seeDescriptor = set()
        self._measure = []
    def set_pose(self, pose):
        self._pose = pose
        self._Rbm = np.array([[np.cos(pose[2,0]), np.sin(pose[2,0])],
                             [-np.sin(pose[2,0]), np.cos(pose[2,0])]])  # 2*2
        self._tb = np.array([[pose[0, 0]], [pose[1, 0]]])  # 2*1
    
    def add_mappoint(self, point):
        self._seeMappints.append(point)
        self._seeDescriptor.add(point._descriptor)
    def add_measure(self, measure):
        self._measure.append(measure)


