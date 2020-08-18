# coding:utf-8
import numpy as np

class Frame:
    def __init__(self, id):
        self._id = id
        self._pose = np.array([[0.0], [0.0], [0.0]])
        self._Rbm = np.array([[1.0, 0.0], [0.0, 1.0]])
        self._tb = np.array([[0.0], [0.0]])
        self._seeMappints = set()
        self._seeDescriptor = set()
    def set_pose(self, pose):
        self._pose = pose
        self._Rbm = np.array([[np.cos(pose[2,0]), np.sin(pose[2,0])],
                             [-np.sin(pose[2,0]), np.cos(pose[2,0])]])  # 2*2
        self._tb = np.array([[pose[0, 0]], [pose[1, 0]]])  # 2*1
    
    def add_mappoint(self, point):
        self._seeMappints.add(point)
        self._seeDescriptor.add(point._descriptor)

    

