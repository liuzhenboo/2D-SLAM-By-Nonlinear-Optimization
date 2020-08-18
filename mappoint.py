# coding:utf-8
import numpy as np

class Mappoint:
    def __init__(self):
        self._descriptor = 0
        self._pose = np.array([[0], [0]])
        self._seeFrames = []
    def set_pose(self, pose):
        self._pose = pose
    def set_descriptor(self, descriptor):
        self._descriptor = descriptor
    
    def add_frame(self, frame):
        self._seeFrames.append(frame)

    
