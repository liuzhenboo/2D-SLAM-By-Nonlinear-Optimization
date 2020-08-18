# -*-coding: UTF-8 -*-
# create by liuzhenbo 2020/8/16 in nwpu

import numpy as np

class MoveModel:
    def __init__(self, init_pose): ## x,y,角度
        self._currentpose = init_pose  # 3*1
        self._Rbm = np.array([[np.cos(init_pose[2,0]), np.sin(init_pose[2,0])],
                             [-np.sin(init_pose[2,0]), np.cos(init_pose[2,0])]])  # 2*2
        self._tb = np.array([[init_pose[0, 0]], [init_pose[1, 0]]])  # 2*1
        self._delta_xy = np.array([[0.2],[0.1]])
        self._delta_jiaodu = 0.03
    def Updatepose(self):
        # update _currentpose
        delta_xy = np.dot(np.linalg.inv(self._Rbm) , (self._delta_xy))
        self._currentpose[0, 0] = delta_xy[0, 0] + self._currentpose[0, 0]
        self._currentpose[1, 0] = delta_xy[1, 0] + self._currentpose[1, 0]
        self._currentpose[2, 0] = self._delta_jiaodu + self._currentpose[2, 0]
    
        # update others
        (self._tb)[0, 0] = (self._currentpose)[0, 0]
        (self._tb)[1, 0] = (self._currentpose)[1, 0]

        (self._Rbm)[0, 0] = np.cos(self._currentpose[2, 0])
        (self._Rbm)[0, 1] = np.sin(self._currentpose[2, 0])
        (self._Rbm)[1, 0] = -np.sin(self._currentpose[2, 0])
        (self._Rbm)[1, 1] = np.cos(self._currentpose[2, 0])
        #print(self._currentpose[2,0])

    