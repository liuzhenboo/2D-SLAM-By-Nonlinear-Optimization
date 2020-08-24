# -*-coding: UTF-8 -*-
# create by liuzhenbo 2020/8/16 in nwpu

import numpy as np
import math
class Measure:
    def __init__(self, movemodel_class=None, landmarks = None ,r = None):
        # 滑动窗口内的观测
        self._pose_id = 0
        self._data = [[],[],[]]
        self._movemodel_class = movemodel_class
        self._r = r
        self._landmarks = landmarks


    def GetMeasure(self, n):
        # 更新pose id
        self._pose_id = n

        # 更新观测关系
        for i in range(0,np.size(self._landmarks._landmarks,1)):
            if (pow((self._movemodel_class._tb[0][0] - 1.0 * self._landmarks._landmarks[0][i]), 2.0) + pow((self._movemodel_class._tb[1][0] - 1.0 * self._landmarks._landmarks[1][i]), 2.0)) <= self._r * self._r and (pow((self._movemodel_class._tb[0][0] - 1.0 * self._landmarks._landmarks[0][i]), 2.0) + pow((self._movemodel_class._tb[1][0] - 1.0 * self._landmarks._landmarks[1][i]), 2.0)) >=0.01 :
                temp = np.array([[self._landmarks._landmarks[0][i]],[self._landmarks._landmarks[1][i]]])
                one_data = np.dot(self._movemodel_class._Rbm, (temp - self._movemodel_class._tb))
                del temp
                x = one_data[0][0] + (0.05)*np.random.normal(0,1)
                #print (alpha)
                y = one_data[1][0] + (0.05)*np.random.normal(0,1)
                self._data[0].append(x)
                self._data[1].append(y)
                self._data[2].append(self._landmarks._landmarks[2][i])
#+ np.random.normal(0,0.05)
    def xy2angle(self, x, y):
        l = math.sqrt(x * x + y * y)
        if y >= 0:
            return math.acos(x/l) + 2*math.pi
        elif x > 0:
            return math.asin(y/l) + 2*math.pi
        else:
            return -math.asin(y/l) + math.pi