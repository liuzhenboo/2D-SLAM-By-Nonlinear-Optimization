# -*-coding: UTF-8 -*-
import numpy as np

class Measure:
    def __init__(self, movemodel_class):
        # 滑动窗口内的观测
        self._pose_id = 0
        self._data = [[],[],[]]
        self._movemodel_class = movemodel_class

    def GetMeasure(self, landmarks, r):
        # 更新pose id
        self._pose_id = self._pose_id + 1

        # 更新观测关系
        for i in range(0,np.size(landmarks,1)):
            if (pow((self._movemodel_class._tb[0][0] - 1.0 * landmarks[0][i]), 2.0) + pow((self._movemodel_class._tb[1][0] - 1.0 * landmarks[1][i]), 2.0)) <= r * r:
                temp = np.array([[landmarks[0][i]],[landmarks[1][i]]])
                one_data = np.dot(self._movemodel_class._Rbm, (temp - self._movemodel_class._tb))
                del temp
                self._data[0].append(one_data[0, 0])
                self._data[1].append(one_data[1, 0])
                # data[0].append(landmarks[0, i])
                # data[1].append(landmarks[1, i])
                self._data[2].append(landmarks[2][i])  
