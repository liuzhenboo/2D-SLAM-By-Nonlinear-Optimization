# coding:utf-8
import numpy as np
import math

class Landmark:
    def __init__(self):
        self._landmarks = np.zeros((3, 441), dtype=int)
        self._landmarks_dict = {}
        for i in range(0, 21):
            for j in range(0, 21):
                (self._landmarks)[0, i * 21 + j] = i
                (self._landmarks)[1, i * 21 + j] = j
                (self._landmarks)[2, i * 21 + j] = i * 21 + j
                (self._landmarks_dict)[i*21 + j] = [i,j]
