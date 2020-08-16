# coding:utf-8
import numpy as np
import math

class Landmark:
    def __init__(self):
        self._landmarks = np.zeros((2, 441))
        for i in range(0, 21):
            for j in range(0, 21):
                (self._landmarks)[0, i*21 + j] = i
                (self._landmarks)[1,i*21 + j] = j
