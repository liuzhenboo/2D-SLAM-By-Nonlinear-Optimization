# -*-coding: UTF-8 -*-
# create by liuzhenbo 2020/8/16 in nwpu

import numpy as np
import math
from math import sin, cos

class Gauss_newton:
    def __init__(self, paramter, init_value):
        self._allow_error = 0.1
        self._current_state = init_value
        self._error = 0.0
        self._paramter = paramter
        self._H = np.zeros((3, 3), dtype=float)
        self._b = np.zeros((3, 1), dtype=float)
        
       
    def Linear(self):
        self._error = 0.0
        self._H = np.zeros((3, 3), dtype=float)
        self._b = np.zeros((3, 1), dtype=float)
        p_x = self._current_state[0][0]
        p_y = self._current_state[1][0]
        theta = self._current_state[2][0]

        for i in range(0, 5):
            l_x = self._paramter[1][2*i]
            l_y = self._paramter[1][2*i+1]
            z_1 = self._paramter[0][2*i]
            z_2 = self._paramter[0][2*i+1]
            J = np.array([[-cos(theta), -sin(theta), -(l_x - p_x) * sin(theta) + cos(theta) * (l_y - p_y)], [sin(theta), -cos(theta), -(l_x - p_x) * cos(theta) - sin(theta) * (l_y - p_y)]])
            e = np.array([[cos(theta)*(l_x-p_x)+sin(theta)*(l_y-p_y)-z_1],[-sin(theta)*(l_x-p_x)+cos(theta)*(l_y-p_y)-z_2]])
            self._H = np.dot(J.T, J) + self._H
            self._b = -np.dot(J.T, e) + self._b
            abs_error = np.dot(e.T, e)
            self._error = self._error + abs_error[0][0]
    
    def Solve(self):
        sum = 0
        while sum < 50:
            sum = sum + 1
            self.Linear()
            #print(np.dot(self.Jacobi().T, self.Jacobi()))
            delta = np.linalg.solve(self._H + 0.1 * np.identity(3), self._b)
            if self._error < self._allow_error:
                return self._current_state
            #print(self._error)
            self._current_state[0][0] = delta[0] + self._current_state[0][0]
            self._current_state[1][0] = delta[1] + self._current_state[1][0]
            self._current_state[2][0] = delta[2] + self._current_state[2][0]
        # print(self.Error())
        # print("迭代：")
        # print(sum)
        # print("次")
        return self._current_state

