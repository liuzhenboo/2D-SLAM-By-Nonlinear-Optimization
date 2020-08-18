# -*-coding: UTF-8 -*-
import numpy as np
import math
class Gauss_newton:
    def __init__(self, paramter, init_vale):
        self._allow_error = 0.001
        self._best_state = np.array([[0.0], [0.0], [0.0], [0.0]])
        self._best_state = init_vale
        self._want_state = np.array([[0.0],[0.0],[0.0]])
        
        self._A = np.array([[1.0,0.0,0,0],[0.0,1.0,0.0,0.0],[1.0,0.0,0.0,0.0],[0.0,1.0,0.0,0.0],[1.0,0.0,0.0,0.0],[0.0,1.0,0.0,0.0],[1.0,0.0,0.0,0.0],[0.0,1.0,0.0,0.0],[1.0,0.0,0.0,0.0],[0.0,1.0,0.0,0.0]])
        self._b = np.array([[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0]])
        
        self._A[0][2] = paramter[0][0]
        self._A[0][3] = -paramter[0][1]
        self._A[1][2] = paramter[0][1]
        self._A[1][3] = paramter[0][0]
        self._b[0][0] = paramter[1][0]
        self._b[1][0] = paramter[1][1]


        self._A[2][2] = paramter[0][2]
        self._A[2][3] = -paramter[0][3]
        self._A[3][2] = paramter[0][3]
        self._A[3][3] = paramter[0][2]
        self._b[2][0] = paramter[1][2]
        self._b[3][0] = paramter[1][3]


        self._A[4][2] = paramter[0][4]
        self._A[4][3] = -paramter[0][5]
        self._A[5][2] = paramter[0][5]
        self._A[5][3] = paramter[0][4]
        self._b[4][0] = paramter[1][4]
        self._b[5][0] = paramter[1][5]

        self._A[6][2] = paramter[0][6]
        self._A[6][3] = -paramter[0][7]
        self._A[7][2] = paramter[0][7]
        self._A[7][3] = paramter[0][6]
        self._b[6][0] = paramter[1][6]
        self._b[7][0] = paramter[1][7]

        self._A[8][2] = paramter[0][8]
        self._A[8][3] = -paramter[0][9]
        self._A[9][2] = paramter[0][9]
        self._A[9][3] = paramter[0][8]
        self._b[8][0] = paramter[1][8]
        self._b[9][0] = paramter[1][9]
        #print(self._A)
        self._error_state = np.dot(self._A, self._best_state) - self._b


    def Jacobi(self):
        return (self._A)

    def Error(self):
        c = np.dot(self._error_state.T, self._error_state)
        return c[0][0]

    def Solve(self):
        sum = 0
        while self.Error() > self._allow_error and sum < 1000:
            sum = sum + 1
            #print(np.dot(self.Jacobi().T, self.Jacobi()))
            delta = np.linalg.solve(np.dot(self.Jacobi().T, self.Jacobi()) + 0.0 * np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]), -np.dot(self.Jacobi().T, self._error_state))
            #print(delta)
            self._best_state[0][0] = delta[0] + self._best_state[0][0]
            self._best_state[1][0] = delta[1] + self._best_state[1][0]
            self._best_state[2][0] = delta[2] + self._best_state[2][0]
            self._best_state[3][0] = delta[3] + self._best_state[3][0]
            self._error_state = np.dot(self._A, self._best_state) - self._b
        # print(self.Error())
        # print("迭代：")
        # print(sum)
        # print("次")
        if self._best_state[2][0] < -1.0:
            self._best_state[2][0] = -1.0
        if self._best_state[2][0] > 1.0:
            self._best_state[2][0] = 1.0

        if self._best_state[3][0] < -1.0:
            self._best_state[3][0] = -1.0
        if self._best_state[3][0] > 1.0:
            self._best_state[3][0] = 1.0
        
        if self._best_state[2][0] > 0.0 and self._best_state[3][0] > 0.0:
            self._want_state[2][0] = math.acos(self._best_state[2][0])
        elif self._best_state[2][0] < 0.0 and self._best_state[3][0] > 0.0:
            self._want_state[2][0] = math.acos(self._best_state[2][0])
        elif self._best_state[2][0] < 0.0 and self._best_state[3][0] < 0.0:
            self._want_state[2][0] = math.pi - math.asin(self._best_state[3][0])
        elif self._best_state[2][0] > 0.0 and self._best_state[3][0] < 0.0:
            self._want_state[2][0] = 2.0*math.pi + math.asin(self._best_state[3][0])
        self._want_state[0][0] = self._best_state[0][0]
        self._want_state[1][0] = self._best_state[1][0]
        return self._want_state

