# coding:utf-8
# create by liuzhenbo 2020/8/16 in nwpu
import time
import sys
import os
import numpy as np
np.set_printoptions(threshold=np.inf)

from math import sin, cos
import math
from scipy import optimize
from measure import Measure
from frame import Frame
from mappoint import Mappoint
from five_point_tracking import Gauss_newton
from movemodel import MoveModel
class Slidewindow_graph:
    def __init__(self):
        self._max_window = 20
        # 滑动窗口中的frame集合
        self._frames_DB = []
        # 滑动窗口中mappoint集合，里面元素为字典(描述子->Mappoints类)
        self._mappoints_DB = {}
        self._state = np.array([])
        self._descriptor2state = {}
        self._frameid2state = {}
        self._jacobi = np.array([])
        self._error = np.array([])
        self._measure = Measure()

        self._prior_matrix = np.array([])
        self._prior_matrixb = np.array([])
        self._lastframe = Frame(0)
        self._coefficient = [[], []]
        self._measure_count = 0
        # draw
        self._esti_pose = [[],[]]
        self._f2ftrack = [[],[]]
        self._slideframes = [[], []]
        self._slidepoints = [[],[]]
    def Initialize(self, init_pose, measure):
        self._measure = measure
        newFrame = Frame(self._measure._pose_id)

        # 初始化第一帧位姿
        newFrame.set_pose(init_pose)
        # 初始化地图点位置
        for i in range(0, len(self._measure._data[0])):
            x_local = self._measure._data[1][i] * math.cos(self._measure._data[0][i])
            y_local = self._measure._data[1][i] * math.sin(self._measure._data[0][i])
            raw_measure = np.array([[x_local], [y_local]])
            raw_measure0 = np.array([[self._measure._data[0][i]],[self._measure._data[1][i]]])

            mp_pose = np.dot(np.linalg.inv(newFrame._Rbm), raw_measure) + newFrame._tb  
            newmappoint = Mappoint()
            newmappoint.set_descriptor(self._measure._data[2][i])
            newmappoint.set_pose(mp_pose)
            newmappoint.add_frame(newFrame)

            newFrame.add_mappoint(newmappoint)
            newFrame.add_newmappoints(newmappoint)
            newFrame.add_measure(raw_measure0, newmappoint._descriptor)
            self._mappoints_DB[newmappoint._descriptor] = newmappoint
        self._frames_DB.append(newFrame)
        self._lastframe = newFrame

    def Update(self, measure):
        # （1）更新新的观测
        self._measure = measure
        # （2）前端跟踪：通过F2F跟踪五个点，初始估计新的状态；并将新的状态加入图
        self.Fivepoint_f2f_track()
        # （3）后端优化：利用滑窗内所有信息优化图
        self.Optimize_graph()
        # （4）保存信息，用于做图
        self.For_draw()

    def Fivepoint_f2f_track(self):
        self._f2ftrack = [[],[]]
        newFrame = Frame(self._measure._pose_id)
        # 根据前后帧数据关联，求出当前帧位姿的初始估计
        n = 0
        i = 0
        self._coefficient = [[],[]]

        while n != 5:
            if self._measure._data[2][i] in self._lastframe._seeDescriptor:
                self._coefficient[1].append((self._mappoints_DB[self._measure._data[2][i]])._pose[0][0])
                self._coefficient[1].append((self._mappoints_DB[self._measure._data[2][i]])._pose[1][0])
                self._coefficient[0].append(self._measure._data[1][i] * cos(self._measure._data[0][i]))
                self._coefficient[0].append(self._measure._data[1][i] * sin(self._measure._data[0][i]))
                n = n + 1
                self._f2ftrack.append(self._measure._data[2][i])
            i = i + 1

        init_gs = np.array([[self._lastframe._pose[0][0]], [self._lastframe._pose[1][0]], [cos(self._lastframe._pose[2][0])], [sin(self._lastframe._pose[2][0])]])
        # 高斯牛顿法求解
        GNsolve = Gauss_newton(self._coefficient, init_gs)
        x = GNsolve.Solve()
        newFrame.set_pose(x)
        # 根据当前帧的位置，来估计新增加mappoint的初始位置；老的mappoints位置不变
        for i in range(0, len(self._measure._data[0])):
            x_local = self._measure._data[1][i] * math.cos(self._measure._data[0][i])
            y_local = self._measure._data[1][i] * math.sin(self._measure._data[0][i]) 
            raw_measure = np.array([[x_local], [y_local]])
            raw_measure0 = np.array([[self._measure._data[0][i]],[self._measure._data[1][i]]])
            if self._measure._data[2][i] in self._mappoints_DB:
                newFrame.add_mappoint(self._mappoints_DB[self._measure._data[2][i]])
                newFrame.add_measure(raw_measure0, self._measure._data[2][i])
                self._mappoints_DB[self._measure._data[2][i]].add_frame(newFrame)
                continue
            else:
                pose = np.dot(np.linalg.inv(newFrame._Rbm), raw_measure) + newFrame._tb  
                newmappoint = Mappoint()
                newmappoint.set_descriptor(self._measure._data[2][i])
                newmappoint.set_pose(pose)
                newmappoint.add_frame(newFrame)

                newFrame.add_mappoint(newmappoint)
                newFrame.add_newmappoints(newmappoint)
                newFrame.add_measure(raw_measure0, newmappoint._descriptor)
                self._mappoints_DB[newmappoint._descriptor] = newmappoint
        self._frames_DB.append(newFrame)
        self._lastframe = newFrame
       
    def Assemble_state(self):
        self._state = np.array([])
        self._descriptor2state = {}
        self._frameid2state = {}
        self._measure_count = 0
        dim = 3*len(self._frames_DB) + 2*len(self._mappoints_DB)
        self._state.resize(dim, 1)
        
        index = 0 
        for i in range(0, len(self._frames_DB)):
            # 装配位姿向量(3*1)
            self._state[index:(index + 3), 0] = self._frames_DB[i]._pose[0:3, 0]
            #print(self._frames_DB[i]._pose[0:3, 0])
            self._frameid2state[self._frames_DB[i]._id] = index
            index = index + 3
            self._measure_count = self._measure_count + len(self._frames_DB[i]._seeMappints)

            # 装配地图点向量(2*1)
            for j in range(0, len(self._frames_DB[i]._new_mappoint_state)):
                if not self._frames_DB[i]._new_mappoint_state[j]._descriptor in self._descriptor2state:
                    self._state[index:(index + 2), 0] = self._frames_DB[i]._new_mappoint_state[j]._pose[0:2, 0]
                    self._descriptor2state[self._frames_DB[i]._new_mappoint_state[j]._descriptor] = index
                    index = index + 2

#        print(self._state)
    def Assemble_jacobi(self):
        self._jacobi = np.array([])
        self._error = np.array([])
        self._jacobi.resize(2 * self._measure_count, len(self._state))
        self._error.resize(2 * self._measure_count, 1)

        measure_index = 0
        for i in range(0, len(self._frames_DB)):
            for j in range(0, len(self._frames_DB[i]._seeMappints)):
                point_index = self._descriptor2state[self._frames_DB[i]._seeMappints[j]._descriptor]
                frame_index = self._frameid2state[self._frames_DB[i]._id]
                #print(frame_index)
                x_f = self._state[frame_index][0]
                y_f = self._state[frame_index + 1][0]
                theta = self._state[frame_index + 2][0]
                x_p = self._state[point_index][0]
                y_p = self._state[point_index + 1][0]
                #print(x_f)
                measure = self._frames_DB[i]._measure[self._frames_DB[i]._seeMappints[j]._descriptor]
                # 单一残差项对frame的2*3雅克比矩阵
                self._jacobi[measure_index][frame_index] = -cos(theta)
                #print(-cos(theta))
                self._jacobi[measure_index][frame_index+1] = -sin(theta)
                self._jacobi[measure_index][frame_index+2] = (x_f-x_p)*sin(theta)+(y_p-y_f)*cos(theta)
                self._jacobi[measure_index + 1][frame_index] = sin(theta)
                self._jacobi[measure_index+1][frame_index+1] = -cos(theta)
                self._jacobi[measure_index+1][frame_index+2] = (x_f-x_p)*cos(theta)+(y_f-y_p)*sin(theta)
                # 单一残差项对mappoint的2*2雅克比矩阵
                self._jacobi[measure_index][point_index] = cos(theta)
                self._jacobi[measure_index][point_index+1] = sin(theta)
                self._jacobi[measure_index+1][point_index] = -sin(theta)
                self._jacobi[measure_index+1][point_index+1] = cos(theta)

                # 残差向量
                self._error[measure_index][0] = (x_p-x_f)*cos(theta)+(y_p-y_f)*sin(theta) - measure[1][0]*cos(measure[0][0])
                self._error[measure_index+1][0] = (x_f-x_p)*sin(theta)+(y_p-y_f)*cos(theta) - measure[1][0] * sin(measure[0][0])

                measure_index = measure_index + 2
        #f = open("./a.txt", 'w+')
        #print(self._jacobi)
        #print >> f, self._jacobi

    def Get_prior(self):
        dim = len(self._state)
        dim1 = 2 * len(self._frames_DB[0]._new_mappoint_state) + 3
        # debug
        # if len(self._frames_DB[0]._new_mappoint_state) == len(self._frames_DB[0]._seeDescriptor):
        #     print('ok')
        dim2 = dim - dim1
        measure_dim = 2 * len(self._frames_DB[0]._new_mappoint_state)

        J_old = self._jacobi[0:measure_dim, 0:dim]
        error_old = self._error[0:measure_dim, 0:1]
        H = np.dot(J_old.T, J_old) + 0.01 * np.identity(dim)
        b = -np.dot(J_old.T, error_old)
        # f = open("./a.txt", 'w+')
        H21 = H[dim1:dim, 0:dim1]
        # print(H21)
        # print >> f, H21

        H11 = H[0:dim1, 0:dim1]
        H12 = H[0:dim1, dim1:dim]
        self._prior_matrix.resize(dim2, dim2)
        self._prior_matrix = np.dot(np.dot(H21, np.linalg.inv(H11)), H12)
        #print(self._prior_matrix)

        self._prior_matrixb.resize(dim2, 1)
        b_old = b[0:dim1, 0]
        self._prior_matrixb = np.dot(np.dot(H21, np.linalg.inv(H11)),b_old)
        #print("维度一致")    

    def Linearization(self):
        self.Assemble_state()
        self.Assemble_jacobi()
    
    # 高斯牛顿迭代
    def Iterative_optimize(self):
        sum = 0
        #print(np.dot(self._error.T, self._error)[0][0])
        if len(self._prior_matrix) != 0:
            temp0 = np.zeros((len(self._state), len(self._state)))
            temp1 = np.zeros((len(self._state), 1))
            dim = len(self._state) - 2*len(self._lastframe._new_mappoint_state) - 3
            temp0[0:dim, 0:dim] = self._prior_matrix
            # debug
            # if len(self._prior_matrix) + 2 * len(self._lastframe._new_mappoint_state) + 3 == len(self._state):
            #     print('ok')
            # else:
            #     print('wrong')
            # if dim == len(self._prior_matrix):
            #     print("ok!")
            # else:
            #     print("wrong!")
            temp1[0:dim, 0] = self._prior_matrixb
            self._prior_matrix = temp0
            self._prior_matrixb = temp1
            #print(self._prior_matrix)
            #print("维度对着呢")
        while np.dot(self._error.T, self._error)[0][0] > -0.001 and sum < 10:
            #print(self._jacobi)
            if len(self._prior_matrix) == 0:
                print("未使用先验！")

                H = np.dot(self._jacobi.T, self._jacobi) + 0.01 * np.identity(len(self._state))
                b = -np.dot(self._jacobi.T, self._error)
            else:
                H = np.dot(self._jacobi.T, self._jacobi) + 0.01 * np.identity(len(self._state)) - self._prior_matrix
                
                b = -np.dot(self._jacobi.T, self._error) - self._prior_matrixb
                print("使用先验！")
         
            delta = np.linalg.solve(H, b)
            #print(delta)
            #exit()
            # 更新线性化点
            self._state = delta + self._state
            # 更新雅克比
            self.Assemble_jacobi()
            # 更新残差向量
            sum = sum + 1
        #print(len(self._state))
        #exit()

    def Cut_window(self):
        #print('cut_window!')

        for i in range(0, len(self._frames_DB[0]._seeMappints)):
            mappoint0 = self._frames_DB[0]._seeMappints[0]
            del self._mappoints_DB[mappoint0._descriptor]
            self._measure_count = self._measure_count - len(mappoint0._seeFrames)
            for j in range(0, len(mappoint0._seeFrames)):
                # print(len(mappoint0._seeFrames))
                # print(self._measure_count - len(mappoint0._seeFrames))
                # # print(len(mappoint0._seeFrames))
                # print(mappoint0._seeFrames[0])
                # print(j)
                frame0 = mappoint0._seeFrames[j]
                (frame0._seeMappints).remove(mappoint0)
                (frame0._seeDescriptor).remove(mappoint0._descriptor)
                del (frame0._measure)[mappoint0._descriptor]
                #del mappoint0
        self._frames_DB.remove(self._frames_DB[0])
            
             
    def Optimize_graph(self):
        #t1 = time.clock()
        self.Linearization()
        #t2 = time.clock()
        #print(t2 - t1)

        #t1 = time.clock()
        self.Iterative_optimize()
        #t2 = time.clock()
        #print(t2-t1)

        #t1 = time.clock()
        self.Flush_graph()
        #t2 = time.clock()
        #print(t2-t1)
        
        #t1 = time.clock()
        if len(self._frames_DB) > self._max_window:
            self.Get_prior()
            self.Cut_window()
        #t2 = time.clock()
        #print(t2-t1)

    def For_draw(self):
        self._esti_pose[0].append(self._lastframe._pose[0][0])
        self._esti_pose[1].append(self._lastframe._pose[1][0])
        self._slideframes = [[], []]
        self._slidepoints = [[], []]
        for i in range(0, len(self._frames_DB)):
            self._slideframes[0].append(self._frames_DB[i]._pose[0][0])
            self._slideframes[1].append(self._frames_DB[i]._pose[1][0])
            for j in range(0, len(self._frames_DB[i]._new_mappoint_state)):
                self._slidepoints[0].append(self._frames_DB[i]._new_mappoint_state[j]._pose[0][0])
                self._slidepoints[1].append(self._frames_DB[i]._new_mappoint_state[j]._pose[1][0])

    def Flush_graph(self):

        for i in range(0, len(self._frames_DB)):
            # 装配位姿向量(3*1)
            index = self._frameid2state[ self._frames_DB[i]._id]

            d = np.array([[0.0], [0.0], [0.0]])
            #print(index)
            d[0][0] = self._state[index, 0]
            d[1][0] = self._state[index+1, 0]
            d[2][0] = self._state[index+2, 0]
            #print(d)
            #print(self._state)
            #exit()
            self._frames_DB[i].set_pose(d)
            # 装配地图点向量(2*1)
            for j in range(0, len(self._frames_DB[i]._seeMappints)):
                temp_index = self._descriptor2state[self._frames_DB[i]._seeMappints[j]._descriptor] 
                self._frames_DB[i]._seeMappints[j]._pose[0:2, 0] = self._state[temp_index:(temp_index + 2), 0]

