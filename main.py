# coding:utf-8
import numpy as np
from numpy import linspace
import matplotlib.pyplot as plt

# 自己的类
from movemodel import MoveModel
from draw import Draw
from landmark import Landmark
from measure import Measure
from solver import Solver
from frame import Frame
from mappoint import Mappoint
from slidewindow_graph import Slidewindow_graph

init_pose = np.array([[12.0], [3.0], [0.0]])
move_model = MoveModel(init_pose)
landmarks = Landmark()
draw = Draw()
solver = Solver()
slidewindow_graph =Slidewindow_graph()

# 循环计数
n = 0
sum = 1000

# 传感器参数
r = 3.0

while n != sum:
    measure = Measure(move_model)
    measure.GetMeasure(landmarks._landmarks, r)
    if n == 0:
        slidewindow_graph.Init_slidewindow_graph(measure._data, measure._pose_id, init_pose)
    else:
        slidewindow_graph.Update_slidewindow_graph(measure._data, measure._pose_id)

    draw.Show_Result(move_model._currentpose, r, landmarks._landmarks)
    move_model.updatepose()
    n = n + 1