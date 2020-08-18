# coding:utf-8
import numpy as np
from numpy import linspace
import matplotlib.pyplot as plt

# 自己的类
from movemodel import MoveModel
from draw import Draw
from landmark import Landmark
from measure import Measure
from frame import Frame
from mappoint import Mappoint
from slidewindow_graph import Slidewindow_graph
from five_point_tracking import Gauss_newton

init_pose = np.array([[12.0], [3.0], [0.0]])
estimate_init_pose = np.array([[12.0], [3.0], [0.0]])
move_model = MoveModel(init_pose)
landmarks = Landmark()
slidewindow_graph = Slidewindow_graph()
draw = Draw(landmarks, slidewindow_graph, move_model)

# 传感器参数
r = 3.0

# 循环计数
n = 0
sum = 500

# 主逻辑
#####################################################################################

while n != sum:

    measure = Measure(move_model, landmarks, r)
    measure.GetMeasure()

    if n == 0:
        slidewindow_graph.Init_slidewindow_graph(estimate_init_pose, measure)
    else:
        slidewindow_graph.Tracking(measure) 
    
    draw.Show_result(r)
    
    move_model.Updatepose()
    n = n + 1