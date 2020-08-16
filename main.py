# coding:utf-8
import numpy as np
from numpy import linspace
import matplotlib.pyplot as plt

from movemodel import MoveModel
from draw import Draw
from landmark import Landmark



# 运动模型
init_pose = np.array([[12.0], [3.0], [0.0]])
move_model = MoveModel(init_pose)
# landmark
landmarks = Landmark()
# 绘图
draw = Draw()

# 循环计数
n = 0

# 传感器参数
r = 3

while n != 1000:
    draw.Show_Result(move_model._currentpose, r, landmarks._landmarks)
    move_model.updatepose()
    n = n + 1