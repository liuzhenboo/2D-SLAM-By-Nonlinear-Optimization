# 2D-SLAM-By-Nonlinear-Optimization

## Features

非线性优化，LM迭代优化，滑动窗口，边缘化，FEJ

## Report

技术文档：
[Reports](./report)

## Reasults

### 只使用前端
注释掉slidewindow_graph.py中函数def Update(self, measure):里的：
    
    self.Optimize_graph()

![picture0](https://github.com/liuzhenboo/2D-SLAM-By-Nonlinear-Optimization/raw/master/pictures/onlyfrontend.png)

### 滑动窗口优化（1）

滑窗之外的观测直接舍去，不使用先验信息。
注释掉注释掉slidewindow_graph.py中函数def Optimize_graph(self):里的：

    self.Get_prior()

![picture1](https://github.com/liuzhenboo/2D-SLAM-By-Nonlinear-Optimization/raw/master/pictures/nomrg.png)

### 滑动窗口优化（2）

滑窗之外的观测信息不直接舍去，利用舒尔补转换成约束矩阵，形成先验信息，在优化中使用。这里使用的边缘化方案，可以保证FEJ。

![picture2](https://github.com/liuzhenboo/2D-SLAM-By-Nonlinear-Optimization/raw/master/pictures/fej-marge.png)
