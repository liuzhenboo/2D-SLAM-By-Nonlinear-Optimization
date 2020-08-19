import numpy as np
import math
from frame import Frame 
#b[0,0] = [1,1]
#print(b)
# b = np.zeros((2, 3))
# n = 0
# while n != 2:
#     a = np.array([[n], [2]])
#     b = np.append(b,a,axis=1)
#     n = n + 1
    
# print(b)

# a = {}
# a[2] = [2]
# a[3] = 3
# a[100] = 9
# if 10 in a:
#     print('s')
# print(a)
# A = np.array([[1, 2, 3], [2, -1, 1], [3, 0, -1],[1,2,4]])
# b = np.array([9, 8, 3,10])
# x = np.linalg.solve(A, b)
# print(x)
# from math import sin, cos 
# from scipy import optimize
# def f(x): 
#     x0, xl, x2 = x.tolist() 
#     return [
#             5*xl+3,
#             4*x0*x0 - 2*sin(xl*x2), 
#             xl*x2 - 1.5,
#             1
#             ]
# result = optimize.fsolve(f, [1,1,1])
# print (result)
# print (f(result))
# while True:
#     noise = np.random.normal(0.0, 0.1)
#     print(noise)
# A = np.array([[1], [2]])
# a = np.dot(A.T, A)
# print(a[0][0])
# a = np.array([[3], [1], [9]])
# b = np.array([[0], [0]])

# a[0:2,0] = b[0:2,0]
# b[0][0] = 100
# if not False:
#     print(a)
# a = Frame(0)
# b = Frame(2)
# c = Frame(3)
# d = [a, b, c]
# print(d)
# d.remove(b)
# print(d)
# print(b)
# f = set()
# f.add(a)
# f.add(b)
# f.add(c)
# print(f)
# f.remove(b)
# print(f)
# print(b)
c = np.array([[1],[2]])
print(len(c))