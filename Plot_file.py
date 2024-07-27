import matplotlib.pyplot as plt

def read_pose_data(file_path):
    with open(file_path, 'r') as file:
        lines = file.readlines()
    x_coords = []
    y_coords = []
    for line in lines:
        x, y = map(float, line.split())
        x_coords.append(x)
        y_coords.append(y)
    return x_coords, y_coords

# 读取三个不同的txt文件
x_coords1, y_coords1 = read_pose_data('pose_data_opt1.txt')
x_coords2, y_coords2 = read_pose_data('pose_data_no1.txt')
x_coords3, y_coords3 = read_pose_data('pose_gt.txt')

# 设置绘图范围
plt.axis([2,20,1,16])

# 标记 x_coords3, y_coords3 的第一个点并标注 "start point"
plt.scatter(x_coords3[0], y_coords3[0], color='red', label='Start Point')  # 标记第一个点为红色，并在图例中添加标签
# plt.text(x_coords3[0], y_coords3[0], 'start point', fontsize=12, color='red', verticalalignment='bottom')  # 添加标签

# 绘制三个轨迹
plt.plot(x_coords1, y_coords1, marker='.', label='Without Observer')
plt.plot(x_coords2, y_coords2, marker='x', label='With Observer')
plt.plot(x_coords3, y_coords3, linestyle='--', color='gray', label='GroundTurth')

# 添加标题和标签
plt.title('Trajectories', fontsize=18)
plt.xlabel('X Coordinate', fontsize=16)
plt.ylabel('Y Coordinate', fontsize=16)

# 添加图例
plt.legend(fontsize=14)

# 显示绘图
plt.show()