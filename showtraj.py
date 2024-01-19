#!/usr/bin/python3
import matplotlib.pyplot as plt
import pandas as pd

# 绘图设置
plt.figure(figsize=(10, 6))

# 循环读取并绘制每条轨迹
for i in range(1, 4):
    file_name = f'trajectory{i}.csv'
    data = pd.read_csv(file_name, names=['x', 'y', 'yaw'])
    plt.plot(data['x'], data['y'], label=f'Trajectory {i}')

# 添加图例
plt.legend()

# 添加轴标签和标题
plt.xlabel('X Position')
plt.ylabel('Y Position')
plt.title('Vehicle Trajectories')

# 显示图形
plt.show()

