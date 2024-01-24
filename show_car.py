import matplotlib.pyplot as plt
import pandas as pd
import matplotlib.patches as patches
import matplotlib.transforms as transforms
import time

# 读取 CSV 文件
def read_csv(filename):
    return pd.read_csv(filename)

# 绘制轨迹
def plot_trajectory(dfs):
    fig, ax = plt.subplots()
    plt.xlim(min([min(df['X']) for df in dfs]) - 1, max([max(df['X']) for df in dfs]) + 1)
    plt.ylim(min([min(df['Y']) for df in dfs]) - 1, max([max(df['Y']) for df in dfs]) + 1)

    # 初始化时间和索引
    current_time = 0
    indices = [0, 0, 0]

    while True:
        ax.clear()

        # 设置坐标轴
        plt.xlim(min([min(df['X']) for df in dfs]) - 1, max([max(df['X']) for df in dfs]) + 1)
        plt.ylim(min([min(df['Y']) for df in dfs]) - 1, max([max(df['Y']) for df in dfs]) + 1)

        for j, df in enumerate(dfs):
            # 更新位置，如果当前时间大于等于 timestamp
            while indices[j] < len(df) and current_time >= df['Timestamp'][indices[j]]:
                indices[j] += 1

            if indices[j] > 0:
                # 绘制到当前点的轨迹
                ax.plot(df['X'][:indices[j]], df['Y'][:indices[j]])

                # 绘制矩形（当前位置）
                current_pos = (df['X'][indices[j] - 1], df['Y'][indices[j] - 1])
                yaw = df['Yaw'][indices[j] - 1]
                rect = patches.Rectangle((current_pos[0]-0.12, current_pos[1]-0.09), 0.24, 0.18, fill=True, color='r', alpha=0.5)
                
                # 创建变换
                t = transforms.Affine2D().rotate_around(current_pos[0], current_pos[1], yaw) + ax.transData
                rect.set_transform(t)
                
                ax.add_patch(rect)

        plt.draw()
        plt.pause(0.05)

        # 增加当前时间
        current_time += 0.1  # 每次循环增加的时间

        # 检查是否所有轨迹都已完成
        if all(index >= len(df) for index, df in zip(indices, dfs)):
            break

    plt.show()

# 主函数
def main():
    filenames = ['newtrajectory0.csv', 'newtrajectory1.csv', 'newtrajectory2.csv']
    dfs = [read_csv(filename) for filename in filenames]
    plot_trajectory(dfs)

if __name__ == '__main__':
    main()
