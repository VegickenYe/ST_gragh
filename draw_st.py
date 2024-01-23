#!/usr/bin/python3
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np

def read_file(filename):
    vectors = []
    with open(filename, 'r') as file:
        for line in file:
            x, y = map(int, line.split())
            vectors.append((x, y))
    return vectors

def read_graph(filename):
    with open(filename, 'r') as file:
        lines = file.readlines()

    s_voxels_num, t_voxels_num = map(int, lines[0].split()[:2])
    time_resolution, distance_resolution = map(float, lines[0].split()[2:])
    points = [list(map(int, line.split())) for line in lines[1:]]

    return s_voxels_num, t_voxels_num, time_resolution, distance_resolution, points

def plot_graph(filename1,filename2):
    s_voxels_num, t_voxels_num, time_res, dist_res, points = read_graph(filename1)
    vectors=read_file(filename2)

    fig, ax = plt.subplots()

    # 设置图表标题
    plt.title(f"firgue - {filename1} & {filename2}")
    for x, y, id in points:
        if(id==0):
            rect = patches.Rectangle((x - 1, y - 1), 1, 1, linewidth=1, edgecolor='blue', facecolor='blue')
            ax.add_patch(rect)
        elif(id==1):
            rect = patches.Rectangle((x - 1, y - 1), 1, 1, linewidth=1, edgecolor='green', facecolor='green')
            ax.add_patch(rect)
        elif(id==2):
            rect = patches.Rectangle((x - 1, y - 1), 1, 1, linewidth=1, edgecolor='yellow', facecolor='yellow')
            ax.add_patch(rect)

    for y,x in vectors:
            rect = patches.Rectangle((x - 1, y - 1), 1, 1, linewidth=1, edgecolor='black', facecolor='black')
            ax.add_patch(rect)
    # for x, y, _ in points:
    #     circle = patches.Circle((x, y), 1, color='blue')
    #     ax.add_patch(circle)

    plt.xlim(0, t_voxels_num )
    plt.ylim(0, s_voxels_num )

    # plt.xticks(np.arange(0, t_voxels_num * time_res, time_res))
    # plt.yticks(np.arange(0, s_voxels_num * dist_res, dist_res))

    # plt.grid(True)
    plt.show()

def read_key():
    while True:
        key = input("请输入一个键(0,1,2):")
        if key in ['0', '1', '2']:
            return key
        else:
            print("无效输入,请重新输入0,1或2。")

i = read_key()
print(f"您输入的键是：{i}")
current_filename1 = f"s_t_graph{i}.txt"
current_filename2 = f"Find_s_t{i}.txt"
plot_graph(current_filename1,current_filename2)