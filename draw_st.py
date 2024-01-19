#!/usr/bin/python3
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np

def read_graph(filename):
    with open(filename, 'r') as file:
        lines = file.readlines()

    s_voxels_num, t_voxels_num = map(int, lines[0].split()[:2])
    time_resolution, distance_resolution = map(float, lines[0].split()[2:])
    points = [list(map(int, line.split())) for line in lines[1:]]

    return s_voxels_num, t_voxels_num, time_resolution, distance_resolution, points

def plot_graph(filename):
    s_voxels_num, t_voxels_num, time_res, dist_res, points = read_graph(filename)
    
    fig, ax = plt.subplots()
    for x, y, id in points:
        if(id==1):
            rect = patches.Rectangle((x - 1, y - 1), 1, 1, linewidth=1, edgecolor='blue', facecolor='blue')
            ax.add_patch(rect)
        else:
            rect = patches.Rectangle((x - 1, y - 1), 1, 1, linewidth=1, edgecolor='green', facecolor='green')
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

plot_graph('s_t_graph.txt')
