#!/usr/bin/env python3
import os
import sys
sys.path.append(os.getcwd()+'/build')
sys.path.append(os.getcwd()+'/src')
import time
import AStarPython
import numpy as np
import matplotlib.pyplot as plt
from Simulator import Simulator


if __name__ == "__main__":
    # define the world
    map_width_meter = 60.0
    map_height_meter = 60.0
    map_resolution = 1
    value_non_obs = 0 # the cell is empty
    value_obs = 255 # the cell is blocked
    # create a simulator
    MySimulator = Simulator(map_width_meter, map_height_meter, map_resolution, value_non_obs, value_obs)
    # number of obstacles
    num_obs = 300
    # [width, length] size of each obstacle [meter]
    size_obs = [1, 1]
    # generate random obstacles
    MySimulator.generate_random_obs(num_obs, size_obs)
    # convert 2D numpy array to 1D list
    world_map = MySimulator.map_array.flatten().tolist()

    # define the start and goal
    # This is for an agent and a set of targets
    agent_position, targets_position = MySimulator.generate_agents_and_targets(num_agents=1, num_targets=300)
    # solve it
    t0 = time.time()
    path_many, distance_vec = AStarPython.FindPathAllMT(agent_position, targets_position, world_map, MySimulator.map_width, MySimulator.map_height)
    t1 = time.time()
    print("Time used for FindPathAllMT is [sec]: ", t1-t0)

    # print("These are all the paths:")
    # for i in range(0,len(path_many),1):
    #     print("This is a path. " + "Distance:" + str(distance_vec[i]))
    #     for j in range(0, len(path_many[i]), 2):
    #         str_print = str(path_many[i][j]) + ', ' + str(path_many[i][j+1])
    #         print(str_print)
    # # visualization
    # MySimulator.plot_many_path(path_many, agent_position, targets_position)


    #########################
    # solve it
    t0 = time.time()
    path_many_2, distance_vec_2 = AStarPython.FindPathAllMP(agent_position, targets_position, world_map, MySimulator.map_width, MySimulator.map_height)
    t1 = time.time()
    print("Time used for FindPathAllMP is [sec]: ", t1-t0)


    #########################
    # solve it
    t0 = time.time()
    path_many_3, distance_vec_3 = AStarPython.FindPathAll(agent_position, targets_position, world_map, MySimulator.map_width, MySimulator.map_height)
    t1 = time.time()
    print("Time used for FindPathAll is [sec]: ", t1-t0)

    # print("These are all the paths:")
    # for i in range(0,len(path_many),1):
    #     print("This is a path. " + "Distance:" + str(distance_vec[i]))
    #     for j in range(0, len(path_many[i]), 2):
    #         str_print = str(path_many[i][j]) + ', ' + str(path_many[i][j+1])
    #         print(str_print)
    # # visualization
    # MySimulator.plot_many_path(path_many, agent_position, targets_position)
    # plt.show()

    print("len(path_many)-len(path_many_2): ", len(path_many)-len(path_many_2))
    dist_diff_vec = np.array(distance_vec).flatten() - np.array(distance_vec_2).flatten()
    print("norm of distance difference MT VS MP: ", np.linalg.norm(dist_diff_vec))

    print("len(path_many)-len(path_many_3): ", len(path_many)-len(path_many_3))
    dist_diff_vec_3 = np.array(distance_vec).flatten() - np.array(distance_vec_3).flatten()
    print("norm of distance difference MT VS Original: ", np.linalg.norm(dist_diff_vec_3))
