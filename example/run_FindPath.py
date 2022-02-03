#!/usr/bin/env python3
import os
import sys
sys.path.append(os.getcwd()+'/build')
sys.path.append(os.getcwd()+'/src')
import time
import math
import AStarPython
import matplotlib.pyplot as plt
from Simulator import Simulator


if __name__ == "__main__":
    # define the world
    map_width_meter = 20.0
    map_height_meter = 20.0
    map_resolution = 2
    value_non_obs = 0 # the cell is empty
    value_obs = 255 # the cell is blocked
    # create a simulator
    MySimulator = Simulator(map_width_meter, map_height_meter, map_resolution, value_non_obs, value_obs)
    # number of obstacles
    num_obs = 150
    # [width, length] size of each obstacle [meter]
    size_obs = [1, 1]
    # generate random obstacles
    MySimulator.generate_random_obs(num_obs, size_obs)
    # convert 2D numpy array to 1D list
    world_map = MySimulator.map_array.flatten().tolist()

    # define the start and goal
    agent_position, targets_position = MySimulator.generate_agents_and_targets(num_agents=1, num_targets=1)
    # solve it
    t0 = time.time()
    path, distance = AStarPython.FindPath(agent_position, targets_position, world_map, MySimulator.map_width, MySimulator.map_height)
    t1 = time.time()
    print("Time used for a single path is [sec]: ", t1-t0)

    distance = 0.0
    for idx in range(0, int(round(len(path)/2)) - 1):
        distance += math.sqrt((path[2*idx]-path[2*idx+2])**2 + (path[2*idx+1]-path[2*idx+3])**2)
    print("Distance computed in python: ", distance)

    # visualization (uncomment next line if you want to visualize a single path)
    MySimulator.plot_single_path(path)
    plt.show()
