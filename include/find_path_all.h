#ifndef FIND_PATH_ALL_H
#define FIND_PATH_ALL_H


#include <iostream>
#include <vector>
#include <tuple>
#include "MapInfo.h"
#include "find_path.h"
#include "get_combination.h"
#include "smooth_path.h"


inline std::tuple<std::vector<std::vector<int>>, std::vector<float>> find_path_all(
    const int *agent_position,
    const std::vector<int> targets_position,
    const MapInfo &Map)
{
    int num_targets = targets_position.size()/2;
    std::vector<int> start_goal_pair = get_combination(num_targets+1, 2);
    std::vector<std::vector<int>> path_all;
    std::vector<float> distance_all;
    int start[2];
    int goal[2];

    for (unsigned long idx = 0; idx < start_goal_pair.size(); idx = idx + 2)
    {
        int start_idx = start_goal_pair[idx];
        int goal_idx = start_goal_pair[idx+1];

        if (start_idx != 0)
        {
            start[0] = targets_position[2*(start_idx-1)];
            start[1] = targets_position[2*(start_idx-1)+1];
        }
        else
        {
            start[0] = agent_position[0];
            start[1] = agent_position[1];
        }

        if (goal_idx != 0)
        {
            goal[0] = targets_position[2*(goal_idx-1)];
            goal[1] = targets_position[2*(goal_idx-1)+1];

        }
        else
        {
            goal[0] = agent_position[0];
            goal[1] = agent_position[1];
        }
        auto [path_single, distance] = find_path(start, goal, Map);

        path_all.push_back(path_single);
        distance_all.push_back(distance);
    }

    return {path_all, distance_all};
}


#endif