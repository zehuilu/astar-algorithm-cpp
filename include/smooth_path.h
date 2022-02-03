#ifndef SMOOTH_PATH_H
#define SMOOTH_PATH_H

#include <iostream>
#include <vector>
#include <cstdlib>
#include "MapInfo.h"
#include "line_of_sight.h"

/*
Input:
    path: {x0,y0, x1,y1, x2,y2, ...}
    Map: a struct MapInfo

Output:
    path_output:
*/
inline std::tuple<std::vector<int>, bool> smooth_path_once(
    std::vector<int> &path,
    const MapInfo &Map)
{
    int num_waypoint = path.size()/2; // each point has two elements
    bool all_block_flag = true;

    for (int i = 0; i < num_waypoint - 2; i++) {
        int start[2];
        start[0] = path[i*2];
        start[1] = path[i*2+1];
        int end[2];
        end[0] = path[i*2+4];
        end[1] = path[i*2+5];

        if (line_of_sight(start, end, Map)) {
            path.erase(path.begin()+i*2+2);
            path.erase(path.begin()+i*2+2);

            num_waypoint -= 1;
            i--;
        }
    }
    return {path, all_block_flag};
}

inline std::vector<int> smooth_path(
    std::vector<int> &path,
    const MapInfo &Map)
{
    std::vector<int> path_output = path;

    while (true) {
        auto _result = smooth_path_once(path_output, Map);
        path_output = std::get<0>(_result);
        auto all_block_flag = std::get<1>(_result);
        if (all_block_flag) break;
    }

    return path_output;
}

#endif