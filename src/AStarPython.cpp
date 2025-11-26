#include <iostream>
#include <vector>
#include <tuple>
#include <stdio.h>
#include <math.h>
#include "stlastar.h"
#include "MapSearchNode.h"
#include "MapInfo.h"
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "get_combination.h"
#include "find_path.h"
#include "smooth_path.h"
#include <chrono>
#include "ThreadPool.h"
#include <future>


/*
FindPathOneByOne: Find all the collision-free paths consecutively, i.e. the paths from a0~t0, t0~t1, t1~t2, ...

Input:
    agent_position: 1D integer array [x, y] for the agent position
    targets_position: 1D integer array [x0,y0, x1,y1, x2,y2, ...] for the targets positions
    world_map: 1D integer array for the map, flattened by a 2D array map; 0 for no obstacles, 255 for obstacles
    mapSizeX: integer for the width of the map
    mapSizeY: integer for the height of the map

Output:
    path: 2D integer array for all the index paths, [[idx_x_0,idx_y_0, idx_x_1,idx_y_1, ...], [idx_x_0,idx_y_0, idx_x_1,idx_y_1, ...], ...]
    distance: 1D float array for all the distances of the paths
*/
inline std::tuple<std::vector<std::vector<int>>, std::vector<float>> FindPathOneByOne(
    const std::vector<int> &agent_position,
    const std::vector<int> &targets_position,
    const std::vector<int> &world_map,
    const int &map_width,
    const int &map_height)
{
    std::vector<std::vector<int>> path_many;
    std::vector<float> distances_many;
    int start[2];
    int goal[2];

    struct MapInfo Map;
    Map.world_map = world_map;
    Map.map_width = map_width;
    Map.map_height = map_height;

    if (targets_position.size() > 0) {
        // start is agent_position, goal is the first two elements of targets_position, doing the search
        goal[0] = targets_position[0];
        goal[1] = targets_position[1];

        auto [path, distance] = find_path(agent_position.data(), goal, Map);

        path_many.push_back(path);
        distances_many.push_back(distance);

        // Regenerate the neighbors for next run
        for (size_t idx = 2; idx < targets_position.size(); idx = idx + 2) {
            goal[0] = targets_position[idx];
            goal[1] = targets_position[idx+1];

            start[0] = targets_position[idx-2];
            start[1] = targets_position[idx-1];

            // start is the previous target, goal is the current target, doing the search

            auto [path, distance] = find_path(start, goal, Map);

            path_many.push_back(path);
            distances_many.push_back(distance);
        }
    }

    // if targets_position is empty, return empty arrays
    return {path_many, distances_many};
}


/*
FindPathAllMT: The multi-threading version of FindPathAll: Find all the collision-free paths from every element to another element in start+targets.

targets_position potentially contains a large number of targets.

Input:
    agent_position: 1D integer vector [x, y] for the agent position
    targets_position: 1D integer vector [x0,y0, x1,y1, x2,y2, ...] for the targets positions
    world_map: 1D integer vector for the map, flattened by a 2D vector map; 0 for no obstacles, 255 for obstacles
    mapSizeX: integer for the width of the map
    mapSizeY: integer for the height of the map

Output:
    path: 2D integer vector for all the index paths, [[idx_x_0,idx_y_0, idx_x_1,idx_y_1, ...], [idx_x_0,idx_y_0, idx_x_1,idx_y_1, ...], ...]
    distance: 1D float vector for all the distances of the paths
*/
inline std::tuple<std::vector<std::vector<int>>, std::vector<float>> FindPathAllMT(
    const std::vector<int> agent_position,
    const std::vector<int> targets_position,
    const std::vector<int> &world_map,
    int &map_width,
    int &map_height)
{
    // release GIL for true parallelism
    pybind11::gil_scoped_release release;

    struct MapInfo Map;
    Map.world_map = world_map;
    Map.map_width = map_width;
    Map.map_height = map_height;

    int num_targets = targets_position.size()/2;
    std::vector<int> start_goal_pair = get_combination(num_targets+1, 2);
    size_t n_pairs = start_goal_pair.size() / 2;
    std::vector<std::vector<int>> path_all(n_pairs);
    std::vector<float> distance_all(n_pairs);

    // choose number of threads
    size_t num_threads = std::thread::hardware_concurrency();
    if (num_threads == 0) num_threads = 4;

    ThreadPool pool(num_threads);
    std::vector<std::future<void>> futures;
    futures.reserve(n_pairs);

    // parallel: submit tasks
    for (size_t k = 0; k < n_pairs; ++k)
    {
        futures.emplace_back(pool.enqueue([&, k] {
            size_t idx = 2 * k;

            int start_idx = start_goal_pair[idx];
            int goal_idx  = start_goal_pair[idx + 1];

            int start[2], goal[2];

            if (start_idx != 0) {
                start[0] = targets_position[2 * (start_idx - 1)];
                start[1] = targets_position[2 * (start_idx - 1) + 1];
            } else {
                start[0] = agent_position[0];
                start[1] = agent_position[1];
            }

            if (goal_idx != 0) {
                goal[0] = targets_position[2 * (goal_idx - 1)];
                goal[1] = targets_position[2 * (goal_idx - 1) + 1];
            } else {
                goal[0] = agent_position[0];
                goal[1] = agent_position[1];
            }

            auto [path_this, distance] = find_path(start, goal, Map);

            path_all[k] = std::move(path_this);
            distance_all[k] = distance;
        }));
    }

    // Wait for all tasks
    for (auto &f : futures) f.get();

    return {path_all, distance_all};
}


/*
FindPathAllMP: The openMP version of FindPathAll: Find all the collision-free paths from every element to another element in start+targets.

targets_position potentially contains a large number of targets.

Input:
    agent_position: 1D integer vector [x, y] for the agent position
    targets_position: 1D integer vector [x0,y0, x1,y1, x2,y2, ...] for the targets positions
    world_map: 1D integer vector for the map, flattened by a 2D vector map; 0 for no obstacles, 255 for obstacles
    mapSizeX: integer for the width of the map
    mapSizeY: integer for the height of the map

Output:
    path: 2D integer vector for all the index paths, [[idx_x_0,idx_y_0, idx_x_1,idx_y_1, ...], [idx_x_0,idx_y_0, idx_x_1,idx_y_1, ...], ...]
    distance: 1D float vector for all the distances of the paths
*/
inline std::tuple<std::vector<std::vector<int>>, std::vector<float>> FindPathAllMP(
    const std::vector<int> agent_position,
    const std::vector<int> targets_position,
    const std::vector<int> &world_map,
    int &map_width,
    int &map_height)
{
    // release GIL for true parallelism
    pybind11::gil_scoped_release release;

    struct MapInfo Map;
    Map.world_map = world_map;
    Map.map_width = map_width;
    Map.map_height = map_height;

    int num_targets = targets_position.size()/2;
    std::vector<int> start_goal_pair = get_combination(num_targets+1, 2);
    size_t n_pairs = start_goal_pair.size() / 2;
    std::vector<std::vector<int>> path_all(n_pairs);
    std::vector<float> distance_all(n_pairs);

    #pragma omp parallel for schedule(dynamic)
    for (size_t idx = 0; idx < n_pairs; idx++)
    {
        int start_idx = start_goal_pair[2*idx];
        int goal_idx = start_goal_pair[2*idx+1];

        int start[2];
        int goal[2];

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
        auto [path_this, distance] = find_path(start, goal, Map);

        path_all[idx] = std::move(path_this);
        distance_all[idx] = distance;
    }

    return {path_all, distance_all};
}


/*
FindPathAll: Find all the collision-free paths from every element to another element in start+targets.

Input:
    agent_position: 1D integer vector [x, y] for the agent position
    targets_position: 1D integer vector [x0,y0, x1,y1, x2,y2, ...] for the targets positions
    world_map: 1D integer vector for the map, flattened by a 2D vector map; 0 for no obstacles, 255 for obstacles
    mapSizeX: integer for the width of the map
    mapSizeY: integer for the height of the map

Output:
    path: 2D integer vector for all the index paths, [[idx_x_0,idx_y_0, idx_x_1,idx_y_1, ...], [idx_x_0,idx_y_0, idx_x_1,idx_y_1, ...], ...]
    distance: 1D float vector for all the distances of the paths
*/
inline std::tuple<std::vector<std::vector<int>>, std::vector<float>> FindPathAll(
    const std::vector<int> agent_position,
    const std::vector<int> targets_position,
    const std::vector<int> &world_map,
    int &map_width,
    int &map_height)
{
    struct MapInfo Map;
    Map.world_map = world_map;
    Map.map_width = map_width;
    Map.map_height = map_height;

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
        auto [path_this, distance] = find_path(start, goal, Map);

        path_all.push_back(path_this);
        distance_all.push_back(distance);
    }

    return {path_all, distance_all};
}


/*
FindPath: Find a collision-free path from a start to a target.

Input:
    startPoint: 1D integer array [x, y] for the start position
    endPoint: 1D integer array [x, y] for the goal position
    world_map: 1D integer array for the map, flattened by a 2D array map; 0 for no obstacles, 255 for obstacles
    mapSizeX: integer for the width of the map
    mapSizeY: integer for the height of the map

Output:
    path: 1D integer array for the index path from startPoint to endPoint, [idx_x_0,idx_y_0, idx_x_1,idx_y_1, idx_x_2,idx_y_2, ...]
    distance: float for the total distance of the path
*/
inline std::tuple<std::vector<int>, float> FindPath(
    const std::vector<int> &start,
    const std::vector<int> &end,
    const std::vector<int> &world_map,
    const int &map_width,
    const int &map_height)
{
    // std::cout << "STL A* Search implementation\n(C)2001 Justin Heyes-Jones\n";

    // Our sample problem defines the world as a 2d array representing a terrain
    // Each element contains an integer from 0 to 5 which indicates the cost 
    // of travel across the terrain. Zero means the least possible difficulty 
    // in travelling (think ice rink if you can skate) whilst 5 represents the 
    // most difficult. 255 indicates that we cannot pass.

    // Create an instance of the search class...

    struct MapInfo Map;
    Map.world_map = world_map;
    Map.map_width = map_width;
    Map.map_height = map_height;

    auto [path, distance] = find_path(start.data(), end.data(), Map);

    return {path, distance};
}


inline std::tuple<std::vector<int>, std::vector<int>, int, float> FindPath_test(
    const std::vector<int> &start,
    const std::vector<int> &end,
    const std::vector<int> &world_map,
    int &map_width,
    int &map_height)
{
    // std::cout << "STL A* Search implementation\n(C)2001 Justin Heyes-Jones\n";

    // Our sample problem defines the world as a 2d array representing a terrain
    // Each element contains an integer from 0 to 5 which indicates the cost 
    // of travel across the terrain. Zero means the least possible difficulty 
    // in travelling (think ice rink if you can skate) whilst 5 represents the 
    // most difficult. 255 indicates that we cannot pass.

    // Create an instance of the search class...

    struct MapInfo Map;
    Map.world_map = world_map;
    Map.map_width = map_width;
    Map.map_height = map_height;

    AStarSearch<MapSearchNode> astarsearch;

    unsigned int SearchCount = 0;
    const unsigned int NumSearches = 1;

    // full path
    std::vector<int> path_full;
    // a short path only contains path corners
    std::vector<int> path_short;
    // how many steps used
    int steps = 0;

    while(SearchCount < NumSearches)
    {
        // MapSearchNode nodeStart;
        MapSearchNode nodeStart = MapSearchNode(start[0], start[1], Map);
        MapSearchNode nodeEnd(end[0], end[1], Map);

        // Set Start and goal states
        astarsearch.SetStartAndGoalStates( nodeStart, nodeEnd );

        unsigned int SearchState;
        unsigned int SearchSteps = 0;

        do
        {
            SearchState = astarsearch.SearchStep();
            SearchSteps++;
        }
        while( SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_SEARCHING );

        if( SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_SUCCEEDED )
        {
            // std::cout << "Search found goal state\n";
            MapSearchNode *node = astarsearch.GetSolutionStart();
            steps = 0;

            // node->PrintNodeInfo();
            path_full.push_back(node->x);
            path_full.push_back(node->y);

            while (true)
            {
                node = astarsearch.GetSolutionNext();

                if ( !node )
                {
                    break;
                }

                // node->PrintNodeInfo();
                path_full.push_back(node->x);
                path_full.push_back(node->y);

                steps ++;

                /*
                Let's say there are 3 steps, x0, x1, x2. To verify whether x1 is a corner for the path.
                If the coordinates of x0 and x1 at least have 1 component same, and the coordinates of 
                x0 and x2 don't have any components same, then x1 is a corner.

                Always append the second path point to path_full.
                When steps >= 2 (starting from the third point), append the point if it's a corner.
                */

                if ((((path_full[2*steps-4]==path_full[2*steps-2]) || (path_full[2*steps-3]==path_full[2*steps-1])) && 
                    ((path_full[2*steps-4]!=node->x) && (path_full[2*steps-3]!=node->y)) && (steps>=2)) || (steps < 2))
                {
                    path_short.push_back(path_full[2*steps-2]);
                    path_short.push_back(path_full[2*steps-1]);
                }
            }

            // the last two elements
            // This works for both steps>2 and steps <=2
            path_short.push_back(path_full[path_full.size()-2]);
            path_short.push_back(path_full[path_full.size()-1]);

            // std::cout << "Solution steps " << steps << endl;

            // Once you're done with the solution you can free the nodes up
            astarsearch.FreeSolutionNodes();
            
        }
        else if( SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_FAILED ) 
        {
            std::cout << "Search terminated. Did not find goal state\n";
        }

        // Display the number of loops the search went through
        // std::cout << "SearchSteps : " << SearchSteps << "\n";

        SearchCount ++;

        astarsearch.EnsureMemoryFreed();

    }

    // auto t_start = std::chrono::high_resolution_clock::now();
    std::vector<int> path_output = smooth_path(path_short, Map);
    // auto t_stop = std::chrono::high_resolution_clock::now();
    // auto duration = std::chrono::duration_cast<std::chrono::microseconds>(t_stop - t_start);
    // std::cout << "smooth path time [microseconds]: " << duration.count() << std::endl;

    float distance = 0.0;
    for (size_t idx = 0; idx < path_output.size()/2 - 1; idx++) {
        distance += std::sqrt(std::pow(path_output[2*idx]-path_output[2*(idx+1)], 2) + std::pow(path_output[2*idx+1]-path_output[2*(idx+1)+1], 2));
    }

    return {path_short, path_output, steps, distance};
}


inline PYBIND11_MODULE(AStarPython, module) {
    module.doc() = "Python wrapper of AStar c++ implementation";

    module.def("FindPath", &FindPath, "Find a collision-free path");
    module.def("FindPathAll", &FindPathAll, "Find all the collision-free paths between every two nodes");
    module.def("FindPathAllMP", &FindPathAllMP, "openMP version: Find all the collision-free paths between every two nodes");
    module.def("FindPathAllMT", &FindPathAllMT, "Multi-threading version: Find all the collision-free paths between every two nodes");
    module.def("FindPathOneByOne", &FindPathOneByOne, "Find all the collision-free paths consecutively");

    module.def("FindPath_test", &FindPath_test, "Find a collision-free path (TEST VERSION)");
}
