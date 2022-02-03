#ifndef LINE_OF_SIGHT_H
#define LINE_OF_SIGHT_H

#include <iostream>
#include <algorithm>
#include <cmath>
#include "MapInfo.h"

/*
line_of_sight() checks if the line which passes (end[0],y1) and (x2,y2) collides the obstacle
cell (start[0],start[1]) by computing the distance between the line and (start[0],start[1]).
If the distance >= sqrt(2)/2, i.e., there is no intersection between the circumscribed
circle of the square cell and the line, then this cell doesn't block the line-of-sight.

Input:
    previousPoint: {column_x, row_y}, denoted as {end[0], y1}
    currentPoint: {column_x, row_y}, denoted as {x2, y2}
    Map: a struct MapInfo

Output:
    false if there is at least one obstacle cell blocks the line-of-sight.
    true if the line-of-sight is clear.
*/

inline bool mIsTraversable(int point[], const MapInfo &Map) {
    return Map.world_map[point[1] * Map.map_width + point[0]] != 255;
}

inline bool line_of_sight(
    int *previousPoint,
    int *currentPoint,
    const MapInfo &Map)
{
    int start[2];
    int end[2];
    start[0] = previousPoint[0];
    start[1] = previousPoint[1];
    end[0]= currentPoint[0];
    end[1] = currentPoint[1];
    int diff_x = end[0]-start[0];
    int diff_y = end[1]-start[1];
    int dir[2]; // Direction of movement. Value can be either 1 or -1.

    // The x and y locations correspond to nodes, not cells. We might need to check different surrounding cells depending on the direction we do the
    // line of sight check. The following values are used to determine which cell to check to see if it is unblocked.

    if(diff_y >= 0)
    {
        dir[1] = 1;
    }
    else
    {
        diff_y = -diff_y;
        dir[1] = -1;
    }

    if(diff_x >= 0)
    {
        dir[0] = 1;
    }
    else
    {
        diff_x = -diff_x;
        dir[0] = -1;
    }
    
    // x,y as shown in plot. lx,ly are coordinates on the boundayr of cell
    // 0.5 converts the cell index to x-y coordinates
    float lx = start[0] + 0.5 + (float) diff_x / diff_y * dir[0] / 2;
    float ly = start[1] + 0.5 + (float) diff_y / diff_x * dir[1] / 2;

    if(diff_x >= diff_y)
    { // Move along the x axis and increment/decrement y.
        while(start[0] != end[0])
        {	//(int) ly will change to next integer when line of sight cross the boundary of cell
            if(!mIsTraversable(start, Map))
                return false;
            if((int) ly != start[1])
            {
                start[1] += dir[1];
                if(!mIsTraversable(start, Map))
                    return false;
            }
            start[0] += dir[0];
            ly += (float) diff_y / diff_x * dir[1];
        }
    }
    else
    {  //if (diff_x < diff_y). Move along the y axis and increment/decrement x.
        while (start[1] != end[1])
        {
            if(!mIsTraversable(start, Map))
                return false;
            if((int) lx != start[0])
            {
                start[0] += dir[0];
                if(!mIsTraversable(start, Map))
                    return false;
            }
            start[1] += dir[1];
            lx += (float) diff_x / diff_y * dir[0];   
        }
    }
    return true;
}

#endif