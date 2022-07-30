// #ifndef PLANNER_H
// #define PLANNER_H

#include <iostream>
#include <string>
#include <algorithm>
#include <vector>
#include <math.h>


namespace planner
{
    struct Node
    
    {
        int x,y, parent;
        bool start = false, goal = false, path = false;
        bool obstacle = false, explored = false, frontier = false;
        // cost from start and estimate cost to goal
        
        float h = INFINITY, g = INFINITY, f = INFINITY;
       

    };

    float h_calculation(Node* Node1, Node* Node2){

        return sqrt(pow((Node1->x-Node2->x),2) + pow((Node1->y-Node2->y),2));
    };

    bool sortcol(const std::vector<float>& v1, const std::vector<float>& v2)
    {
        return v1[1] > v2[1];
    }



}


// #endif