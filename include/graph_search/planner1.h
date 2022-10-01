#ifndef PLANNER_H1
#define PLANNER_H1

#include <iostream>
#include <string>
#include <algorithm>
#include <vector>
#include <math.h>


namespace planner
{
    //The data struction for the grip map
    struct Node
    
    {
        int x,y, parent= 0;
        bool start = false, goal = false, path = false;
        bool obstacle = false, hidden_obstacle = false, explored = false, frontier = false;
        // cost from start and estimate cost to goal
        
        float h = INFINITY, g = INFINITY, f = INFINITY;
        

    };


    float h_calculation(Node* Node1, Node* Node2);

    bool sortcol(const std::vector<float>& v1, const std::vector<float>& v2);
    

    // //Generate the map array for future calculation
    void map_generation(Node* graph, int n, int start, int goal, std::vector<int> & obstacles);

    void sequential_explore(planner::Node* graph, int n, int start_index, int goal_index, std::vector<int>& path_to_goal);

    bool edge_detection(int explored_index, int n, int i, int* neighbor);

}


#endif