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

    //Generate the map array for future calculation
    void map_generation(Node* graph, int n, int start, int goal, std::vector<int> & obstacles){
        
        

        //Assign coordinate
        for (int y =0; y<n; y++){

            for (int x=0; x<n; x++){

                graph[y*n+x].x = x;
                graph[y*n+x].y = y;
            }
        }
        graph[start].start = true;
        graph[start].g = 0;
        graph[start].h = h_calculation(&graph[start], &graph[goal]);
        graph[start].f = graph[start].h + graph[start].g;
        graph[start].explored = true;

        graph[goal].goal = true;
        graph[goal].h = 0;

        for (int i =0; i<obstacles.size(); i++){
            graph[obstacles[i]].obstacle = true;
        }


    }



}


#endif