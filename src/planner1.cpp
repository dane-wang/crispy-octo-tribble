#include "graph_search/planner1.h"
#include <iostream>
#include <string>
#include <algorithm>
#include <vector>
#include <math.h>




float planner::h_calculation(planner::Node* Node1, planner::Node* Node2){

        return sqrt(pow((Node1->x-Node2->x),2) + pow((Node1->y-Node2->y),2));
    };


bool planner::sortcol(const std::vector<float>& v1, const std::vector<float>& v2)
    {
        return v1[1] > v2[1];
    }

void planner::map_generation(planner::Node* graph, int n, int start, int goal, std::vector<int> & obstacles){
        
        

    //Assign coordinate
    for (int y =0; y<n; y++){

        for (int x=0; x<n; x++){

            graph[y*n+x].x = x;
            graph[y*n+x].y = y;
        }
    }
    graph[start].start = true;
    graph[start].g = 0;
    // graph[start].h = h_calculation(&graph[start], &graph[goal]);
    // graph[start].f = graph[start].h + graph[start].g;
    // graph[start].explored = true;

    graph[goal].goal = true;
    // graph[goal].h = 0;

    for (int i =0; i<obstacles.size(); i++){
        graph[obstacles[i]].obstacle = true;
    }


}


void planner::sequential_explore(planner::Node* graph, int n, int start_index, int goal_index, std::vector<int>& path_to_goal){

    //Initialize everything
    bool path_found = false;
    std::vector<std::vector<float> > q_list;
    graph[start_index].g = 0;
    if (graph[start_index].h == INFINITY) graph[start_index].h = h_calculation(&graph[start_index], &graph[goal_index]);
    graph[start_index].f = graph[start_index].g + graph[start_index].h;
    q_list.push_back({(float) start_index, graph[start_index].f});
    int neighbor[8] = {1, -1, n, -n, n+1, n-1, -n+1, -n-1};

    while(q_list.size()!=0 && !path_found){
        // pop the node with smallest node
        auto smallest_node = q_list.back();
        q_list.pop_back();
        int explored_index = smallest_node[0];

        //std::cout << explored_index << std::endl;

        graph[explored_index].explored = true;
        graph[explored_index].frontier = false;

        // if we found the path
        if (explored_index == goal_index){
            std::cout << "found" << std::endl;
            // planner::Node* temp_node = graph[explored_index].parent;
            // while (!temp_node->start){
                
            //     temp_node->path = true;
            //     temp_node = temp_node->parent;
            // }
            path_found = true;
        }

        //std::cout << new_explore_index << std::endl;
        if (!path_found){
            for (int i=0; i<8; i++)
            {
                int new_index = explored_index + neighbor[i];
                float cost = (i < 4) ? 1 : sqrt(2);

                //Check if the new index possible (like if it will go out of the map)
                bool edge_detect = true;

                if ((explored_index%n ==0 && (neighbor[i] == -1 || neighbor[i] == n-1 || neighbor[i] == -n-1 )) || ((explored_index+1)%n ==0 && (neighbor[i] == 1 || neighbor[i] == n+1 || neighbor[i] == -n+1 )) || new_index<0 || new_index >= n*n){
                    edge_detect = false;
                }


                if (graph[new_index].obstacle == false && graph[new_index].frontier == false && graph[new_index].explored == false && edge_detect)
                {
                    graph[new_index].g = graph[explored_index].g + cost;
                    if (graph[new_index].h == INFINITY) graph[new_index].h = planner::h_calculation(&graph[new_index], &graph[goal_index]);
                    graph[new_index].f = graph[new_index].h + graph[new_index].g;
                    graph[new_index].parent = explored_index;
                    graph[new_index].frontier = true;

                    q_list.push_back({(float) new_index, graph[new_index].f});
                }
                else if (edge_detect && graph[new_index].obstacle == false && (graph[new_index].frontier == true || graph[new_index].explored == true))
                {
                    if (graph[new_index].g > graph[explored_index].g + cost)
                    {
                        graph[new_index].g = graph[explored_index].g + cost;
                        graph[new_index].f = graph[new_index].h + graph[new_index].g;
                        graph[new_index].parent = explored_index;
                        q_list.push_back({(float) new_index, graph[new_index].f});

                    }
                }
                

            }
        
            std::sort(q_list.begin(), q_list.end(), planner::sortcol);
        }
        else{
            int path1 = goal_index;
            while (path1 != start_index)
            {
                path_to_goal.push_back(path1);
                graph[path1].path = true;
                path1 = graph[path1].parent;
            }
        
        }
        //std::cout << q_list.size() << std::endl;
        
    }          

}

bool planner::edge_detection(int explored_index, int n, int i, int* neighbor) {

    bool edge_detect = true;
    int new_index = explored_index + neighbor[i];

    // int neighbor[8] = {1, -1, n, -n, n+1, n-1, -n+1, -n-1};

    if ((explored_index%n ==0 && (neighbor[i] == -1 || neighbor[i] == n-1 || neighbor[i] == -n-1 )) || (explored_index%(n-1) ==0 && (neighbor[i] == 1 || neighbor[i] == n+1 || neighbor[i] == -n+1 ) && explored_index!=0) || new_index<0 || new_index >= n*n){
            edge_detect = false;
        }



    return edge_detect;
}


int planner::coordtoindex(std::vector<int>& coordinate, int n){

    int index = coordinate[0] + coordinate[1]*n;
    return index;

}

std::vector<int> planner::indextocoord(int index, int n){
    std::vector<int> coordinate(2);

    coordinate[0] = index%n;
    coordinate[1] = index/n;

    return coordinate;

}

void planner::obstacle_detection(int current, planner::Node* graph, int n){

    std::vector<int> curr_coordinate = planner::indextocoord(current, n);

    

    for (int i = -2; i<3; i++){
        for (int j = -2; j<3; j++){
            std::vector<int> new_coordinate = {curr_coordinate[0]+i, curr_coordinate[1]+j };

            if (new_coordinate[0]>=n || new_coordinate[0]<0 || new_coordinate[1]>=n || new_coordinate[1]<0  ) continue;

            int new_index = planner::coordtoindex(new_coordinate, n);

            if (graph[new_index].hidden_obstacle){

                // std::cout <<"checking" << std::endl;

                graph[new_index].hidden_obstacle = false;
                graph[new_index].obstacle = true;

            } 
        }
    }

    

}

void planner::add_hidden_obstacles(Node* graph, std::vector<int> & hidden_obstacles){

    for (int i =0; i<hidden_obstacles.size(); i++){
        graph[hidden_obstacles[i]].hidden_obstacle = true;
    }

}

