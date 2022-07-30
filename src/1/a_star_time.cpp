#include <time.h>  
#include <vector>
#include <chrono>
#include "planner1.h"
#include <algorithm>

int main (int argc, char **argv) 
{ 
    using std::chrono::high_resolution_clock;
    using std::chrono::duration_cast;
    using std::chrono::duration;
    using std::chrono::milliseconds;
	auto t1 = high_resolution_clock::now();
    int n = 20;
    
    planner::Node graph[n*n];

    for (int y =0; y<n; y++){

        for (int x=0; x<n; x++){

            graph[y*n+x].x = x;
            graph[y*n+x].y = y;
        }
    }

    // Initialize the start and goal node
    int start = 1;
    int goal = n*n-1;
    bool path_found = false;

    int path1 = goal;

    graph[start].start = true;
    graph[start].g = 0;
    graph[start].h = planner::h_calculation(&graph[start], &graph[goal]);
    graph[start].f = graph[start].h + graph[start].g;
    graph[start].explored = true;

    graph[goal].goal = true;
    graph[goal].h = 0;


    graph[350].obstacle = true;
    graph[341].obstacle = true;
    graph[320].obstacle = true;
    graph[71].obstacle = true;
    // Create the priority queue for frontier
    std::vector<std::vector<float> > q_list;
    q_list.push_back({(float) start, graph[start].f});
    //std::cout << q_list.size() << std::endl;

    int neighbor[4] = {1, -1, n, -n};


  
    while(q_list.size()!=0 && !path_found){
            // pop the node with smallest node
            auto smallest_node = q_list.back();
            q_list.pop_back();
            int explored_index = smallest_node[0];

            //  std::cout << explored_index << std::endl;

            graph[explored_index].explored = true;
            graph[explored_index].frontier = false;

            // if we found the path
            if (explored_index == goal){
                auto t2 = high_resolution_clock::now();
                duration<double, std::milli> ms_double = t2 - t1;
                std::cout <<"Total time is \t" << ms_double.count() << "ms\n";
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
                for (int i=0; i<4; i++)
                {
                    int new_index = explored_index + neighbor[i];
                    bool edge_detect = true;

                    if ((explored_index%n ==0 && neighbor[i] == -1) || (explored_index%(n-1) ==0 && neighbor[i] == 1) || new_index<0 || new_index >= n*n){
                        edge_detect = false;
                    }


                    if (graph[new_index].obstacle == false && graph[new_index].frontier == false && graph[new_index].explored == false && edge_detect)
                    {
                        graph[new_index].g = graph[explored_index].g + 1;
                        graph[new_index].h = planner::h_calculation(&graph[new_index], &graph[goal]);
                        graph[new_index].f = graph[new_index].h + graph[new_index].g;
                        graph[new_index].parent = explored_index;
                        graph[new_index].frontier = true;

                        q_list.push_back({(float) new_index, graph[new_index].f});
                    }
                    else if (edge_detect && graph[new_index].obstacle == false && (graph[new_index].frontier == true || graph[new_index].explored == true))
                    {
                        if (graph[new_index].g > graph[explored_index].g + 1)
                        {
                            graph[new_index].g = graph[explored_index].g + 1;
                            graph[new_index].f = graph[new_index].h + graph[new_index].g;
                            graph[new_index].parent = explored_index;
                            q_list.push_back({(float) new_index, graph[new_index].f});

                        }
                    }
                    

                }
            
                std::sort(q_list.begin(), q_list.end(), planner::sortcol);
            }
            else{
                while (path1 != start)
                {
                    graph[path1].path = true;
                    path1 = graph[path1].parent;
                }
                


            }
            //std::cout << q_list.size() << std::endl;
            


              
            
           
        }
        
	
	return 0; 
}
