#include <ros/ros.h> 
//#include <geometry_msgs>
// #include "geometry_msgs/Point.h"
#include <time.h>  
#include <vector>
#include "graph_search/my_msg.h"
#include "graph_search/planner1.h"
#include <algorithm>

int main (int argc, char **argv) 
{ 
    // 初始化ROS节点 节点名字
	ros::init (argc, argv, "start_and_goal"); 
	
    //map size
    int n = 100;
    
    planner::Node graph[n*n];

    for (int y =0; y<n; y++){

        for (int x=0; x<n; x++){

            graph[y*n+x].x = x;
            graph[y*n+x].y = y;
        }
    }

    // Initialize the start and goal node
    int start = 10;
    int goal = 7000;
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


    // 节点句柄
	ros::NodeHandle nh; 

    // 发布消息 话题名字 队列大小
	ros::Publisher pub = nh.advertise<graph_search::my_msg> ("start_and_goal", 100);
    
    //geometry_msgs::Point start_goal;
    graph_search::my_msg map;

    //int g = 0;
    
    
    while (ros::ok()) {

        while(ros::ok() && q_list.size()!=0 && !path_found){
            // pop the node with smallest node
            auto smallest_node = q_list.back();
            q_list.pop_back();
            int explored_index = smallest_node[0];

             std::cout << explored_index << std::endl;

            graph[explored_index].explored = true;
            graph[explored_index].frontier = false;

            // if we found the path
            if (explored_index == goal){
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
            


              
            std::vector<int8_t> v(n*n, 0);
            for (int y =0; y<n; y++){

                for (int x=0; x<n; x++){

                    if (graph[y*n+x].start) {
                        v[y*n+x] = 120;
                    }
                    else if (graph[y*n+x].goal)
                    {
                        v[y*n+x] = 140;
                    }
                    else if (graph[y*n+x].path){
                        v[y*n+x] = 250;
                    }
                    else if (graph[y*n+x].obstacle){
                        v[y*n+x] = 100;
                    }
                    else if (graph[y*n+x].frontier){
                        v[y*n+x] = 50;
                    }
                    else if (graph[y*n+x].explored){
                        v[y*n+x] = 200;
                    }
                    
                    
                }
            }
           
            // start_goal.x = std::rand()%120+1;
            // start_goal.y = 120;
            ros::Rate loop_rate(5);
            
            map.points = v;

            // map.points[10] = 120;
            // map.points[125] = 140;
        
        
            // 广播
            pub.publish(map);
            ros::spinOnce(); 
            loop_rate.sleep(); 
        }
        if (path_found){
            ros::Rate loop_rate(5);
            pub.publish(map);
            ros::spinOnce(); 
            loop_rate.sleep(); 

        }
        
	} 
	return 0; 
}
