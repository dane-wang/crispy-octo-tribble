#include <ros/ros.h> 
//#include <geometry_msgs>
// #include "geometry_msgs/Point.h"
#include <time.h>  
#include <vector>
#include "graph_search/my_msg.h"
#include "graph_search/planner1.h"
#include <algorithm>
#include <xmlrpcpp/XmlRpcValue.h>
#include <stdio.h>
#include <iostream>
#include <queue>
#include "graph_search/parallel_explore.cuh"
#include <chrono>

extern "C" void parallel_explore(planner::Node* graph, int n, int start_index, int goal_index, int max_thread, std::vector<int>& path);

int main(int argc, char** argv){
    ros::init(argc, argv, "parallel_planning_timing");
    //generate map info from the config file
    int n, max_thread_size, use_parallel_planning;
    std::vector<int> start_coord, goal_coord;
    std::vector<int> obstacles;
    std::vector<int> hidden_obstacles;
    XmlRpc::XmlRpcValue xml_obstacles;
    XmlRpc::XmlRpcValue xml_hidden_obstacles;

    ros::param::get("map_size", n);
    ros::param::get("start_position", start_coord);
    ros::param::get("goal_position", goal_coord);
    ros::param::get("obstacles", xml_obstacles);
    ros::param::get("hidden_obstacles", xml_hidden_obstacles);
    ros::param::get("max_thread", max_thread_size);
    ros::param::get("use_parallel", use_parallel_planning);
    

    // Initialize the start and goal node
    int start = start_coord[0]+start_coord[1] * n;
    int goal = goal_coord[0]+goal_coord[1] * n;

    int current = start;
    bool path_found = false;

    ros::NodeHandle nh; 

    // 发布消息 话题名字 队列大小
	ros::Publisher pub = nh.advertise<graph_search::my_msg> ("planning_info", 100);
    ros::Rate loop_rate(3);
    
    //geometry_msgs::Point start_goal;
    graph_search::my_msg map;

  
    // Initialize the obstacles list
    for(int i=0; i< xml_obstacles.size(); i++){
        int obstacles_index =  (int)xml_obstacles[i][0] +  (int)xml_obstacles[i][1] * n;
        obstacles.push_back( obstacles_index);
    }

    // Initialize the hidden obstacles list
    for(int i=0; i< xml_hidden_obstacles.size(); i++){
        int hidden_obstacles_index =  (int)xml_hidden_obstacles[i][0] +  (int)xml_hidden_obstacles[i][1] * n;
        hidden_obstacles.push_back( hidden_obstacles_index);
    }

    planner::Node graph[n*n];
    planner::Node graph_copy[n*n];
    planner::map_generation(&graph[0], n, start, goal, obstacles);
    planner::add_hidden_obstacles(&graph[0], hidden_obstacles);

    
    std::vector<int> path;

  

    while (ros::ok()){
        while (ros::ok() && current!=goal)
        {
           
            if (!path_found){
                std::copy(graph, graph+n*n, graph_copy);
                auto start_time = std::chrono::high_resolution_clock::now();
                if (use_parallel_planning) 
                {
                    parallel_explore(&graph_copy[0], n, current, goal, max_thread_size, path);
                }
                else{
                    planner::sequential_explore(&graph_copy[0], n, current, goal, path);
                }
                auto stop = std::chrono::high_resolution_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start_time);
                std::cout << "Exectuation time is " << duration.count() << std::endl;
                std::cout<< "Path length is "<<path.size()<< std::endl;
                path_found = true;

            }
           

            current = path.back();
            path.pop_back();
            planner::obstacle_detection(current, &graph[0], n);
            // std::cout<< "Path length is "<<current<< std::endl;

            

            std::vector<int8_t> v(n*n, 0);

            for (int k =0; k< path.size(); k++){

                v[path[k]] = 250;
            }

            for (int y =0; y<n; y++)
            { 
                for (int x=0; x<n; x++){

                    if (graph[y*n+x].start) {
                    
                        v[y*n+x] = 120;
                        }
                    else if (graph[y*n+x].obstacle){
                        v[y*n+x] = 100;
                        }

                    else if (y*n+x == current){
                        v[y*n+x] = 200;
                        }
                    else if (graph[y*n+x].goal)
                    {
                        v[y*n+x] = 140;
                        }                                         
                }
            }
       
                
            map.points = v;
            
        
            pub.publish(map);
            ros::spinOnce(); 
            loop_rate.sleep(); 
            if (!path.empty()){
                for (int i =(path.size()-1); i> (path.size()-7); i--){
                int path_index = path[i];
                if (graph[path_index].obstacle) {
                    path_found = false;
                    path.clear();
                    break;
                    }

                }
            }
            
        }

        if (current == goal){
            pub.publish(map);
            ros::spinOnce(); 
            loop_rate.sleep(); 
        }
            

        

    }
    
    

    return 0;
}