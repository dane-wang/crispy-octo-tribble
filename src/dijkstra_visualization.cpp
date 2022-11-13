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
#include "graph_search/parallel_dijkstra.cuh"
#include <chrono>

extern "C" void parallel_dijkstra(planner::Node* graph, int n, int goal_index, int max_thread);




int main(int argc, char** argv){
    ros::init(argc, argv, "dijkstra_visualization");
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
    
    planner::map_generation_dijkstra(&graph[0], n, start, goal, obstacles);
    planner::add_hidden_obstacles(&graph[0], hidden_obstacles);



    
    std::vector<int> path;

    // gpu_warmup();

  
    parallel_dijkstra(&graph[0], n, goal, max_thread_size);

  

    while (ros::ok()){
        
            

        std::vector<int8_t> v(n*n, 0);

        

        for (int y =0; y<n; y++)
        { 
            for (int x=0; x<n; x++){
                
                v[y*n+x] = graph[y*n+x].h;                     
            }
        }
        // std::cout << graph[0].h << std::endl;

       
            
        map.points = v;
        
    
        pub.publish(map);
        // ros::spinOnce(); 
        loop_rate.sleep(); 
        

    }
    
    

    return 0;
}