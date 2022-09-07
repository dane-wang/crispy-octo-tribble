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

extern "C" void parallel_explore(planner::Node* graph, int n, int start_index, int goal_index, int max_thread, std::vector<int>& path);

int main(int argc, char** argv){
    ros::init(argc, argv, "parallel_planning_timing");
    //generate map info from the config file
    int n, max_thread_size;
    std::vector<int> start_coord, goal_coord;
    std::vector<int> obstacles;
    XmlRpc::XmlRpcValue xml_obstacles;

    ros::param::get("map_size", n);
    ros::param::get("start_position", start_coord);
    ros::param::get("goal_position", goal_coord);
    ros::param::get("obstacles", xml_obstacles);
    ros::param::get("max_thread", max_thread_size);

    // Initialize the start and goal node
    int start = start_coord[0]+start_coord[1] * n;
    int goal = goal_coord[0]+goal_coord[1] * n;

    // int start = 0;
    // int goal = 456;
    // std::vector<int>  obstacles = {10, 40 ,400, 230};

    // Initialize the obstacles list
    for(int i=0; i< xml_obstacles.size(); i++){
        int obstacles_index =  (int)xml_obstacles[i][0] +  (int)xml_obstacles[i][1] * n;
        obstacles.push_back( obstacles_index);
    }
    planner::Node graph[n*n];
    planner::map_generation(&graph[0], n, start, goal, obstacles);

    std::vector<int> path;

    parallel_explore(&graph[0], n, start, goal, max_thread_size, path);

    std::cout<< "Path length is "<<path.size()<< std::endl;

    ros::NodeHandle nh; 

    // 发布消息 话题名字 队列大小
	ros::Publisher pub = nh.advertise<graph_search::my_msg> ("planning_info", 100);
    
    //geometry_msgs::Point start_goal;
    graph_search::my_msg map;

    std::vector<int8_t> v(n*n, 0);
    for (int y =0; y<n; y++)
    { 
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
    // for (int k =0; k< n*n; k++){

    //   std::cout<< static_cast<int16_t>(v[k]) << std::endl;

    // }
        
    map.points = v;
    ros::Rate loop_rate(5);
    while(ros::ok)
    {
        pub.publish(map);
        loop_rate.sleep(); 

    }
    
    

    return 0;
}