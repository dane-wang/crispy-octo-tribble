#include <ros/ros.h> 
//#include <geometry_msgs>
// #include "geometry_msgs/Point.h"
#include <time.h>  
#include <vector>
#include "graph_search/my_msg.h"
#include "graph_search/planner1.h"
#include <algorithm>
#include <xmlrpcpp/XmlRpcValue.h>

#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <stdio.h>
#include <cuda.h>

#include <iostream>
#include <thrust/host_vector.h>
#include <thrust/device_vector.h>
#include <thrust/extrema.h>
#include <thrust/merge.h>
#include <queue>

#include "graph_search/parallel_explore.cuh"

extern "C" void parallel_explore(planner::Node* graph, int n, bool path_found, int start_index, int max_thread);

int main(){
    int n =10;
    planner::Node graph[n*n];
    bool path_found = false;
    int start = 10;
    int max_thread_size = 5;
    parallel_explore(&graph[0], n, path_found, start, max_thread_size);

    return 0;
}