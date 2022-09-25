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

__device__ bool path_found_gpu;
__device__ int neighbor_gpu[8];
__device__ int goal_gpu;


struct is_negative
{
  __host__ __device__
  bool operator()(int x)
  {
    return x ==-1;
  }
};



template <typename T, typename T1> 
__global__ void get_f(T* q,  planner::Node* graph, T1* h, int q_size )
{

  int tid = blockIdx.x *blockDim.x + threadIdx.x;
  if (tid < q_size){
    int node = q[tid];

    h[tid] = graph[node].f;

    // printf("%d", q[tid]);
  }

}

template <typename T>
__global__ void explore(T* q,  planner::Node* graph, T* new_q  )
{
  int tid = threadIdx.x;
  int explored_index = q[tid];
  int n = neighbor_gpu[3];

  graph[explored_index].explored = true;
  graph[explored_index].frontier = false;

  if (graph[explored_index].goal){
    printf("FOUND");
    printf("Hello from thread %d, I am exploring %d\n", tid, explored_index);
    // planner::Node* temp_node = graph[explored_index].parent;
    // while (!temp_node->start){
       
    //     temp_node->path = true;
    //     temp_node = temp_node->parent;
    // }
    path_found_gpu = true;
  }

  if (!path_found_gpu){
    for (int i=0; i<8; i++)
    {   
      
      
      int new_index = explored_index + neighbor_gpu[i];
      float cost;
      
      if (i<4){
        cost = 1;
      }
      else {
        cost = sqrt(2.0);
      }

      bool edge_detect = true;

      
                
      // if ((explored_index%n ==0 && neighbor_gpu[i] == -1) || ((explored_index+1)%n ==0 && neighbor_gpu[i] == 1 &&explored_index!=0) || new_index<0 || new_index >= n*n){
      //   edge_detect = false;
      // }

      if ((explored_index%n ==0 && (neighbor_gpu[i] == -1 || neighbor_gpu[i] == n-1 || neighbor_gpu[i] == -n-1 )) || ((explored_index+1)%n==0 && (neighbor_gpu[i] == 1 || neighbor_gpu[i] == n+1 || neighbor_gpu[i] == -n+1 )) || new_index<0 || new_index >= n*n){
        edge_detect = false;
      }

      


      if (graph[new_index].obstacle == false && graph[new_index].frontier == false && graph[new_index].explored == false && edge_detect)
      {
        graph[new_index].g = graph[explored_index].g + cost;
          
        float h_1 = sqrt(pow((graph[new_index].x-graph[goal_gpu].x),2) + pow((graph[new_index].y-graph[goal_gpu].y),2));
          // printf("%f", h_1);
        graph[new_index].h = h_1;

          
        graph[new_index].f = graph[new_index].h + graph[new_index].g;
        graph[new_index].parent = explored_index;
        graph[new_index].frontier = true;
        
        new_q[8*tid+i] = new_index;
      }
      else if (edge_detect && graph[new_index].obstacle == false && (graph[new_index].frontier == true || graph[new_index].explored == true))
      {
        if (graph[new_index].g > graph[explored_index].g + cost)
        {
          graph[new_index].g = graph[explored_index].g + cost;
          graph[new_index].f = graph[new_index].h + graph[new_index].g;
          graph[new_index].parent = explored_index;
        }
      }
    }

  }

}



  


int main(int argc, char** argv)
{
  ros::init(argc, argv, "parallel_planning");
  

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

  // Initialize the obstacles list
  for(int i=0; i< xml_obstacles.size(); i++){
      int obstacles_index =  (int)xml_obstacles[i][0] +  (int)xml_obstacles[i][1] * n;
      obstacles.push_back( obstacles_index);
  }
  planner::Node graph[n*n];
  planner::map_generation(&graph[0], n, start, goal, obstacles);

  int path1 = goal;
  bool path_found = false;


  ros::NodeHandle nh; 

  // 发布消息 话题名字 队列大小
	ros::Publisher pub = nh.advertise<graph_search::my_msg> ("planning_info", 100);
    
  //geometry_msgs::Point start_goal;
  graph_search::my_msg map;

  // Start to work with CUDA
  thrust::host_vector<int> q_lists;

  q_lists.push_back(start);

  // Start to allocate memory on gpu:
  
  const int map_size = n*n*sizeof(planner::Node);

  planner::Node *map_gpu;

  int neighbor[8] = {1, -1, n, -n, n+1, n-1, -n+1, -n-1};

  cudaMalloc( (void**)&map_gpu, map_size );
  cudaMemcpy(map_gpu, &graph, map_size, cudaMemcpyHostToDevice);

  cudaMemcpyToSymbol(path_found_gpu, &path_found,  sizeof(bool));
  cudaMemcpyToSymbol(neighbor_gpu, &neighbor,  8*sizeof(int));
  cudaMemcpyToSymbol(goal_gpu, &goal,  sizeof(int));


  thrust::device_vector<int> q_lists_gpu = q_lists;

  // parallel_explore(&graph[0], n, path_found, start, max_thread_size);
  
  


  while (ros::ok()) {
    while(ros::ok() && q_lists_gpu.size()!=0 && !path_found){

      int q_size = q_lists_gpu.size();

      //new_q is the list store the frontier generated from this step of exploration
      thrust::device_vector<int> new_q_lists_gpu(8*q_size);
      thrust::fill(new_q_lists_gpu.begin(), new_q_lists_gpu.end(), -1);


      //Determine how many thread should be launched
      int thread_size = min(max_thread_size, q_size);

 
      //Launch the kernel to explore the map
      explore<<<1,thread_size>>>(thrust::raw_pointer_cast(q_lists_gpu.data()),  map_gpu, thrust::raw_pointer_cast(new_q_lists_gpu.data()));
      cudaDeviceSynchronize();
      cudaMemcpyFromSymbol(&path_found, path_found_gpu,  sizeof(bool), 0, cudaMemcpyDeviceToHost );
      cudaMemcpy(&graph, map_gpu,  map_size, cudaMemcpyDeviceToHost );


      // Remove all element that is not used during the exploration and repeated value
      
      new_q_lists_gpu.erase(thrust::remove_if(new_q_lists_gpu.begin(), new_q_lists_gpu.end(), is_negative()),  new_q_lists_gpu.end() );
      
      new_q_lists_gpu.erase(thrust::unique(new_q_lists_gpu.begin(), new_q_lists_gpu.end()), new_q_lists_gpu.end() );
      
      // Create new q list based on origional and updated q
      if (q_size <= max_thread_size) {
        q_lists_gpu.clear();
        q_lists_gpu = new_q_lists_gpu;
        new_q_lists_gpu.clear();
      }
      else {
        
        q_lists_gpu.erase(q_lists_gpu.begin(), q_lists_gpu.begin()+max_thread_size );
        q_lists_gpu.insert(q_lists_gpu.end(), new_q_lists_gpu.begin(), new_q_lists_gpu.end() );
        new_q_lists_gpu.clear();

        // //sort the q_list based on the f value
        thrust::device_vector<float> f_value(q_lists_gpu.size());
        get_f<<<1, q_lists_gpu.size()>>>(thrust::raw_pointer_cast(q_lists_gpu.data()),  map_gpu, thrust::raw_pointer_cast(f_value.data()), q_lists_gpu.size() );
        thrust::sort_by_key(f_value.begin(), f_value.end(), q_lists_gpu.begin() );
      }

      
      //q_size = q_lists_gpu.size();
      // thrust::device_vector<float> h_value(q_size);

      // if (q_size > 1024) {
      //   int block = q_size / 1024 + 1;
        
      //   get_h<<<block, 1024>>>(thrust::raw_pointer_cast(q_lists_gpu.data()),  map_gpu, thrust::raw_pointer_cast(&h_value[0]), q_size );

      //   thrust::sort_by_key(h_value.begin(), h_value.end(), q_lists_gpu.begin() );

      // }

      if (path_found){
        while (path1 != start)
          {
              graph[path1].path = true;
              path1 = graph[path1].parent;
          }


      }

      

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
