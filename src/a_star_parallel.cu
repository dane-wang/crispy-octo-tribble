#include <ros/ros.h> 
//#include <geometry_msgs>
// #include "geometry_msgs/Point.h"
#include <time.h>  
#include <vector>
#include "graph_search/my_msg.h"
#include "graph_search/planner1.h"
#include <algorithm>

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

__device__ bool path_found_gpu;
__device__ int neighbor_gpu[4];
__device__ int goal_gpu;

struct is_negative
{
  __host__ __device__
  bool operator()(int x)
  {
    return x ==-1;
  }
};



template <typename T> 
__global__ void get_h(T* q,  planner::Node* graph, float* h, int q_size )
{

  int tid = blockIdx.x *blockDim.x + threadIdx.x;
  if (tid < q_size){
    int node = q[tid];

    h[tid] = graph[node].h;

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
    printf("Hello from thread %d, I am explored %d\n", tid, explored_index);
    // planner::Node* temp_node = graph[explored_index].parent;
    // while (!temp_node->start){
       
    //     temp_node->path = true;
    //     temp_node = temp_node->parent;
    // }
    path_found_gpu = true;
  }

  if (!path_found_gpu){
    for (int i=0; i<4; i++)
    {   

      int new_index = explored_index + neighbor_gpu[i];
      bool edge_detect = true;

      if (new_index >=0) {
                
        if ((explored_index%n ==0 && neighbor_gpu[i] == -1) || (explored_index%(n-1) ==0 && neighbor_gpu[i] == 1) || new_index<0 || new_index >= n*n){
          edge_detect = false;
        }


        if (graph[new_index].obstacle == false && graph[new_index].frontier == false && graph[new_index].explored == false && edge_detect)
        {
          graph[new_index].g = graph[explored_index].g + 1;
            
          float h_1 = sqrt(pow((graph[new_index].x-graph[goal_gpu].x),2) + pow((graph[new_index].y-graph[goal_gpu].y),2));
            // printf("%f", h_1);
          graph[new_index].h = h_1;

            
          graph[new_index].f = graph[new_index].h + graph[new_index].g;
          graph[new_index].parent = explored_index;
          graph[new_index].frontier = true;
          
          new_q[4*tid+i] = new_index;
        }
        else if (edge_detect && graph[new_index].obstacle == false && (graph[new_index].frontier == true || graph[new_index].explored == true))
        {
          if (graph[new_index].g > graph[explored_index].g + 1)
          {
            graph[new_index].g = graph[explored_index].g + 1;
            graph[new_index].f = graph[new_index].h + graph[new_index].g;
            graph[new_index].parent = explored_index;
          }
        }

      }
        

    }

  }

}



  


int main(int argc, char** argv)
{
  ros::init(argc, argv, "parallel_planning");

  int n = 100;

  // planner::Node* graph1;
  // graph1 = (planner::Node*)malloc(sizeof(planner::Node)*2000*2000);
  planner::Node graph[n*n];

  

  // map initialization
  for (int y =0; y<n; y++){

    for (int x=0; x<n; x++){

      graph[y*n+x].x = x;
      graph[y*n+x].y = y;
    }
  }

  // thrust::host_vector<int> H(4);

  // std::cout << "a " << H.size() << std::endl;

  // Initialize the start and goal node
  
  int start = 10;
  int goal = 7000;

  int path1 = goal;
  bool path_found = false;

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
  int neighbor[4] = {1, -1, n, -n};

  // Create the priority queue for frontier
  

  // Start to work with CUDA

  thrust::host_vector<int> q_lists;

  q_lists.push_back(start);

  // Start to allocate memory on gpu:
  
  const int map_size = n*n*sizeof(planner::Node);

  planner::Node *map_gpu;

  


  cudaMalloc( (void**)&map_gpu, map_size );
  cudaMemcpy(map_gpu, &graph, map_size, cudaMemcpyHostToDevice);

  cudaMemcpyToSymbol(path_found_gpu, &path_found,  sizeof(bool));
  cudaMemcpyToSymbol(neighbor_gpu, &neighbor,  4*sizeof(int));
  cudaMemcpyToSymbol(goal_gpu, &goal,  sizeof(int));


  thrust::device_vector<int> q_lists_gpu = q_lists;

  ros::NodeHandle nh; 

    // 发布消息 话题名字 队列大小
	ros::Publisher pub = nh.advertise<graph_search::my_msg> ("start_and_goal", 100);
    
    //geometry_msgs::Point start_goal;
  graph_search::my_msg map;


  while (ros::ok()) {
    while(ros::ok() && q_lists_gpu.size()!=0 && !path_found){

      int q_size = q_lists_gpu.size();
      thrust::device_vector<int> new_q_lists_gpu(4*q_size);
      thrust::fill(new_q_lists_gpu.begin(), new_q_lists_gpu.end(), -1);

      int thread_size = min(1024, q_size);

 
      
      explore<<<1,thread_size>>>(thrust::raw_pointer_cast(q_lists_gpu.data()),  map_gpu, thrust::raw_pointer_cast(new_q_lists_gpu.data()));
      cudaDeviceSynchronize();
      cudaMemcpyFromSymbol(&path_found, path_found_gpu,  sizeof(bool), 0, cudaMemcpyDeviceToHost );

      cudaMemcpy(&graph, map_gpu,  map_size, cudaMemcpyDeviceToHost );

      // Remove all element that is not used during the exploration and repeated value
      
      new_q_lists_gpu.erase(thrust::remove_if(new_q_lists_gpu.begin(), new_q_lists_gpu.end(), is_negative()),  new_q_lists_gpu.end() );
      new_q_lists_gpu.erase(thrust::unique(new_q_lists_gpu.begin(), new_q_lists_gpu.end()), new_q_lists_gpu.end() );

      std::cout << new_q_lists_gpu.size() << std::endl;

      // for (int o =0; o< new_q_lists_gpu.size(); o++){
      //   int as = new_q_lists_gpu[o];

        
      //   // printf("%d\n", as);
        
      // }

      // for (int o =0; o< new_q_lists_gpu.size(); o++){
      //   int ss = new_q_lists_gpu[0];
      //   printf("%d\n", ss);
      // }
      
      // Create new q list based on origional and updated q
      if (q_size <= 1024) {
        q_lists_gpu.clear();
        q_lists_gpu = new_q_lists_gpu;
        new_q_lists_gpu.clear();
      }
      else {
        
        q_lists_gpu.erase(q_lists_gpu.begin(), q_lists_gpu.begin()+1024 );
        q_lists_gpu.insert(q_lists_gpu.end(), new_q_lists_gpu.begin(), new_q_lists_gpu.end() );
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
