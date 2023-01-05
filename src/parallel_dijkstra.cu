
#include <time.h>  
#include <vector>
#include "graph_search/planner1.h"
#include "graph_search/parallel_dijkstra.cuh"
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


// __device__ bool path_found_gpu;
__device__ int neighbor_gpu[8];
__device__ int g_gpu;

struct is_negative
{
  __host__ __device__
  bool operator()(int x)
  {
    return x ==-1;
  }
};



// template <typename T, typename T1> 
// __global__ void get_f(T* q,  planner::Node* graph, T1* h, int q_size )
// {

//   int tid = blockIdx.x *blockDim.x + threadIdx.x;
//   if (tid < q_size){
//     int node = q[tid];

//     h[tid] = graph[node].f;

//     // printf("%d", q[tid]);
//   }

// }

// template <typename T>
// __global__ void explore(T* q,  planner::Node* graph, T* new_q )
// {
//   int tid = threadIdx.x;
//   int explored_index = q[tid];
//   int n = neighbor_gpu[3];


//   graph[explored_index].explored = true;
//   if (graph[explored_index].h > g_gpu){
//     graph[explored_index].h = g_gpu;
//   }

//   for (int i=0; i<8; i++)
//   {   
    
//     int new_index = explored_index + neighbor_gpu[i];

//     bool edge_detect = true;

    
//     if ((explored_index%n ==0 && (neighbor_gpu[i] == -1 || neighbor_gpu[i] == n-1 || neighbor_gpu[i] == -n-1 )) || ((explored_index+1)%n ==0 && (neighbor_gpu[i] == 1 || neighbor_gpu[i] == n+1 || neighbor_gpu[i] == -n+1 )) || new_index<0 || new_index >= n*n){
//       edge_detect = false;
//     }


//     if (graph[new_index].obstacle == false && edge_detect && !graph[new_index].explored )
//     {      
//       new_q[8*tid+i] = new_index;
//     }
   
//   }

  

// }

template <typename T>
__global__ void explore(T* q,  planner::Node* graph, T* new_q  )
{
  int tid = threadIdx.x;
  int explored_index = q[tid];
  int n = neighbor_gpu[2];

  // graph[explored_index].explored = true;
  // graph[explored_index].frontier = false;

  // if (explored_index == 0){
  //   printf("I am working on 0" );
  // }




  for (int i=0; i<8; i++)
  {   
    
    int new_index = explored_index + neighbor_gpu[i];

    bool edge_detect = true;

    
              
    // if ((explored_index%n ==0 && neighbor_gpu[i] == -1) || (explored_index%(n-1) ==0 && neighbor_gpu[i] == 1 &&explored_index!=0) || new_index<0 || new_index >= n*n){
    //   edge_detect = false;
    // }

    if ((explored_index%n ==0 && (neighbor_gpu[i] == -1 || neighbor_gpu[i] == n-1 || neighbor_gpu[i] == -n-1 )) || ((explored_index+1)%n ==0 && (neighbor_gpu[i] == 1 || neighbor_gpu[i] == n+1 || neighbor_gpu[i] == -n+1 )) || new_index<0 || new_index >= n*n){
      edge_detect = false;
    }


    if (graph[new_index].obstacle == false && graph[new_index].h == INFINITY && edge_detect)
    {
      graph[new_index].h = g_gpu;

      // graph[new_index].parent = explored_index;
      // graph[new_index].frontier = true;
      
      new_q[8*tid+i] = new_index;
    }
    else if (edge_detect && (graph[new_index].h != INFINITY))
    {
      if (graph[new_index].h > g_gpu)
      {
        graph[new_index].h = g_gpu;
        
        graph[new_index].parent = explored_index;
      }
    }
  }

  
}



extern "C"
void parallel_dijkstra(planner::Node* graph, int n, int goal_index, int max_thread){


  // Use dijkstra to initialize the heuristic values for all the node.


  //The heuristic for goal is 0
  // graph[goal_index].h = 0;
  graph[goal_index].g = 0;
  graph[goal_index].h = 0;
  graph[goal_index].f = graph[goal_index].g + graph[goal_index].h;
  int g = 0;
  
  // bool path_found = false;
  // int goal = goal_index;

  thrust::host_vector<int> q_lists;
  q_lists.push_back(goal_index);

  const int map_size = n*n*sizeof(planner::Node);

  planner::Node *map_gpu;

  int neighbor[8] = {1, -1, n, -n, n+1, n-1, -n+1, -n-1};

  //Copy all needed variables to gpu
  cudaMalloc( (void**)&map_gpu, map_size );
  cudaMemcpy(map_gpu, graph, map_size, cudaMemcpyHostToDevice);

  // cudaMemcpyToSymbol(path_found_gpu, &path_found,  sizeof(bool));
  cudaMemcpyToSymbol(neighbor_gpu, &neighbor,  8*sizeof(int));
  cudaMemcpyToSymbol(g_gpu, &g,  sizeof(int));

  //q list on gpu
  thrust::device_vector<int> q_lists_gpu = q_lists;

  while(q_lists_gpu.size()!=0){


    int q_size = q_lists_gpu.size();

    //new_q is the list store the frontier generated from this step of exploration
    thrust::device_vector<int> new_q_lists_gpu(8*q_size);
    thrust::fill(new_q_lists_gpu.begin(), new_q_lists_gpu.end(), -1);

    g = g+5;
    cudaMemcpyToSymbol(g_gpu, &g,  sizeof(int));
  


    //Determine how many thread should be launched
    int thread_size = min(max_thread, q_size);

    

    //Launch the kernel to explore the map
    explore<<<1,thread_size>>>(thrust::raw_pointer_cast(q_lists_gpu.data()),  map_gpu, thrust::raw_pointer_cast(new_q_lists_gpu.data()));
    cudaDeviceSynchronize();
    

    

    // // Remove all element that is not used during the exploration and repeated value

    
    
    new_q_lists_gpu.erase(thrust::remove_if(new_q_lists_gpu.begin(), new_q_lists_gpu.end(), is_negative()),  new_q_lists_gpu.end() );
    
    new_q_lists_gpu.erase(thrust::unique(new_q_lists_gpu.begin(), new_q_lists_gpu.end()), new_q_lists_gpu.end() );

    // std::cout<< new_q_lists_gpu.size() << std::endl;

    q_lists_gpu.clear();
 
    q_lists_gpu = new_q_lists_gpu;
    new_q_lists_gpu.clear();

    
    // if (new_q_lists_gpu.empty()) break;
    
    // // Create new q list based on origional and updated q
    // if (q_size <= max_thread) {
    //   q_lists_gpu.clear();
 
    //   q_lists_gpu = new_q_lists_gpu;
    //   new_q_lists_gpu.clear();
    //   }
    // else {
      
    //   q_lists_gpu.erase(q_lists_gpu.begin(), q_lists_gpu.begin()+max_thread );
    //   q_lists_gpu.insert(q_lists_gpu.end(), new_q_lists_gpu.begin(), new_q_lists_gpu.end() );
    //   new_q_lists_gpu.clear();

      
    //   }

    
    //q_size = q_lists_gpu.size();
    // thrust::device_vector<float> h_value(q_size);

    // if (q_size > 1024) {
    //   int block = q_size / 1024 + 1;
      
    //   get_h<<<block, 1024>>>(thrust::raw_pointer_cast(q_lists_gpu.data()),  map_gpu, thrust::raw_pointer_cast(&h_value[0]), q_size );

    //   thrust::sort_by_key(h_value.begin(), h_value.end(), q_lists_gpu.begin() );

    }

  cudaMemcpy(graph, map_gpu,  map_size, cudaMemcpyDeviceToHost );

    


 
}