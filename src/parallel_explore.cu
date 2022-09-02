

#include <time.h>  
#include <vector>
#include "graph_search/planner1.h"
#include "graph_search/parallel_explore.cuh"
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
    for (int i=0; i<4; i++)
    {   
      
      int new_index = explored_index + neighbor_gpu[i];
      bool edge_detect = true;

      
                
      if ((explored_index%n ==0 && neighbor_gpu[i] == -1) || (explored_index%(n-1) ==0 && neighbor_gpu[i] == 1 &&explored_index!=0) || new_index<0 || new_index >= n*n){
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
extern "C"
void parallel_explore(planner::Node* graph, int n, bool path_found, int start_index, int max_thread){

    
    thrust::host_vector<int> q_lists;
    q_lists.push_back(start_index);

    const int map_size = n*n*sizeof(planner::Node);

    planner::Node *map_gpu;

    int neighbor[4] = {1, -1, n, -n};

    cudaMalloc( (void**)&map_gpu, map_size );
    cudaMemcpy(map_gpu, &graph, map_size, cudaMemcpyHostToDevice);

    cudaMemcpyToSymbol(path_found_gpu, &path_found,  sizeof(bool));
    cudaMemcpyToSymbol(neighbor_gpu, &neighbor,  4*sizeof(int));
    // cudaMemcpyToSymbol(goal_gpu, &goal,  sizeof(int));


    thrust::device_vector<int> q_lists_gpu = q_lists;
    thrust::device_vector<float> f_value(q_lists_gpu.size());
    get_f<<<1, q_lists_gpu.size()>>>(thrust::raw_pointer_cast(q_lists_gpu.data()),  map_gpu, thrust::raw_pointer_cast(f_value.data()), q_lists_gpu.size() );
    

}
