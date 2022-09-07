

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
void parallel_explore(planner::Node* graph, int n, int start_index, int goal_index, int max_thread, std::vector<int>& path_to_goal){

  //Setup everything for planning
  bool path_found = false;
  int goal = goal_index;
  thrust::host_vector<int> q_lists;
  q_lists.push_back(start_index);

  const int map_size = n*n*sizeof(planner::Node);

  planner::Node *map_gpu;

  int neighbor[4] = {1, -1, n, -n};

  //Copy all needed variables to gpu
  cudaMalloc( (void**)&map_gpu, map_size );
  cudaMemcpy(map_gpu, graph, map_size, cudaMemcpyHostToDevice);

  cudaMemcpyToSymbol(path_found_gpu, &path_found,  sizeof(bool));
  cudaMemcpyToSymbol(neighbor_gpu, &neighbor,  4*sizeof(int));
  cudaMemcpyToSymbol(goal_gpu, &goal,  sizeof(int));

  //q list on gpu
  thrust::device_vector<int> q_lists_gpu = q_lists;

  while(q_lists_gpu.size()!=0 && !path_found){
    int q_size = q_lists_gpu.size();

    //new_q is the list store the frontier generated from this step of exploration
    thrust::device_vector<int> new_q_lists_gpu(4*q_size);
    thrust::fill(new_q_lists_gpu.begin(), new_q_lists_gpu.end(), -1);


    //Determine how many thread should be launched
    int thread_size = min(max_thread, q_size);


    //Launch the kernel to explore the map
    explore<<<1,thread_size>>>(thrust::raw_pointer_cast(q_lists_gpu.data()),  map_gpu, thrust::raw_pointer_cast(new_q_lists_gpu.data()));
    cudaDeviceSynchronize();
    cudaMemcpyFromSymbol(&path_found, path_found_gpu,  sizeof(bool), 0, cudaMemcpyDeviceToHost );
    // cudaMemcpy(&graph, map_gpu,  map_size, cudaMemcpyDeviceToHost );


    // Remove all element that is not used during the exploration and repeated value
    
    new_q_lists_gpu.erase(thrust::remove_if(new_q_lists_gpu.begin(), new_q_lists_gpu.end(), is_negative()),  new_q_lists_gpu.end() );
    
    new_q_lists_gpu.erase(thrust::unique(new_q_lists_gpu.begin(), new_q_lists_gpu.end()), new_q_lists_gpu.end() );
    
    // Create new q list based on origional and updated q
    if (q_size <= max_thread) {
      q_lists_gpu.clear();
      q_lists_gpu = new_q_lists_gpu;
      new_q_lists_gpu.clear();
    }
    else {
      
      q_lists_gpu.erase(q_lists_gpu.begin(), q_lists_gpu.begin()+max_thread );
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
      cudaMemcpy(graph, map_gpu,  map_size, cudaMemcpyDeviceToHost );
      int path1 = goal;
      while (path1 != start_index)
        {  
          path_to_goal.push_back(path1);
          graph[path1].path = true;
          // path.push_back(path1);
          path1 = graph[path1].parent;
        }


    }
  }


 
}
