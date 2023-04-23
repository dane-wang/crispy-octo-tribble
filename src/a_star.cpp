#include <ros/ros.h> 
//#include <geometry_msgs>
// #include "geometry_msgs/Point.h"
#include <time.h>  
#include <vector>
#include "graph_search/my_msg.h"
#include "graph_search/planner1.h"
#include <algorithm>
#include <xmlrpcpp/XmlRpcValue.h>
#include <cmath>
#include <queue>
#include "graph_search/parallel_dijkstra.cuh"

extern "C" void parallel_dijkstra(planner::Node* graph, int n, int goal_index, int max_thread);

int main (int argc, char **argv) 
{ 
    // 初始化ROS节点 节点名字
	ros::init (argc, argv, "start_and_goal"); 
	
    //generate map info from the config file
    int n, max_thread_size;
    std::vector<int> start_coord, goal_coord;
    std::vector<int> obstacles;
    XmlRpc::XmlRpcValue xml_obstacles;

    ros::param::get("map_size", n);
    ros::param::get("start_position", start_coord);
    ros::param::get("goal_position", goal_coord);
    ros::param::get("obstacles", xml_obstacles);
    // ros::param::get("max_thread", max_thread_size);

    // Initialize the start and goal node
    int start = start_coord[0]+start_coord[1] * n;
    int goal = goal_coord[0]+goal_coord[1] * n;

    // Initialize the obstacles list
    for(int i=0; i< xml_obstacles.size(); i++){
        int obstacles_index =  (int)xml_obstacles[i][0] +  (int)xml_obstacles[i][1] * n;
        obstacles.push_back( obstacles_index);
    }

    //Generate the map
    planner::Node* graph = new planner::Node[n*n];

    planner::map_generation(&graph[0], n, start, goal, obstacles);
    
    bool path_found = false;

    // parallel_dijkstra(&graph[0], n, goal, max_thread_size);

    graph[start].f = graph[start].g + graph[start].h;


    std::priority_queue< std::vector<float>, std::vector< std::vector<float> >, planner::priority_queue_compare > q_list;

    q_list.push({(float) start, graph[start].f});
    // std::cout << graph[start].f << std::endl;

    int neighbors[8][2] = {{0,1}, {0,-1}, {1,0}, {-1,0}, {1,1}, {1,-1}, {-1,1}, {-1,-1}};

    int neighbor[16];

    for (int i =0; i< 8; i++){
        for (int j=0; j<2; j++){

        neighbor[2*i+j] = neighbors[i][j];



        }

    }

    // 节点句柄
	ros::NodeHandle nh; 

    // 发布消息 话题名字 队列大小
	ros::Publisher pub = nh.advertise<graph_search::my_msg> ("planning_info", 100);
    
    //geometry_msgs::Point start_goal;
    graph_search::my_msg map;

    //int g = 0;
    
    
    while (ros::ok()) {

        while(ros::ok() && q_list.size()!=0 && !path_found){
            // pop the node with smallest node
            auto smallest_node = q_list.top();
            q_list.pop();
            int explored_index = smallest_node[0];

            if (graph[explored_index].explored) continue;


            int explored_coord[2];
            explored_coord[0] = explored_index%n;
            explored_coord[1] = explored_index/n;


            //std::cout << explored_index << std::endl;

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
                for (int i=0; i<8; i++)
                {
                    int neighbor1[2];
                    neighbor1[0] = neighbor[2*i];
                    neighbor1[1] = neighbor[2*i+1];

                    int new_coord[2];
                    new_coord[0] = explored_coord[0] + neighbor1[0];
                    new_coord[1] = explored_coord[1] + neighbor1[1];


                    int new_index = new_coord[0] + new_coord[1]*n;

                    float cost = (i < 4) ? 1 : sqrt(2);

                    if (new_index<0 || new_index >= n*n) continue;


                    //Check if the new index possible (like if it will go out of the map)
                    bool edge_detect = true;

                    if ((new_coord[0] >= n) || (new_coord[0] < 0)  || (new_coord[1] >= n) || (new_coord[1] <0 )){

                        edge_detect = false;
                    }



                    if (graph[new_index].obstacle == false && graph[new_index].frontier == false && graph[new_index].explored == false && edge_detect)
                    {
                        graph[new_index].g = graph[explored_index].g + cost;
                        if (graph[new_index].h == INFINITY) graph[new_index].h = planner::h_calculation(&graph[new_index], &graph[goal]);
                        graph[new_index].f = graph[new_index].h + graph[new_index].g;
                        graph[new_index].parent = explored_index;
                        graph[new_index].frontier = true;

                        q_list.push({(float) new_index, graph[new_index].f});
                    }
                    else if (edge_detect && graph[new_index].obstacle == false && (graph[new_index].frontier == true || graph[new_index].explored == true))
                    {
                        if (graph[new_index].g > graph[explored_index].g + cost)
                        {
                            graph[new_index].g = graph[explored_index].g + cost;
                            graph[new_index].f = graph[new_index].h + graph[new_index].g;
                            graph[new_index].parent = explored_index;
                            
                            if (graph[new_index].explored == true) {
                                graph[new_index].explored == false;
                                q_list.push({(float) new_index, graph[new_index].f});
                            }
                        }
                    }
                    

                }
            
                // std::sort(q_list.begin(), q_list.end(), planner::sortcol);
            }
            else{
                int path1 = goal;
                while (path1 != start)
                {
                    graph[path1].path = true;
                    path1 = graph[path1].parent;
                }
                


            }
            //std::cout << q_list.size() << std::endl;
            


            //Create the message to publish
            //Assign values based on grip status. Help visualization
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
            ros::Rate loop_rate(10);
            
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
