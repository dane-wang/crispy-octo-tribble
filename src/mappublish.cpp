#include <ros/ros.h> 
#include <nav_msgs/OccupancyGrid.h>
#include "geometry_msgs/Point.h"
#include <vector>
#include "graph_search/my_msg.h"


ros::Publisher pub;

void mapCallback (const graph_search::my_msg::ConstPtr& msg ){

    // 定义地图对象
    nav_msgs::OccupancyGrid myMap;
    

    // 地图数据 
    int n =100;
    //myMap.info.map_load_time = ros::Time(0);
    myMap.info.resolution = 0.5;
    myMap.info.width = n;
    myMap.info.height = n;
    myMap.info.origin.position.x = 0;
    myMap.info.origin.position.y = 0;
    myMap.info.origin.position.z = 0;
    myMap.info.origin.orientation.x = 0;
    myMap.info.origin.orientation.y = 0;
    myMap.info.origin.orientation.z = 0;
    myMap.info.origin.orientation.w = 0;

    //std::vector<int8_t> v(n*n, 0);

    
    
    myMap.data = msg->points;

    // myMap.data[msg.x] = 120;
    // myMap.data[msg.y] = 140;
 
    

    // frame id
    myMap.header.frame_id = "map";

    // 消息发布频率
    

    


    ros::Rate loop_rate(5);

	while (ros::ok()) 
	{ 
        // 广播
		pub.publish(myMap);
	    ros::spinOnce(); 
		loop_rate.sleep(); 
	 } 
	
}


int main (int argc, char **argv) 
{ 
    // 初始化ROS节点 节点名字
	ros::init (argc, argv, "mappublish"); 
	
    // 节点句柄
	ros::NodeHandle nh; 

    // 发布消息 话题名字 队列大小
	pub = nh.advertise<nav_msgs::OccupancyGrid> ("mappublish", 100);
    ros::Subscriber sub = nh.subscribe("start_and_goal",100,mapCallback);
    ros::spin();
    return 0; 

}

