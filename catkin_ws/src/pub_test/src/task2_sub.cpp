#include "ros/ros.h" 
#include "std_msgs/Int32.h" 
#include <iostream> 
void number_callback(const std_msgs::Int32::ConstPtr& msg)
{       
	printf("message from Arduino is %d \n",   msg->data);
} 
int main(int argc, char **argv)
{       
	ros::init(argc, argv, "task2_sub");      
	ros::NodeHandle node_obj;      

	//ros::Publisher number_publisher = node_obj.advertise<std_msgs::Int32>("/numbers", 1);
    ros::Subscriber number_subscriber = node_obj.subscribe("/return_numbers",  0, number_callback);
  	
    ros::spin();
 	return 0;
}

