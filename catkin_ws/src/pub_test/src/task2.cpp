#include "ros/ros.h" 
#include "std_msgs/Int32.h" 
#include <iostream> 
void number_callback(const std_msgs::Int32::ConstPtr& msg)
{       
	printf("message from Arduino is %d \n",   msg->data);
} 
int main(int argc, char **argv)
{       
	ros::init(argc, argv, "task2");      
	ros::NodeHandle node_obj;      

	ros::Publisher number_publisher = node_obj.advertise<std_msgs::Int32>("/numbers", 1);
    ros::Subscriber number_subscriber = node_obj.subscribe("/return_numbers",  0, number_callback);

  	ros::Rate loop_rate(0.3);
	int number_count = 0;
  	while (ros::ok())
  	{   
                  
		std_msgs::Int32 msg;
		msg.data = number_count;
    	printf("user's input is %d \n", msg.data);
    	number_publisher.publish(msg);
    	//ros::spinOnce();
        loop_rate.sleep();
    	++number_count;      
	}
    ros::spin();
 	return 0;
}

