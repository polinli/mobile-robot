#include "ros/ros.h" 
#include "std_msgs/Int32.h" 
#include <iostream> 

int main(int argc, char **argv)
{       
	ros::init(argc, argv, "task2_pub");      
	ros::NodeHandle node_obj;      

	ros::Publisher number_publisher = node_obj.advertise<std_msgs::Int32>("/numbers", 1);
    //ros::Subscriber number_subscriber = node_obj.subscribe("/return_numbers",  0, number_callback);

  	ros::Rate loop_rate(0.3);
	//int number_count = 1;
  	while (ros::ok())
  	{   
                  
		std_msgs::Int32 msg;
		//msg.data = number_count;
        printf("user's input is ");
    	scanf("%d", &msg.data);
    	number_publisher.publish(msg);
        ros::Duration(0.5).sleep();
    	ros::spinOnce();
        loop_rate.sleep();
    	//++number_count;      
	}
    //ros::spin();
 	return 0;
}

