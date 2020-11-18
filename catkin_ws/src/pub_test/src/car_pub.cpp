#include "ros/ros.h" 
#include "std_msgs/Int32.h"
#include "std_msgs/Int8MultiArray.h" 
#include <iostream> 

int main(int argc, char **argv)
{       
	ros::init(argc, argv, "car_pub");      
	ros::NodeHandle node_obj;      

	ros::Publisher number_publisher = node_obj.advertise<std_msgs::Int8MultiArray>("/wheel_speed", 10);
    //ros::Subscriber number_subscriber = node_obj.subscribe("/return_numbers",  0, number_callback);

  	ros::Rate loop_rate(0.3);
	//int number_count = 1;
    int wheel_speed_left;
    int wheel_speed_right;

  	while (ros::ok())
  	{   
                  
		std_msgs::Int8MultiArray msg;
		msg.data.clear();

        printf("user's right is ");
    	scanf("%d", &wheel_speed_right);
        printf("user's left is ");
    	scanf("%d", &wheel_speed_left);
        msg.data.push_back(wheel_speed_left);
        msg.data.push_back(wheel_speed_right);

    	number_publisher.publish(msg);
        ros::Duration(0.5).sleep();
    	ros::spinOnce();
        loop_rate.sleep();
    	//++number_count;      
	}
    //ros::spin();
 	return 0;
}

