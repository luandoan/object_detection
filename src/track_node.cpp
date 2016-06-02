#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
//#include "jackal_msgs/"
//#include "jackal_msgs/"
#include <sstream>
#include <iostream>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Float32MultiArray.h"


// declare global vars
void arrayCallback(const std_msgs::Int32MultiArray::ConstPtr& array);
int Arr[2];

int main(int argc, char **argv)
{
	int targetX=135;
	int targetY=150;
	int posGain=0.001;
	ros::init(argc, argv, "searching");
	ros::NodeHandle n;
	ros::Subscriber point_sec = n.subscribe("/point_pos", 100, arrayCallback);

	ros::Publisher chatter_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);
//	ros::Publisher 

	ros::Rate loop_rate(10);

	int count = 0;

	// loop one approadh the ball
	while (ros::ok())
	{
		std::cout << "x:  " << abs(targetY-Arr[1]) << std::endl;
		std::cout << "y:  " << abs(targetX-Arr[0]) << std::endl;

		geometry_msgs::Twist cmd_vel_msg_;

		cmd_vel_msg_.linear.x = 0;
		cmd_vel_msg_.linear.y = 0;
		cmd_vel_msg_.angular.z = 0;

		// moving robot for searching the position of target
		if (Arr[0] > 0)
		{
			if (abs(targetY-Arr[1]) > 2) cmd_vel_msg_.linear.x = (targetY-Arr[1])*posGain;
			if (abs(targetX-Arr[0]) > 2) cmd_vel_msg_.linear.y = (targetX-Arr[0])*posGain;
			std::cout << "Vel set " << std::endl;
		}

		// once we have reached the target location within tolerance break from loop
		if (abs(targetY-Arr[1]) < 2 && abs(targetX-Arr[0]) < 5)
		{
			break;
		}

		chatter_pub.publish(cmd_vel_msg_);

		// publish the pose
		float pose[6] = {0.591, 0.523, 0.533, 0.7997, 0.821, 0.904};
		std_msgs::Float32MultiArray msg;
		for (int i = 0; i < 6; i++)
		{
			msg.data.push_back(pose[i]);
		}

		ros::spinOnce();
		loop_rate.sleep();

		++count;
	}

	// end program
	return 0;
}

void arrayCallback(const std_msgs::Int32MultiArray::ConstPtr& array)
{
	int i = 0;
	// print all the remaining numbers
	for (std::vector<int>::const_iterator it = array->data.begin(); it != array -> data.end(); ++it)
	{
		Arr[i] = *it;
		i++;
	}
	return;
}
