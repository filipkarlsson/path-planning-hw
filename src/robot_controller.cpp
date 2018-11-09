#include <iostream>
#include <sstream>


#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_msgs/String.h>


/* 	robot prolly has some kind of sensor
		->	sensor detects obstacles
			-> So robot uh, subscribes to it, mayyybeeee333e
		-> robot then calls some quadtree shit
		-> RRT on the quadtreeeee maybe I should review the notes

*/

int main(int argc, char **argv)
{
	ros::init(argc, argv, "robot_controller");
	ros::NodeHandle node_handle;
	ros::Publisher velocity_publisher = node_handle.advertise<geometry_msgs::Twist>("/cmd_vel", 1); // TODO: Look into this more

	// send twist commands
	geometry_msgs::Twist base_cmd;
	ros::Rate loop_rate(10);

	while(ros::ok())
	{
		// TODO: Kinematics

		base_cmd.linear.x = 0.25;

		velocity_publisher.publish(base_cmd);

		/// Sleep for as long as needed to achieve the loop rate.
		loop_rate.sleep();
		
	}

	
	

	return 0;
}