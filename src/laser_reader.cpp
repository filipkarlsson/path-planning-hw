#include <iostream>
#include <sstream>


#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>
#include <std_msgs/String.h>

#include "laser_reader.h"


/*
	Apparently, stage publishes laser scanner data to
		/base_scan

	SO laser_reader needs to SUBSCRIBE to /base_scan




~/ros_ws » rosmsg show sensor_msgs/LaserScan                lappytoppy@lappytop
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
float32 angle_min
float32 angle_max
float32 angle_increment
float32 time_increment
float32 scan_time
float32 range_min
float32 range_max
float32[] ranges
float32[] intensities

*/

// Somewhat redundant file; demonstrates subscribing to laser scan topic data (published by stage) and republishing that



//sensor_msgs::LaserScan laser_scan;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "laser_reader");
    ros::NodeHandle node_handle;

    // send twist commands

    ros::Rate loop_rate(60);

    while(ros::ok())
    {
        // TODO: Kinematics


        
    }

    return 0;

}



LaserReader::LaserReader()
{
	// publish the laser scan
	// this will make an entry: topic: laser_scan // publisher: laser_reader (.cpp)
	laser_scan_subscriber = node_handle.subscribe<sensor_msgs::LaserScan>("base_scan", 60, &LaserReader::laser_scan_callback, this);
	laser_scan_publisher = node_handle.advertise<sensor_msgs::LaserScan>("laser_scan", 60);
}	


void LaserReader::laser_scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	double angle_min = scan->angle_min;
	double angle_max = scan->angle_max;
	double angle_increment = scan->angle_increment;
	int num_readings = (angle_max - angle_min) / angle_increment;
	//laser_scan.header = scan->header;
	//laser_scan.angle_min = scan->angle_min;

	ROS_INFO_STREAM("Assert");

}


void LaserReader::scan_world()
{
}






/*

std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
float32 angle_min
float32 angle_max
float32 angle_increment
float32 time_increment
float32 scan_time
float32 range_min
float32 range_max
float32[] ranges
float32[] intensities

*/
