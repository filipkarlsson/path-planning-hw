#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>
#include <std_msgs/String.h>


class LaserReader
{
	public:
		
		
		LaserReader();
		void read_laser_scanner();
		void scan_world();

	private:
		void laser_scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan);
		ros::NodeHandle node_handle;
		ros::Publisher laser_scan_publisher;
		ros::Subscriber laser_scan_subscriber;
};