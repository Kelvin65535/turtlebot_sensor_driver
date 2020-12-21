#include <iostream>
#include <std_msgs/Int32.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "lidar_driver.h"
using namespace std;

bool is_scan_stop = false;
bool is_motor_stop = false;
string laser_link = "laser_link";
lidar_driver driver;

void publish_scan(ros::Publisher *pub, double *dist, int count, ros::Time start, double scan_time)
{
	static int scan_count = 0;
	sensor_msgs::LaserScan scan_msg;
	scan_msg.header.stamp = start;
	scan_msg.header.frame_id = laser_link;
	scan_count++;

	scan_msg.angle_min = 0;
	scan_msg.angle_max = 2 * M_PI;
	scan_msg.angle_increment = (scan_msg.angle_max - scan_msg.angle_min) / (double) (count - 1);
	scan_msg.scan_time = scan_time;
	scan_msg.time_increment = scan_time / (double) (count - 1);

	scan_msg.range_min = 0.15;
	scan_msg.range_max = 6.0;

	scan_msg.intensities.resize(count);
	scan_msg.ranges.resize(count);

	for (int i = count - 1; i >= 0; i--)
	{
		if (dist[count - i - 1] == 0.0)
			scan_msg.ranges[i] = std::numeric_limits<float>::infinity();
		else
			scan_msg.ranges[i] = dist[count - i - 1] / 1000.0;
		//scan_msg.intensities[i] = 0;
	}
	pub->publish(scan_msg);
}

void startStopCB(const std_msgs::Int32ConstPtr msg)
{
	Command cmd = (Command) msg->data;
	int res = 0;
	switch (cmd)
	{
	case STOP_DATA:
		if (!is_scan_stop)
		{
			res = driver.SendLidarCommand(STOP_DATA);
			if (res >= 0)
			{
				is_scan_stop = true;
				ROS_INFO("scan stopped");
			}
		}
		break;
	case STOP_MOTOR_AND_DATA:
		if (!is_scan_stop)
		{
			res = driver.SendLidarCommand(STOP_MOTOR_AND_DATA);
			if (res >= 0)
			{
				is_scan_stop = true;
				ROS_INFO("scan stopped");
			}
		}
		if (!is_motor_stop)
		{
			res = driver.SendLidarCommand(STOP_MOTOR);
			if (res >= 0)
			{
				is_motor_stop = true;
				ROS_INFO("motor stopped");
			}
		}
		break;
	case START_MOTOR_AND_DATA:
		if (is_scan_stop)
		{
			res = driver.SendLidarCommand(START_SCAN);
			if (res >= 0)
			{
				ROS_INFO("scan started");
				is_scan_stop = false;
				is_motor_stop = false;
			}
		}
		break;
	default:
		ROS_WARN("Unkonw command: %d ", cmd);
		break;
	}
}

int main(int argv, char **argc)
{
	ros::init(argv, argc, "laser_node");
	ros::NodeHandle n;

	string scan_topic = "scan";
	ros::param::get("~scan_topic", scan_topic);
	ros::param::get("~laser_link", laser_link);
	ros::Publisher scan_pub = n.advertise<sensor_msgs::LaserScan>(scan_topic, 1000);
	ros::Subscriber stop_sub = n.subscribe<std_msgs::Int32>("startOrStop", 10, startStopCB);

	string port = "/dev/ttyUSB0";
	ros::param::get("~serial_port", port);
    
    
	while (driver.OpenLidarSerial(port.c_str(), B230400) < 0)
	{

		ROS_WARN_THROTTLE(1, "could not open port:%s", port.c_str());
		usleep(500000);
		if(!ros::ok())
			return 0;
	}

	int res = driver.SendLidarCommand(START_SCAN);


	while (ros::ok() && res < 0)
	{

		ROS_WARN_THROTTLE(1, "could not start scan, will try again.");
		usleep(200000);
		res = driver.SendLidarCommand(START_SCAN);
	}

	double angle[PACKLEN + 10];
	double distance[PACKLEN + 10];
	double data[PACKLEN + 10];
	double speed;
	int count = 0;
	
	ros::Time starts = ros::Time::now();
	ros::Time ends = ros::Time::now();
	ROS_INFO("talker....");
    
	while (ros::ok())
	{
		ros::spinOnce();

		if (is_scan_stop)
			continue;

		memset(data, 0, sizeof(data));
		res = driver.GetLidarScanData(angle, distance, PACKLEN, &speed);
                         // ROS_ERROR("I IM HEREres:%d\n",res);
		if (res <= 0)
		{
			ROS_WARN_THROTTLE(1, "Could not get scan data with res = %d, will try again.", res);
			if (res < 0)
			{
				while (ros::ok() && driver.OpenLidarSerial(port.c_str(), B230400) < 0)
				{
					ROS_WARN_THROTTLE(1, "could not open serial, will try again.");
					usleep(500000);
				}
			}
			while (ros::ok() && driver.SendLidarCommand(START_SCAN) < 0)
			{
				ROS_WARN_THROTTLE(1, "could not start scan, will try again.");
				usleep(500000);
			}
			continue;
		}
		ROS_INFO_THROTTLE(2, "talker");
		for (int i = 0; i < res; i++)
		{
			data[i] = distance[i];
		}
		ends = ros::Time::now();
		float scan_duration = (ends - starts).toSec() * 1e-3;
		publish_scan(&scan_pub, data, res, starts, scan_duration);
		starts = ends;
	}

	driver.SendLidarCommand(STOP_DATA);
	driver.CloseLidarSerial();

	return 0;
}

