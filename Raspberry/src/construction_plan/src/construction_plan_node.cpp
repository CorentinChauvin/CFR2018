#include <iostream>
#include <string>

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>


using namespace std;

void callback_raspicam_image(sensor_msgs::Image image)
{
	ROS_INFO("An image has been taken");
}

//ros::package::getPath("evolutive_map")


int main(int argc, char** argv)
{
	// ROS Initialization
	ros::init(argc, argv, "construction_plan");
	ROS_INFO("Node construction_plan connected to roscore");
	ros::NodeHandle nh_("~");//ROS Handler - local namespace.

	// Subscribing
	ros::Subscriber sub_raspicam_photo = nh_.subscribe<sensor_msgs::Image> ("/raspicam/image", 1, callback_raspicam_image);

	// Publishing
	ros::Publisher pub_take_photo = nh_.advertise<std_msgs::Float64>("/raspicam/take_photo", 1);

	// Main loop
	//std_msgs::Empty emptyMessage;
	std_msgs::Float64 emptyMessage;
	
	ros::Rate rate(1);
	while (ros::ok())
	{
		ros::spinOnce();

		pub_take_photo.publish(emptyMessage);

		rate.sleep();
	}
	
	
	//ros::spin();
	
	
}
