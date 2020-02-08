#include <ros/ros.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "landing_controller");
	ros::NodeHandle node_handle;
	ROS_INFO("Hello, ROS World!");

	ros::Rate loop_rate(1);
	while( ros::ok() )
	{
//		ROS_INFO("Spinning...");

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
