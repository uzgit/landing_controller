#include <ros/ros.h>
#include <mavros_msgs/LandingTarget.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>

#include <cmath>

using namespace std;

enum LANDING_MODE
{
	NONE = 0,	// no landing markers detected
	GUIDED,		// landing markers detected, but still not in landing mode
	APPROACH,	// landing markers detected, initial approach
	SUB_EPSILON,	// close in x and y dimensions
	FIX,		// fixed with small x/y distances
	DESCENT,	// fixed and descending
	TOUCHDOWN,	// on the landing pad
	ENGINE_STOP,	// landed
	ABORT		// landing failure
};

mavros_msgs::LandingTarget landing_target_message;
ros::Publisher landing_target_publisher;
ros::Publisher landing_target_publisher_pose_stamped;
int frame_id = 1;

int landing_pad_id[2] = {1, 2};
ros::Time last_detection_time(0);
ros::Duration abort_time(0.5);

geometry_msgs::Pose landing_pad_pose;

void landing_pad_pose_callback(const geometry_msgs::Pose::ConstPtr& msg)
{
	landing_pad_pose = *msg;
	last_detection_time = ros::Time::now();
}

void send_landing_target_message()
{
	ROS_INFO("Sending landing target message!");

	landing_target_message.header.stamp.sec = (int)ros::Time::now().sec;
	landing_target_message.header.stamp.nsec = (int)ros::Time::now().nsec;
	landing_target_message.header.frame_id = 149;
	landing_target_message.target_num = 1;
	landing_target_message.frame = mavros_msgs::LandingTarget::LOCAL_NED;
	landing_target_message.type = mavros_msgs::LandingTarget::VISION_FIDUCIAL;
	landing_target_message.angle[0] = atan(landing_pad_pose.position.x / landing_pad_pose.position.z);
	landing_target_message.angle[1] = atan(landing_pad_pose.position.y / landing_pad_pose.position.z);
	landing_target_message.distance = sqrt( pow(landing_pad_pose.position.x, 2) + pow(landing_pad_pose.position.y, 2) + pow(landing_pad_pose.position.z, 2) );

	landing_target_message.size[0] = 0.3;
	landing_target_message.size[1] = 0.3;

	frame_id ++;

	landing_target_publisher.publish(landing_target_message);

	geometry_msgs::PoseStamped landing_target_message_pose_stamped;
	landing_target_message_pose_stamped.header.frame_id = frame_id;
	landing_target_message_pose_stamped.header.stamp    = ros::Time::now();
	landing_target_message_pose_stamped.pose = landing_pad_pose;

//	landing_target_publisher_pose_stamped.publish(landing_target_message_pose_stamped);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "landing_controller");
	ros::NodeHandle node_handle;
	ROS_INFO("Started landing_controller!");

	// Subscriber to get position(s) of landing pad marker(s)
        // ros::Subscriber visual_subscriber = node_handle.subscribe("/whycon_ros/visual", 1000, visual_callback);
	ros::Subscriber landing_pad_pose_subscriber = node_handle.subscribe("landing_pad/relative_pose", 1000, landing_pad_pose_callback);
	landing_target_publisher = node_handle.advertise<mavros_msgs::LandingTarget>("/mavros/landing_target/raw", 1000);
	// landing_target_publisher_pose_stamped = node_handle.advertise<mavros_msgs::LandingTarget>("/mavros/landing_target/pose_in", 1000);

	ros::Rate loop_rate(7);
	while( ros::ok() )
	{
//		ROS_INFO("Spinning...");

		ros::spinOnce();

		if( ros::Time::now() - last_detection_time < abort_time )
		{
			send_landing_target_message();
		}

		loop_rate.sleep();
	}

	return 0;
}
