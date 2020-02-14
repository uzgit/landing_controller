#include <ros/ros.h>
#include <mavros_msgs/LandingTarget.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/transform_datatypes.h>
#include <sensor_msgs/Imu.h>

#include <tf2/LinearMath/Transform.h>

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

tf2::Transform drone_relative_transform;
tf2::Transform camera_relative_transform;

tf2::Quaternion landing_frame_quaternion;
tf2::Quaternion drone_relative_quaternion;
tf2::Quaternion imu_quaternion;
double imu_yaw, imu_pitch, imu_roll;
tf2::Quaternion camera_imu_quaternion;
double camera_yaw, camera_pitch, camera_roll;
tf2::Quaternion camera_relative_quaternion;
double camera_relative_yaw, camera_relative_pitch, camera_relative_roll;

geometry_msgs::Pose landing_pad_camera_pose;
geometry_msgs::Pose landing_pad_relative_pose;
geometry_msgs::Vector3 landing_pad_position_msg;
ros::Publisher landing_pad_position_publisher;

mavros_msgs::LandingTarget landing_target_message;
ros::Publisher landing_target_publisher;
int frame_id = 1;

int landing_pad_id[2] = {1, 2};
ros::Time last_detection_time(0);
ros::Duration abort_time(0.5);

void visual_callback(const visualization_msgs::MarkerArray::ConstPtr& msg)
{
	bool detection = false;

	int num_markers = msg->markers.size();
	double x, y, z;
	double rotation_x, rotation_y, rotation_z, rotation_w;

	geometry_msgs::Pose buffer;

	int i = 0;
	while( i < num_markers && ! detection )
	{
		int id = msg->markers[i].id;

		buffer = msg->markers[i].pose;

		x = msg->markers[i].pose.position.x;
		y = msg->markers[i].pose.position.y;
		z = msg->markers[i].pose.position.z;

		rotation_x = msg->markers[i].pose.orientation.x;
		rotation_y = msg->markers[i].pose.orientation.y;
		rotation_z = msg->markers[i].pose.orientation.z;
		rotation_w = msg->markers[i].pose.orientation.w;

		int ii = 0;
		while( ii < 2 && landing_pad_id[ii] != id )
		{
			ii ++;
		}
		if( ii < 2 )
		{
			detection = true;
		}

		i ++;
	}

	if( detection )
	{
		landing_pad_camera_pose = buffer;

		last_detection_time = ros::Time::now();
	}
}

void imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
        tf2::convert(msg->orientation, imu_quaternion);
        tf2::Matrix3x3(imu_quaternion).getEulerYPR(imu_yaw, imu_pitch, imu_roll);
}

void camera_imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
        tf2::convert(msg->orientation, camera_imu_quaternion);
        tf2::Matrix3x3(camera_imu_quaternion).getEulerYPR(camera_yaw, camera_pitch, camera_roll);
}

void send_landing_target_message()
{
	double magnitude = sqrt( pow(landing_pad_relative_pose.position.x, 2) + pow(landing_pad_relative_pose.position.y, 2) + pow(landing_pad_relative_pose.position.z, 2));

	landing_target_message.header.stamp.sec = (int)ros::Time::now().sec;
	landing_target_message.header.stamp.nsec = (int)ros::Time::now().nsec;
	landing_target_message.header.frame_id = 149;
	landing_target_message.target_num = 1;
	landing_target_message.frame = mavros_msgs::LandingTarget::LOCAL_NED;
	landing_target_message.type = mavros_msgs::LandingTarget::VISION_FIDUCIAL;
//	landing_target_message.angle[0] = asin( landing_pad_relative_pose.position.y / magnitude );
//	landing_target_message.angle[1] = asin( landing_pad_relative_pose.position.x / magnitude );
	landing_target_message.angle[0] = atan( landing_pad_relative_pose.position.y / landing_pad_relative_pose.position.z );
	landing_target_message.angle[1] = atan( landing_pad_relative_pose.position.x / landing_pad_relative_pose.position.z );
	landing_target_message.angle[0] *= -1;
	landing_target_message.angle[1] *= -1;
	ROS_INFO("landing target angle: (%0.3f, %0.3f)", landing_target_message.angle[0], landing_target_message.angle[1]);

	landing_target_message.distance = magnitude;
	landing_target_message.size[0] = 0.01;
	landing_target_message.size[1] = 0.01;

	frame_id ++;

	landing_target_publisher.publish(landing_target_message);
}

int main(int argc, char** argv)
{
	// init and say hi
	ros::init(argc, argv, "landing_controller");
	ROS_INFO("Started landing_controller!");

	// create subscribers and publishers
	ros::NodeHandle node_handle;
        ros::Subscriber visual_subscriber = node_handle.subscribe("/whycon_ros/visual", 1000, visual_callback);
	ros::Subscriber camera_imu_subscriber = node_handle.subscribe("/iris/camera/imu", 1000, camera_imu_callback);
	ros::Subscriber imu_subscriber = node_handle.subscribe("/iris/imu", 1000, imu_callback);
	landing_target_publisher = node_handle.advertise<mavros_msgs::LandingTarget>("/mavros/landing_target/raw", 1000);
	landing_pad_position_publisher = node_handle.advertise<geometry_msgs::Vector3>("/landing_pad/relative_position", 1000);

	// initialize landing frame quaternion
	// set quaternion using euler in yaw-pitch-roll format
	//				 (0,      pi/2,     0)
	landing_frame_quaternion.setEuler(0, 1.57079632679, 0);

	ros::Rate loop_rate(0.5);
	while( ros::ok() )
	{
		// callbacks
		ros::spinOnce();

		// calculate relevant rotations
		drone_relative_quaternion  = imu_quaternion * landing_frame_quaternion.inverse();
		drone_relative_quaternion.normalize();
		camera_relative_quaternion = camera_imu_quaternion * imu_quaternion.inverse();		// rotation from imu to camera
		camera_relative_quaternion.normalize();
	
		// construct transform objects
		camera_relative_transform = tf2::Transform(camera_relative_quaternion, tf2::Vector3(0, 0, 0));
		tf2::Matrix3x3(camera_relative_quaternion).getEulerYPR(camera_relative_yaw, camera_relative_pitch, camera_relative_roll);
		drone_relative_transform = tf2::Transform(drone_relative_quaternion, tf2::Vector3(0, 0, 0));

		// carry out transforms
		tf2::Vector3 position = tf2::Vector3(landing_pad_camera_pose.position.x, landing_pad_camera_pose.position.y, landing_pad_camera_pose.position.z);
		position = camera_relative_transform(position);
		position = drone_relative_transform(position);

/**************************************************************************************************************************************************************************/
//		ROS_INFO("imu quaternion: <%0.3f, %0.3f, %0.3f, %0.3f>", imu_quaternion.x(), imu_quaternion.y(), imu_quaternion.z(), imu_quaternion.w());
		ROS_INFO("imu orientation (YPR): <%0.3f, %0.3f, %0.3f>", imu_yaw, imu_pitch, imu_roll);
		ROS_INFO("camera orientation (YPR): <%0.3f, %0.3f, %0.3f>", camera_yaw, camera_pitch, camera_roll);
		ROS_INFO("camera relative orientation (YPR): <%0.3f, %0.3f, %0.3f>", camera_relative_yaw, camera_relative_pitch, camera_relative_roll);
//		ROS_INFO("cam quat relative orientation (YPR): <%0.3f, %0.3f, %0.3f>", temp_yaw, temp_pitch, temp_roll);
//		ROS_INFO("landing pad camera position: (%0.3f, %0.3f, %0.3f)", landing_pad_camera_pose.position.x, landing_pad_camera_pose.position.y, landing_pad_camera_pose.position.z);
//		ROS_INFO("landing pad relative position: (%0.3f, %0.3f, %0.3f)", maybe_relative_position.x(), maybe_relative_position.y(), maybe_relative_position.z());
//		ROS_INFO("<x, y>: <%0.3f, %0.3f>", angle_x, angle_y);
		ROS_INFO("landing pad position: <%0.3f, %0.3f, %0.3f>", position.x(), position.y(), position.z());
/**************************************************************************************************************************************************************************/
		if( ros::Time::now() - last_detection_time < abort_time )
		{
			landing_pad_relative_pose.position.x = position.x();
			landing_pad_relative_pose.position.y = position.y();
			landing_pad_relative_pose.position.z = position.z();

			landing_pad_position_publisher.publish(landing_pad_position_msg);	
		;
			send_landing_target_message();
		}

		loop_rate.sleep();
	}

	return 0;
}
