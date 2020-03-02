#include <ros/ros.h>
#include <mavros_msgs/LandingTarget.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/transform_datatypes.h>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Transform.h>
#include <cmath>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <gazebo_msgs/LinkStates.h>
#include <gazebo_msgs/LinkState.h>

using namespace std;

ros::Publisher landing_target_raw_publisher;
mavros_msgs::LandingTarget landing_target_message;

ros::Publisher landing_pad_camera_odom_publisher;
ros::Publisher landing_pad_relative_pose_publisher;
ros::Publisher landing_pad_relative_pose_stamped_publisher;
ros::Publisher landing_pad_global_pose_publisher;
ros::Publisher landing_pad_global_pose_stamped_publisher;
ros::Publisher landing_pad_global_pose_filtered_publisher;
ros::Publisher setpoint_position_local_publisher;
ros::Publisher landing_pad_position_publisher;
ros::Publisher imu_data_forwarding_publisher;
ros::Publisher camera_imu_data_forwarding_publisher;
ros::Publisher visual_odometry_publisher;

geometry_msgs::PoseStamped landing_pad_camera_pose;
geometry_msgs::Pose landing_pad_relative_pose;
geometry_msgs::Pose landing_pad_global_pose;
geometry_msgs::Pose landing_pad_global_pose_filtered;
geometry_msgs::PoseStamped landing_pad_relative_pose_stamped;
geometry_msgs::PoseStamped landing_pad_relative_pose_absolute_yaw_stamped;
geometry_msgs::PoseStamped landing_pad_global_pose_stamped;
geometry_msgs::PoseStamped local_position_pose_stamped;

nav_msgs::Odometry landing_pad_camera_odom_message;
nav_msgs::Odometry visual_odometry_message;
nav_msgs::Odometry visual_odometry_filtered_message;

sensor_msgs::Imu imu_buffer;
sensor_msgs::Imu camera_imu_buffer;

tf2_ros::Buffer transform_buffer;

// bit IDs of landing pad 
int landing_pad_id[2] = {1, 2};

// timing
ros::Time last_detection_time(0);
ros::Duration abort_time(0.5);
ros::Time last_target_send_time(0);
ros::Duration target_send_interval(0.5);

gazebo_msgs::LinkState link_state_message;
ros::Publisher landing_pad_estimate_publisher;

tf2::Quaternion camera_imu_orientation;

void odometry_filtered_callback(const nav_msgs::Odometry::ConstPtr msg)
{
	landing_pad_global_pose_filtered = msg->pose.pose;
	landing_pad_global_pose_filtered_publisher.publish(landing_pad_global_pose_filtered);
}


void local_position_pose_callback(const geometry_msgs::PoseStamped::ConstPtr msg)
{
	// this message is in END (east, north, down)
	local_position_pose_stamped = *msg;

	static tf2_ros::TransformBroadcaster transform_broadcaster;
	geometry_msgs::TransformStamped transform_stamped_message;
	transform_stamped_message.header.stamp = ros::Time::now();
	transform_stamped_message.header.frame_id = "global_ned";
	transform_stamped_message.child_frame_id = "local_ned_absolute_yaw";

	// we go from END to NED
	transform_stamped_message.transform.translation.x = msg->pose.position.y;
	transform_stamped_message.transform.translation.y = msg->pose.position.x;
	transform_stamped_message.transform.translation.z = msg->pose.position.z;
	transform_stamped_message.transform.rotation.w = 1;
	transform_stamped_message.transform.rotation.x = 0;
	transform_stamped_message.transform.rotation.y = 0;
	transform_stamped_message.transform.rotation.z = 0;

	transform_broadcaster.sendTransform(transform_stamped_message);
}

void imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
	imu_buffer = *msg;
	tf2::convert(msg->orientation, imu_buffer.orientation);
	imu_buffer.header.frame_id = "imu_link";
	imu_data_forwarding_publisher.publish(imu_buffer);

	static tf2_ros::TransformBroadcaster transform_broadcaster;
	geometry_msgs::TransformStamped transform_stamped_message;
	transform_stamped_message.header.stamp = ros::Time::now();
	transform_stamped_message.header.frame_id = "local_ned_absolute_yaw";
	transform_stamped_message.child_frame_id = "local_ned";

	tf2::Quaternion orientation, yaw;
	tf2::fromMsg(msg->orientation, orientation);

	double y, p, r;
	tf2::Matrix3x3(orientation).getEulerYPR(y, p, r);

	yaw.setRPY(0, 0, -y);
	geometry_msgs::Quaternion yaw_message = tf2::toMsg(yaw);

	transform_stamped_message.transform.rotation = yaw_message;
	transform_broadcaster.sendTransform(transform_stamped_message);
}

void camera_imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
	camera_imu_buffer = *msg;
	tf2::convert(msg->orientation, camera_imu_buffer.orientation);
	tf2::convert(msg->orientation, camera_imu_orientation);
	camera_imu_buffer.header.frame_id = "camera_imu_frame";
	camera_imu_data_forwarding_publisher.publish(camera_imu_buffer);

	static tf2_ros::TransformBroadcaster transform_broadcaster;
	geometry_msgs::TransformStamped transform_stamped_message;
	transform_stamped_message.header.stamp = ros::Time::now();
	transform_stamped_message.header.frame_id = "base_link";
	transform_stamped_message.child_frame_id = "camera_imu_frame";
	transform_stamped_message.transform.rotation = msg->orientation;
	transform_broadcaster.sendTransform(transform_stamped_message);
}

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
		landing_pad_camera_pose.header.stamp = ros::Time::now();
		landing_pad_camera_pose.header.frame_id = "camera_frame";
		landing_pad_camera_pose.pose = buffer;
		
		landing_pad_camera_pose.pose.position.x = buffer.position.z;
		landing_pad_camera_pose.pose.position.y = buffer.position.x;
		landing_pad_camera_pose.pose.position.z = -buffer.position.y;
		
		last_detection_time = ros::Time::now();
	}
}

// local NED is unusable because it does implicate GPS
// used with global yaw
int seq = 0;
void send_target_position_local_pose_stamped()
{
	seq ++;

//	geometry_msgs::PoseStamped buffer = landing_pad_relative_pose_stamped;
	geometry_msgs::PoseStamped buffer = landing_pad_global_pose_stamped;
//	geometry_msgs::PoseStamped buffer = landing_pad_relative_pose_absolute_yaw_stamped;
	geometry_msgs::PoseStamped buffer2 = buffer;

	buffer.header.seq   = seq;
	buffer.header.stamp = ros::Time::now();
	buffer.header.frame_id = "12";

//	buffer.pose.orientation = landing_pad_relative_pose.orientation;
	
	// relative pose is in NWD, but we need ENU
//	buffer.pose.position.x = landing_pad_relative_pose.position.y;
//	buffer.pose.position.y = landing_pad_relative_pose.position.x;
/*
	buffer.pose.position.x += local_position_pose_stamped.pose.position.x;
	buffer.pose.position.y += local_position_pose_stamped.pose.position.y;
	buffer.pose.position.z += local_position_pose_stamped.pose.position.z;
*/
	buffer.pose.position.x = buffer2.pose.position.y;
	buffer.pose.position.y = buffer2.pose.position.x;
	buffer.pose.position.z = 10;
	
	setpoint_position_local_publisher.publish(buffer);
}

int main(int argc, char** argv)
{
	// init and say hi
	ros::init(argc, argv, "landing_controller");
	ROS_INFO("Started landing_controller!");

	// create node handle
	ros::NodeHandle node_handle;
	
	// create subscribers
        ros::Subscriber visual_subscriber = node_handle.subscribe("/whycon_ros/visual", 1000, visual_callback);
	ros::Subscriber camera_imu_subscriber = node_handle.subscribe("/iris/camera/imu", 1000, camera_imu_callback);
	ros::Subscriber imu_subscriber = node_handle.subscribe("/iris/imu", 1000, imu_callback);
	ros::Subscriber local_position_pose_subscriber = node_handle.subscribe("/mavros/local_position/pose", 1000, local_position_pose_callback);
	
	// create publishers
	landing_pad_relative_pose_publisher = node_handle.advertise<geometry_msgs::Pose>("/landing_pad/relative_pose", 1000);
	landing_pad_global_pose_publisher = node_handle.advertise<geometry_msgs::Pose>("/landing_pad/global_pose", 1000);
	landing_pad_global_pose_stamped_publisher = node_handle.advertise<geometry_msgs::PoseStamped>("/landing_pad/global_pose_stamped", 1000);
	landing_pad_relative_pose_stamped_publisher = node_handle.advertise<geometry_msgs::PoseStamped>("/landing_pad/relative_pose_stamped", 1000);
	landing_pad_global_pose_filtered_publisher = node_handle.advertise<geometry_msgs::Pose>("/landing_pad/global_pose_filtered", 1000);
	setpoint_position_local_publisher = node_handle.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 1000);

	// for filtering
	landing_pad_camera_odom_publisher = node_handle.advertise<nav_msgs::Odometry>("/odometry/camera/landing_pad_pose", 1000);
	imu_data_forwarding_publisher = node_handle.advertise<sensor_msgs::Imu>("/imu_data", 1000);
	visual_odometry_publisher = node_handle.advertise<nav_msgs::Odometry>("/visual_odometry", 1000);
	ros::Subscriber odometry_filtered_subscriber = node_handle.subscribe("/odometry/filtered", 1000, odometry_filtered_callback);

	// for transforms
	static tf2_ros::TransformListener transform_listener(transform_buffer);

	ros::Rate loop_rate(40);
	while( ros::ok() )
	{
		// callbacks
		ros::spinOnce();
	
		if( ros::Time::now() - last_detection_time < abort_time )
		{
			// transform pose in camera frame to pose relative to the drone in NED
			try
			{
				landing_pad_relative_pose_stamped 		= transform_buffer.transform(landing_pad_camera_pose, "local_ned", ros::Duration(0.1));
				landing_pad_relative_pose_absolute_yaw_stamped	= transform_buffer.transform(landing_pad_relative_pose_stamped, "local_ned_absolute_yaw", ros::Duration(0.1));

				landing_pad_global_pose_stamped.pose.position.x = landing_pad_relative_pose_absolute_yaw_stamped.pose.position.x + local_position_pose_stamped.pose.position.y;
				landing_pad_global_pose_stamped.pose.position.y = -landing_pad_relative_pose_absolute_yaw_stamped.pose.position.y - local_position_pose_stamped.pose.position.x;
				landing_pad_global_pose_stamped.pose.position.z = max(local_position_pose_stamped.pose.position.z - landing_pad_relative_pose_absolute_yaw_stamped.pose.position.z, 0.1);
			}
			catch( tf2::TransformException &exception )
			{
				ROS_WARN("%s", exception.what());
			}

			// publish relative pose
			landing_pad_relative_pose_publisher.publish(landing_pad_relative_pose);
			landing_pad_relative_pose_stamped_publisher.publish(landing_pad_relative_pose_stamped);
			landing_pad_global_pose_stamped_publisher.publish(landing_pad_global_pose_stamped);

			// Direct the drone towards the landing pad
			
			if( ros::Time::now() - last_target_send_time >= ros::Duration(0.1) )
			{
				last_target_send_time = ros::Time::now();
			}
		}

		loop_rate.sleep();
	}

	return 0;
}
