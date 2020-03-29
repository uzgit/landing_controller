#include <ros/ros.h>
#include <mavros_msgs/PositionTarget.h>
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

using namespace std;

ros::Publisher landing_pad_camera_pose_publisher;
ros::Publisher landing_pad_relative_pose_stamped_publisher;
ros::Publisher landing_pad_global_pose_stamped_publisher;
ros::Publisher setpoint_raw_local_publisher;
ros::Publisher displacement_n_publisher;
ros::Publisher displacement_e_publisher;
ros::Publisher displacement_u_publisher;
ros::Publisher displacement_n_setpoint_publisher;
ros::Publisher displacement_e_setpoint_publisher;
ros::Publisher displacement_u_setpoint_publisher;
ros::Publisher pid_enable_n_publisher;
ros::Publisher pid_enable_e_publisher;
ros::Publisher pid_enable_u_publisher;

std_msgs::Float64 std_msgs_zero;
std_msgs::Float64 setpoint_n;
std_msgs::Float64 setpoint_e;
std_msgs::Float64 setpoint_u;
std_msgs::Bool std_msgs_true;
std_msgs::Bool std_msgs_false;

geometry_msgs::PoseStamped landing_pad_camera_pose;
geometry_msgs::PoseStamped landing_pad_whycon_pose;
geometry_msgs::PoseStamped landing_pad_apriltag_pose;
geometry_msgs::PoseStamped landing_pad_relative_pose_stamped;
geometry_msgs::PoseStamped landing_pad_relative_pose_stamped_straightened;
geometry_msgs::PoseStamped landing_pad_relative_pose_absolute_yaw_stamped;
geometry_msgs::PoseStamped landing_pad_global_pose_stamped;
geometry_msgs::PoseStamped local_position_pose_stamped;
geometry_msgs::Vector3 target_velocity;

tf2_ros::Buffer transform_buffer;

bool stopped = false;

// timing
ros::Time last_detection_time(0);
ros::Duration abort_time(0.5);
ros::Time last_target_send_time(0);
ros::Duration target_send_interval(0.5);

ros::Publisher landing_pad_estimate_publisher;

// remove rotation from a pose by rotating it by the inverse of its rotation
geometry_msgs::PoseStamped straighten_pose( const geometry_msgs::PoseStamped & _pose_in )
{
	// declare pose variables
	geometry_msgs::PoseStamped pose_out;
	geometry_msgs::PoseStamped pose_in = _pose_in;

	// access the transform broadcaster
	static tf2_ros::TransformBroadcaster transform_broadcaster;

	// declare a message in which to store the new transform
	geometry_msgs::TransformStamped transform_stamped_message;

	// set the transform's header
	transform_stamped_message.header.stamp = ros::Time::now();
	// make the parent frame_id equal to the child's frame_id with '_straightened' appended
	transform_stamped_message.header.frame_id = pose_in.header.frame_id + "_straightened";
	transform_stamped_message.child_frame_id = pose_in.header.frame_id;

	// invert the rotation
	tf2::Quaternion rotation, inverse_rotation;
	tf2::fromMsg(pose_in.pose.orientation, rotation);
	inverse_rotation = rotation.inverse();

	// set the rotation from the parent to the child
	transform_stamped_message.transform.rotation = tf2::toMsg(inverse_rotation);
//	transform_stamped_message.transform.rotation = pose_in.pose.orientation;

	// send the transform
	transform_broadcaster.sendTransform(transform_stamped_message);

	// transform the pose and return it
	return transform_buffer.transform(pose_in, transform_stamped_message.header.frame_id, ros::Duration(0.05));
}

/*
void local_position_pose_callback(const geometry_msgs::PoseStamped::ConstPtr msg)
{
	// this message is in END (east, north, down)
	local_position_pose_stamped = *msg;

	static tf2_ros::TransformBroadcaster transform_broadcaster;
	geometry_msgs::TransformStamped transform_stamped_message;
	transform_stamped_message.header.stamp = ros::Time::now();
	transform_stamped_message.header.frame_id = "global_edn";
	transform_stamped_message.child_frame_id = "body_NEU_absolute_yaw";

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
	static tf2_ros::TransformBroadcaster transform_broadcaster;
	geometry_msgs::TransformStamped transform_stamped_message;
	transform_stamped_message.header.stamp = ros::Time::now();
	transform_stamped_message.header.frame_id = "body_NEU_absolute_yaw";
	transform_stamped_message.child_frame_id = "body_NEU";
	
	tf2::Quaternion orientation, yaw;
	tf2::fromMsg(msg->orientation, orientation);
	orientation *= tf2::Quaternion(0, 0, 1, 0);
	geometry_msgs::Quaternion orientation_message = tf2::toMsg(orientation);
	
	transform_stamped_message.transform.rotation = orientation_message;
	transform_broadcaster.sendTransform(transform_stamped_message);
}

void camera_imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
	static tf2_ros::TransformBroadcaster transform_broadcaster;
	geometry_msgs::TransformStamped transform_stamped_message;
	transform_stamped_message.header.stamp = ros::Time::now();
	transform_stamped_message.header.frame_id = "body_NEU";
	transform_stamped_message.child_frame_id = "camera_imu_frame";
	transform_stamped_message.transform.rotation = msg->orientation;
	transform_broadcaster.sendTransform(transform_stamped_message);
}
*/

void control_effort_n_callback(const std_msgs::Float64::ConstPtr &msg)
{
	target_velocity.y = msg->data;
}

void control_effort_e_callback(const std_msgs::Float64::ConstPtr &msg)
{
	target_velocity.x = msg->data;
}

void control_effort_u_callback(const std_msgs::Float64::ConstPtr &msg)
{
	target_velocity.z = msg->data;
}

/*
# Message for SET_POSITION_TARGET_LOCAL_NED
#
# Some complex system requires all feautures that mavlink
# message provide. See issue #402.

std_msgs/Header header

uint8 coordinate_frame
uint8 FRAME_LOCAL_NED = 1
uint8 FRAME_LOCAL_OFFSET_NED = 7
uint8 FRAME_BODY_NED = 8
uint8 FRAME_BODY_OFFSET_NED = 9

uint16 type_mask
uint16 IGNORE_PX = 1 # Position ignore flags
uint16 IGNORE_PY = 2
uint16 IGNORE_PZ = 4
uint16 IGNORE_VX = 8 # Velocity vector ignore flags
uint16 IGNORE_VY = 16
uint16 IGNORE_VZ = 32
uint16 IGNORE_AFX = 64 # Acceleration/Force vector ignore flags
uint16 IGNORE_AFY = 128
uint16 IGNORE_AFZ = 256
uint16 FORCE = 512 # Force in af vector flag
uint16 IGNORE_YAW = 1024
uint16 IGNORE_YAW_RATE = 2048

geometry_msgs/Point position
geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 acceleration_or_force
float32 yaw
float32 yaw_rate
*/

/*
std_msgs/Header header
uint8 coordinate_frame
uint16 type_mask
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 acceleration_or_force
float32 yaw
float32 yaw_rate
*/

void landing_pad_whycon_pose_callback(const geometry_msgs::PoseStamped::ConstPtr msg)
{
	landing_pad_whycon_pose = *msg;
	geometry_msgs::PoseStamped buffer = landing_pad_whycon_pose;
	try
	{
		straighten_pose(landing_pad_whycon_pose);
//		buffer = straighten_pose(landing_pad_whycon_pose);
		landing_pad_whycon_pose = transform_buffer.transform(landing_pad_whycon_pose, "body_enu", ros::Duration(0.1));
	}
	catch( ... )
	{
		ROS_WARN("caught an exception.");
	}
	
//	tf2::Quaternion orientation;
//	tf2::fromMsg(buffer.pose.orientation, orientation);
//	tf2::fromMsg(msg->pose.orientation, orientation);

//	double yaw, pitch, roll;
//	tf2::Matrix3x3(orientation).getEulerYPR(yaw, pitch, roll);

//	ROS_INFO("YPR: <%0.2f, %0.2f, %0.2f", yaw, pitch, roll);
//	ROS_INFO_STREAM(buffer);
//	ROS_INFO_STREAM(landing_pad_whycon_pose);
}

void landing_pad_camera_pose_callback(const geometry_msgs::PoseStamped::ConstPtr msg)
{
	landing_pad_camera_pose = *msg;
}

void landing_pad_apriltag_pose_callback(const geometry_msgs::PoseStamped::ConstPtr msg)
{
	landing_pad_apriltag_pose = *msg;
}

/*
// target_position is in NWU
void approach(geometry_msgs::PoseStamped target_position)
{
	mavros_msgs::PositionTarget buffer;

	buffer.header.stamp = ros::Time::now();
	buffer.header.frame_id = "world";
	buffer.coordinate_frame = 8; // FRAME_BODY_NED
	buffer.type_mask = 4088; // ignore everything except positional arguments x, y

	buffer.position.x = -target_position.pose.position.y;
	buffer.position.y = target_position.pose.position.x;
	buffer.position.z = 0;

	setpoint_raw_local_publisher.publish(buffer);
}

void hold_plane_position()
{
	mavros_msgs::PositionTarget buffer;

	buffer.header.stamp = ros::Time::now();
	buffer.header.frame_id = "world";
	buffer.coordinate_frame = 8; // FRAME_BODY_NED
	buffer.type_mask = 4039; // ignore everything except velocity arguments x, y, z

	buffer.velocity.x = 0;
	buffer.velocity.y = 0;
	buffer.velocity.z = 0;

	setpoint_raw_local_publisher.publish(buffer);
}
*/

void set_velocity_target_neu( geometry_msgs::Vector3 _target_velocity )
{
	mavros_msgs::PositionTarget buffer;

	buffer.header.stamp = ros::Time::now();
	buffer.header.frame_id = "world";
	buffer.coordinate_frame = 8; // FRAME_BODY_NED
	buffer.type_mask = 4039; // ignore everything except velocity arguments x, y

	buffer.velocity.x = _target_velocity.x;
	buffer.velocity.y = _target_velocity.y;
	buffer.velocity.z = _target_velocity.z;

	setpoint_raw_local_publisher.publish(buffer);
}

void descend_in_place()
{
	mavros_msgs::PositionTarget buffer;

	buffer.header.stamp = ros::Time::now();
	buffer.header.frame_id = "world";
	buffer.coordinate_frame = 8; // FRAME_BODY_NED
	buffer.type_mask = 4039; // ignore everything except z velocity

	buffer.velocity.z = -0.5;

	setpoint_raw_local_publisher.publish(buffer);
}

double plane_distance_to(geometry_msgs::PoseStamped pose)
{
	return sqrt( pow(pose.pose.position.x, 2) + pow(pose.pose.position.y, 2) );
}

int main(int argc, char** argv)
{
	// init and say hi
	ros::init(argc, argv, "landing_controller");
	ROS_INFO("Started landing_controller!");

	// init
	std_msgs_true.data  = true;
	std_msgs_false.data = false;
	std_msgs_zero.data  = 0;

	// create node handle
	ros::NodeHandle node_handle;
	
	// create subscribers
//	ros::Subscriber camera_imu_subscriber = node_handle.subscribe("/camera/imu", 1000, camera_imu_callback);
//	ros::Subscriber imu_subscriber = node_handle.subscribe("/imu", 1000, imu_callback);
//	ros::Subscriber local_position_pose_subscriber = node_handle.subscribe("/mavros/local_position/pose", 1000, local_position_pose_callback);
	ros::Subscriber landing_pad_camera_pose_subscriber = node_handle.subscribe("/landing_pad/camera_pose", 1000, landing_pad_camera_pose_callback);
	ros::Subscriber landing_pad_whycon_pose_subscriber = node_handle.subscribe("/landing_pad/whycon_pose", 1000, landing_pad_whycon_pose_callback);
	ros::Subscriber landing_pad_apriltag_pose_subscriber = node_handle.subscribe("/landing_pad/apriltag_pose", 1000, landing_pad_apriltag_pose_callback);
	ros::Subscriber control_effort_n_subscriber = node_handle.subscribe("/pid/control_effort/n", 1000, control_effort_n_callback);
	ros::Subscriber control_effort_e_subscriber = node_handle.subscribe("/pid/control_effort/e", 1000, control_effort_e_callback);
	ros::Subscriber control_effort_u_subscriber = node_handle.subscribe("/pid/control_effort/u", 1000, control_effort_u_callback);

	// create publishers
	landing_pad_camera_pose_publisher = node_handle.advertise<geometry_msgs::PoseStamped>("/landing_pad/camera_pose", 1000);
	landing_pad_global_pose_stamped_publisher = node_handle.advertise<geometry_msgs::PoseStamped>("/landing_pad/global_pose_stamped", 1000);
	landing_pad_relative_pose_stamped_publisher = node_handle.advertise<geometry_msgs::PoseStamped>("/landing_pad/relative_pose_stamped", 1000);
	setpoint_raw_local_publisher = node_handle.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 1000);
	displacement_n_publisher = node_handle.advertise<std_msgs::Float64>("/landing_pad/body_neu/displacement/n", 1000);
	displacement_e_publisher = node_handle.advertise<std_msgs::Float64>("/landing_pad/body_neu/displacement/e", 1000);
	displacement_u_publisher = node_handle.advertise<std_msgs::Float64>("/landing_pad/body_neu/displacement/u", 1000);
	displacement_n_setpoint_publisher = node_handle.advertise<std_msgs::Float64>("/pid/body_neu/setpoint/n", 1000);
	displacement_e_setpoint_publisher = node_handle.advertise<std_msgs::Float64>("/pid/body_neu/setpoint/e", 1000);
	displacement_u_setpoint_publisher = node_handle.advertise<std_msgs::Float64>("/pid/body_neu/setpoint/u", 1000);
	pid_enable_n_publisher = node_handle.advertise<std_msgs::Bool>("/pid/pid_enable/n/", 1000);
	pid_enable_e_publisher = node_handle.advertise<std_msgs::Bool>("/pid/pid_enable/e/", 1000);
	pid_enable_u_publisher = node_handle.advertise<std_msgs::Bool>("/pid/pid_enable/u/", 1000);

	// for transforms
	static tf2_ros::TransformListener transform_listener(transform_buffer);

	ros::Rate loop_rate(40);
	while( ros::ok() )
	{
		// callbacks
		ros::spinOnce();
	
		//if( ros::Time::now() - last_detection_time < abort_time )
		if( ros::Time::now() - landing_pad_camera_pose.header.stamp < abort_time )
		{
			// transform pose in camera frame to pose relative to the drone in NED
			try
			{
				// this is done because it generates a necessary transform
				straighten_pose(landing_pad_apriltag_pose);

				// determine how to react to the landing pad detection
				landing_pad_relative_pose_stamped 		= transform_buffer.transform(landing_pad_camera_pose, "body_enu", ros::Duration(0.1));
//				landing_pad_relative_pose_stamped_straightened	= straighten_pose(landing_pad_relative_pose_stamped);

//				ROS_INFO_STREAM(landing_pad_relative_pose_stamped_straightened);

				// publish relative pose
				landing_pad_relative_pose_stamped_publisher.publish(landing_pad_relative_pose_stamped);
				landing_pad_global_pose_stamped_publisher.publish(landing_pad_global_pose_stamped);
			}
			catch( tf2::TransformException &exception )
			{
				ROS_WARN("main loop: %s", exception.what());
			}

			// Direct the drone towards the landing pad
			if( ros::Time::now() - last_target_send_time >= ros::Duration(0.1) )
			{
				last_target_send_time = ros::Time::now();
			        
				double distance = plane_distance_to(landing_pad_relative_pose_stamped_straightened);
				// publish PID states
				displacement_n_publisher.publish( landing_pad_relative_pose_stamped.pose.position.y );
				displacement_e_publisher.publish( landing_pad_relative_pose_stamped.pose.position.x );
				displacement_u_publisher.publish( landing_pad_relative_pose_stamped.pose.position.z );

				// publish PID setpoints (always 0)
				displacement_n_setpoint_publisher.publish( std_msgs_zero );
				displacement_e_setpoint_publisher.publish( std_msgs_zero );
				displacement_u_setpoint_publisher.publish( std_msgs_zero );
			
				if( distance >= 0.5  )
				{
					// do not descend while not in landing range
					pid_enable_u_publisher.publish( std_msgs_false );
					target_velocity.z = 0;
				}
				else
				{
					// descend if within landing range
					pid_enable_u_publisher.publish( std_msgs_true );

				}
				// approach using velocity
//				set_velocity_target_neu( target_velocity );
			}
		}

		loop_rate.sleep();
	}

	return 0;
}
