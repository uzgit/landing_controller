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

ros::Publisher landing_pad_camera_odom_publisher;
ros::Publisher landing_pad_camera_pose_publisher;
ros::Publisher landing_pad_base_link_pose_stamped_publisher;
ros::Publisher landing_pad_relative_pose_publisher;
ros::Publisher landing_pad_relative_pose_stamped_publisher;
ros::Publisher landing_pad_global_pose_publisher;
ros::Publisher landing_pad_global_pose_stamped_publisher;
ros::Publisher landing_pad_global_pose_filtered_publisher;
ros::Publisher setpoint_raw_local_publisher;
ros::Publisher landing_pad_position_publisher;
ros::Publisher imu_data_forwarding_publisher;
ros::Publisher camera_imu_data_forwarding_publisher;
ros::Publisher visual_odometry_publisher;

geometry_msgs::PoseStamped landing_pad_camera_pose;
geometry_msgs::PoseStamped landing_pad_base_link_pose_stamped;
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

bool stopped = false;

// bit IDs of landing pad 
//int landing_pad_id[2] = {1, 2};
int landing_pad_id[2] = {1, 9999999};

// timing
ros::Time last_detection_time(0);
ros::Duration abort_time(0.5);
ros::Time last_target_send_time(0);
ros::Duration target_send_interval(0.5);

ros::Publisher landing_pad_estimate_publisher;

tf2::Quaternion camera_imu_orientation;

geometry_msgs::PoseStamped straighten_pose( const geometry_msgs::PoseStamped & _pose_in )
{
	// declare pose variables
	geometry_msgs::PoseStamped pose_out;
	geometry_msgs::PoseStamped pose_in = _pose_in;

	// make the child_frame_id equal to the parent's frame_id with '_straightened' appended
	std::string child_frame_id = pose_in.header.frame_id + "_straightened";

	// access the transform broadcaster
	static tf2_ros::TransformBroadcaster transform_broadcaster;

	// declare a message in which to store the new transform
	geometry_msgs::TransformStamped transform_stamped_message;

	// set the transform's header
	transform_stamped_message.header.stamp = ros::Time::now();
	transform_stamped_message.header.frame_id = pose_in.header.frame_id;
	transform_stamped_message.child_frame_id = child_frame_id;

	// set the rotation from the parent to the child
	
	// for normalizing rotation just in case (doesn't seem to change anything)
	tf2::Quaternion rotation;
	tf2::fromMsg(pose_in.pose.orientation, rotation);
	tf2::Quaternion offset(0.027, 0.025, 0.027, 0.999);
	rotation *= offset;
	rotation = rotation.normalize();
	pose_in.pose.orientation = tf2::toMsg(rotation);
	
	transform_stamped_message.transform.rotation = pose_in.pose.orientation;

	// send the transform
	transform_broadcaster.sendTransform(transform_stamped_message);

	// transform the pose and return it
	return transform_buffer.transform(pose_in, child_frame_id, ros::Duration(0.05));
}

void local_position_pose_callback(const geometry_msgs::PoseStamped::ConstPtr msg)
{
	// this message is in END (east, north, down)
	local_position_pose_stamped = *msg;

	static tf2_ros::TransformBroadcaster transform_broadcaster;
	geometry_msgs::TransformStamped transform_stamped_message;
	transform_stamped_message.header.stamp = ros::Time::now();
	transform_stamped_message.header.frame_id = "global_edn";
	transform_stamped_message.child_frame_id = "body_edn_absolute_yaw";

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
	transform_stamped_message.header.frame_id = "body_edn_absolute_yaw";
	transform_stamped_message.child_frame_id = "body_edn";

	tf2::Quaternion orientation, yaw;
	tf2::fromMsg(msg->orientation, orientation);

	double y, p, r;
	tf2::Matrix3x3(orientation).getEulerYPR(y, p, r);

	yaw.setRPY(r, p, y);
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
	transform_stamped_message.header.frame_id = "body_edn";
	transform_stamped_message.child_frame_id = "camera_imu_frame";
	transform_stamped_message.transform.rotation = msg->orientation;
	transform_broadcaster.sendTransform(transform_stamped_message);
}

void visual_callback(const visualization_msgs::MarkerArray::ConstPtr& msg)
{
	bool detection = false;
	int num_markers = msg->markers.size();

	ROS_INFO_STREAM(num_markers);

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

	//if( detection )
	try
	{
		detection = true;
		buffer = msg->markers[0].pose;
	}
	catch( ... )
	{
		ROS_WARN("no marker");
	}
	if( true )
	{
		landing_pad_camera_pose.header.stamp = ros::Time::now();
		landing_pad_camera_pose.header.frame_id = "camera_frame";
		landing_pad_camera_pose.pose = buffer;
		
		last_detection_time = ros::Time::now();

		landing_pad_camera_pose_publisher.publish(landing_pad_camera_pose);
	}
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

// target_position is in EDN
void approach(geometry_msgs::PoseStamped target_position)
{
	mavros_msgs::PositionTarget buffer;

	buffer.header.stamp = ros::Time::now();
	buffer.header.frame_id = "world";
	buffer.coordinate_frame = 8; // FRAME_BODY_NED
	buffer.type_mask = 4088; // ignore everything except positional arguments x, y

	buffer.position.x = target_position.pose.position.z;
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
	buffer.type_mask = 4039; // ignore everything except positional arguments x, y

	buffer.velocity.x = 0;
	buffer.velocity.y = 0;
	buffer.velocity.z = 0;

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

	// create node handle
	ros::NodeHandle node_handle;
	
	// create subscribers
        ros::Subscriber visual_subscriber = node_handle.subscribe("/whycon_ros/visual", 1000, visual_callback);
	ros::Subscriber camera_imu_subscriber = node_handle.subscribe("/iris/camera/imu", 1000, camera_imu_callback);
	ros::Subscriber imu_subscriber = node_handle.subscribe("/iris/imu", 1000, imu_callback);
	ros::Subscriber local_position_pose_subscriber = node_handle.subscribe("/mavros/local_position/pose", 1000, local_position_pose_callback);
	
	// create publishers
	landing_pad_camera_pose_publisher = node_handle.advertise<geometry_msgs::PoseStamped>("/landing_pad/camera_pose", 1000);
	landing_pad_global_pose_publisher = node_handle.advertise<geometry_msgs::Pose>("/landing_pad/global_pose", 1000);
	landing_pad_global_pose_stamped_publisher = node_handle.advertise<geometry_msgs::PoseStamped>("/landing_pad/global_pose_stamped", 1000);
	landing_pad_base_link_pose_stamped_publisher = node_handle.advertise<geometry_msgs::PoseStamped>("/landing_pad/base_link_pose_stamped", 1000);
	landing_pad_relative_pose_stamped_publisher = node_handle.advertise<geometry_msgs::PoseStamped>("/landing_pad/relative_pose_stamped", 1000);
	landing_pad_global_pose_filtered_publisher = node_handle.advertise<geometry_msgs::Pose>("/landing_pad/global_pose_filtered", 1000);
	setpoint_raw_local_publisher = node_handle.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 1000);

	// for filtering
	landing_pad_camera_odom_publisher = node_handle.advertise<nav_msgs::Odometry>("/odometry/camera/landing_pad_pose", 1000);
	imu_data_forwarding_publisher = node_handle.advertise<sensor_msgs::Imu>("/imu_data", 1000);
	visual_odometry_publisher = node_handle.advertise<nav_msgs::Odometry>("/visual_odometry", 1000);

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
				landing_pad_relative_pose_stamped 		= transform_buffer.transform(landing_pad_camera_pose, "body_edn",		ros::Duration(0.1));
				landing_pad_relative_pose_stamped		= straighten_pose(landing_pad_relative_pose_stamped);
//				landing_pad_base_link_pose_stamped		= transform_buffer.transform(landing_pad_camera_pose, "body_edn", 		ros::Duration(0.1));
				
				landing_pad_relative_pose_absolute_yaw_stamped	= transform_buffer.transform(landing_pad_camera_pose, "body_edn_absolute_yaw",	ros::Duration(0.05));
				landing_pad_relative_pose_absolute_yaw_stamped	= straighten_pose(landing_pad_relative_pose_absolute_yaw_stamped);

				landing_pad_global_pose_stamped.pose.position.x = local_position_pose_stamped.pose.position.y + landing_pad_relative_pose_absolute_yaw_stamped.pose.position.z;
				landing_pad_global_pose_stamped.pose.position.y = -local_position_pose_stamped.pose.position.x - landing_pad_relative_pose_absolute_yaw_stamped.pose.position.x;
				landing_pad_global_pose_stamped.pose.position.z = max(local_position_pose_stamped.pose.position.z - landing_pad_relative_pose_absolute_yaw_stamped.pose.position.y, 0.1);
			
				geometry_msgs::PoseStamped buffer		= transform_buffer.transform(landing_pad_camera_pose, "body_edn_absolute_yaw", 		ros::Duration(0.1));
				ROS_INFO_STREAM(straighten_pose(buffer));
				//ROS_INFO_STREAM(buffer);
			}
			catch( tf2::TransformException &exception )
			{
				ROS_WARN("main loop: %s", exception.what());
			}

			// publish relative pose
			landing_pad_relative_pose_stamped_publisher.publish(landing_pad_relative_pose_stamped);
			landing_pad_global_pose_stamped_publisher.publish(landing_pad_global_pose_stamped);

			// Direct the drone towards the landing pad
			if( ros::Time::now() - last_target_send_time >= ros::Duration(0.5) )
			{
				last_target_send_time = ros::Time::now();
				double distance = plane_distance_to(landing_pad_relative_pose_stamped);
				//ROS_INFO_STREAM(landing_pad_relative_pose_stamped);
//				ROS_INFO("%f", distance); 
//				if( distance >= 0.3 )
//				{
//					approach(landing_pad_relative_pose_stamped);
//				}
//				else// if( ! stopped )
//				{
//					descend_in_place();
					//hold_plane_position();
				//	stopped = true;
//					if( distance < 0.3 )
//					{
//						descend();
//					}
//				}
			}
		}

		loop_rate.sleep();
	}

	return 0;
}
