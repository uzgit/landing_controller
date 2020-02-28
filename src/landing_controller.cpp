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
	local_position_pose_stamped = *msg;
}

void imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
	imu_buffer = *msg;
	tf2::convert(msg->orientation, imu_buffer.orientation);
	imu_buffer.header.frame_id = "imu_link";
	imu_data_forwarding_publisher.publish(imu_buffer);

	//		    x, y, z, w
	tf2::Quaternion NED(0, 0.707, 0, 0.707);
/*
	static tf2_ros::TransformBroadcaster transform_broadcaster;
	geometry_msgs::TransformStamped transform_stamped_message;
	transform_stamped_message.header.stamp = ros::Time::now();
	transform_stamped_message.header.frame_id = "base_link";
	transform_stamped_message.child_frame_id = "base_link_NED";
	transform_stamped_message.transform.rotation = NED * (msg->orientation).inverse();
	transform_broadcaster.sendTransform(transform_stamped_message);
*/
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
//		landing_pad_camera_pose.header.frame_id = "camera_imu_frame";
		landing_pad_camera_pose.header.frame_id = "camera_frame";
		landing_pad_camera_pose.pose = buffer;
		landing_pad_camera_pose.pose.position.x = buffer.position.z;
		landing_pad_camera_pose.pose.position.y = buffer.position.x;
		landing_pad_camera_pose.pose.position.z = -buffer.position.y;
		last_detection_time = ros::Time::now();
	}

	// determine relative position of landing pad even with no detection (in case detection is momentarily lost)
	// main loop will determine whether or not to publish based on timing
	// transform pose in camera frame to pose relative to the drone in NED
	try
	{
		// works when transforming from origin=camera_imu_frame
		landing_pad_relative_pose_stamped = transform_buffer.transform(landing_pad_camera_pose, "base_link", ros::Duration(0.2));

/*
		// get the current rotation of the camera
		geometry_msgs::TransformStamped camera_transform_stamped_message = transform_buffer.lookupTransform("base_link", "camera_frame", ros::Time(0));
		tf2::Quaternion camera_rotation;
		tf2::fromMsg(camera_transform_stamped_message.transform.rotation, camera_rotation);

		// use transform broadcaster
		static tf2_ros::TransformBroadcaster transform_broadcaster;
		
		// generate a transform from base_link to NED
		geometry_msgs::TransformStamped transform_stamped_message;

		// set header
		transform_stamped_message.header.stamp = ros::Time::now();
		transform_stamped_message.header.frame_id = "base_link";
		transform_stamped_message.child_frame_id = "camera_NED";
		
		// target orientation
		tf2::Quaternion NED(0, 0.707, 0, 0.707);
		
		// get required additional rotation
		// subtract camera_rotation from NED_rotation
		tf2::Quaternion NED_correction = NED * camera_rotation.inverse();

		double y,p,r;
		tf2::Matrix3x3(camera_rotation).getEulerYPR(y, p, r);
		ROS_INFO("gimbal < %0.2f, %0.2f, %0.2f >", y, p, r);
		tf2::Matrix3x3(camera_imu_orientation).getEulerYPR(y, p, r);
		ROS_INFO("imu    < %0.2f, %0.2f, %0.2f >", y, p, r);

		// set rotation to the rotation to get from camera to NED
		transform_stamped_message.transform.rotation = tf2::toMsg(NED);
		transform_broadcaster.sendTransform(transform_stamped_message);


		tf2::Quaternion undo_rotation;
		tf2::fromMsg(landing_pad_relative_pose_stamped.pose.orientation, undo_rotation);
		tf2::Transform undo_rotation_transform;
		undo_rotation_transform.setRotation( undo_rotation.inverse() );

		geometry_msgs::TransformStamped undo_rotation_transform_stamped;
		undo_rotation_transform_stamped.transform = tf2::toMsg(undo_rotation_transform);

		tf2::doTransform(landing_pad_relative_pose_stamped, landing_pad_relative_pose_stamped, undo_rotation_transform_stamped);
*/

/*
		tf2::Transform camera_to_world( camera_imu_orientation );
		tf2::Transform gimbal_to_drone = camera_to_world;
		geometry_msgs::TransformStamped gimbal_to_drone_stamped;
	       	gimbal_to_drone_stamped.transform = tf2::toMsg(gimbal_to_drone);
		tf2::doTransform(landing_pad_camera_pose, landing_pad_relative_pose_stamped, gimbal_to_drone_stamped);
*/
	}
	catch( tf2::TransformException &exception )
	{
		ROS_WARN("%s", exception.what());
	}
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
	landing_target_raw_publisher = node_handle.advertise<mavros_msgs::LandingTarget>("/mavros/landing_target/raw", 1000);
	
//	landing_pad_camera_pose_publisher = node_handle.advertise<geometry_msgs::Pose>("/landing_pad/camera_pose", 1000);
	landing_pad_relative_pose_publisher = node_handle.advertise<geometry_msgs::Pose>("/landing_pad/relative_pose", 1000);
	landing_pad_global_pose_publisher = node_handle.advertise<geometry_msgs::Pose>("/landing_pad/global_pose", 1000);
	landing_pad_relative_pose_stamped_publisher = node_handle.advertise<geometry_msgs::PoseStamped>("/landing_pad/relative_pose_stamped", 1000);
	landing_pad_global_pose_filtered_publisher = node_handle.advertise<geometry_msgs::Pose>("/landing_pad/global_pose_filtered", 1000);

	ros::Subscriber local_position_pose_subscriber = node_handle.subscribe("/mavros/local_position/pose", 1000, local_position_pose_callback);
	setpoint_position_local_publisher = node_handle.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 1000);

	// for filtering
	landing_pad_camera_odom_publisher = node_handle.advertise<nav_msgs::Odometry>("/odometry/camera/landing_pad_pose", 1000);
	imu_data_forwarding_publisher = node_handle.advertise<sensor_msgs::Imu>("/imu_data", 1000);
	visual_odometry_publisher = node_handle.advertise<nav_msgs::Odometry>("/visual_odometry", 1000);
	ros::Subscriber odometry_filtered_subscriber = node_handle.subscribe("/odometry/filtered", 1000, odometry_filtered_callback);

	// for transforms
	tf2_ros::TransformListener transform_listener(transform_buffer);

/**************************************************************************************************************/
	ros::Rate loop_rate(20);
	while( ros::ok() )
	{
		// callbacks
		ros::spinOnce();
	
		if( ros::Time::now() - last_detection_time < abort_time )
		{
			// transform whycon pose to relative pose

			// publish relative pose
			landing_pad_relative_pose_publisher.publish(landing_pad_relative_pose);
//			std::cout << "(" << landing_pad_relative_pose.orientation.w << ", " << landing_pad_relative_pose.orientation.x << ", " << landing_pad_relative_pose.orientation.y << ", " << landing_pad_relative_pose.orientation.z << ")" << std::endl;

			// Direct the drone towards the landing pad
//			send_target_position_local_pose_stamped();

			// for visual update: mavros uses ENU, gazebo uses NWD
			landing_pad_global_pose.position.x = local_position_pose_stamped.pose.position.y + landing_pad_relative_pose.position.x;
			landing_pad_global_pose.position.y = -local_position_pose_stamped.pose.position.x + landing_pad_relative_pose.position.y;
			landing_pad_global_pose.position.z = max(local_position_pose_stamped.pose.position.z + landing_pad_relative_pose.position.z, 0.05);
//			landing_pad_global_pose.orientation = landing_pad_relative_pose.orientation;

			landing_pad_relative_pose_stamped_publisher.publish(landing_pad_relative_pose_stamped);
			landing_pad_global_pose_publisher.publish(landing_pad_global_pose);
		}

		loop_rate.sleep();
	}
/**************************************************************************************************************/

	return 0;
}
