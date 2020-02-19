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

// for Gazebo simulation
#include <gazebo_msgs/LinkStates.h>
#include <gazebo_msgs/LinkState.h>

using namespace std;

ros::Publisher landing_target_raw_publisher;
mavros_msgs::LandingTarget landing_target_message;

ros::Publisher landing_pad_relative_pose_publisher;
ros::Publisher landing_pad_global_pose_publisher;
ros::Publisher setpoint_position_local_publisher;
ros::Publisher landing_pad_position_publisher;

geometry_msgs::Pose landing_pad_camera_pose;
geometry_msgs::Pose landing_pad_relative_pose;
geometry_msgs::Pose landing_pad_global_pose;
geometry_msgs::PoseStamped landing_pad_relative_pose_stamped;
geometry_msgs::PoseStamped local_position_pose_stamped;

// bit IDs of landing pad 
int landing_pad_id[2] = {1, 2};

// timing
ros::Time last_detection_time(0);
ros::Duration abort_time(0.5);

tf2::Transform ground_plane_link_transform;
tf2::Quaternion imu_orientation;
tf2::Transform imu_transform;
tf2::Quaternion camera_imu_orientation;
tf2::Transform camera_imu_transform;

geometry_msgs::Pose iris_pose;

gazebo_msgs::LinkState link_state_message;
ros::Publisher landing_pad_estimate_publisher;

void local_position_pose_callback(const geometry_msgs::PoseStamped::ConstPtr msg)
{
	local_position_pose_stamped = *msg;
}

void imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
	tf2::convert(msg->orientation, imu_orientation);
	imu_transform.setRotation(imu_orientation);
}

void camera_imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
	tf2::convert(msg->orientation, camera_imu_orientation);
	camera_imu_transform.setRotation(camera_imu_orientation);
}

int seq = 0;
void send_target_position_local_pose_stamped()
{
	seq ++;

	landing_pad_relative_pose_stamped.header.seq   = seq;
	landing_pad_relative_pose_stamped.header.stamp = ros::Time::now();
//	landing_pad_relative_pose_stamped.header.frame_id = "relative_pose";

	landing_pad_relative_pose_stamped.pose.orientation = landing_pad_relative_pose.orientation;
	
	// relative pose is in NWD, but we need ENU
	landing_pad_relative_pose_stamped.pose.position.x = - landing_pad_relative_pose.position.y;
	landing_pad_relative_pose_stamped.pose.position.y = landing_pad_relative_pose.position.x;

	landing_pad_relative_pose_stamped.pose.position.x += local_position_pose_stamped.pose.position.x;
	landing_pad_relative_pose_stamped.pose.position.y += local_position_pose_stamped.pose.position.y;
	landing_pad_relative_pose_stamped.pose.position.z += local_position_pose_stamped.pose.position.z;

	landing_pad_relative_pose_stamped.pose.position.z = 10;
	
	setpoint_position_local_publisher.publish(landing_pad_relative_pose_stamped);
}

void send_landing_target_message()
{
	landing_target_message.header.stamp.sec  = (int)ros::Time::now().sec;
	landing_target_message.header.stamp.nsec = (int)ros::Time::now().nsec;
	landing_target_message.header.frame_id = 149;
	landing_target_message.target_num = 1;
	landing_target_message.frame = mavros_msgs::LandingTarget::LOCAL_NED;
	landing_target_message.type = mavros_msgs::LandingTarget::VISION_FIDUCIAL;
	landing_target_message.angle[0] = atan( landing_pad_relative_pose.position.x / landing_pad_relative_pose.position.z );
	landing_target_message.angle[1] = atan( landing_pad_relative_pose.position.y / landing_pad_relative_pose.position.z );
	landing_target_message.size[0] = 0.1;
	landing_target_message.size[1] = 0.1;

	landing_target_raw_publisher.publish(landing_target_message);
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
		landing_pad_camera_pose = buffer;
		landing_pad_camera_pose.position.x = buffer.position.z;
		landing_pad_camera_pose.position.y = buffer.position.x;
		landing_pad_camera_pose.position.z = -buffer.position.y;
		last_detection_time = ros::Time::now();
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
	
	landing_pad_relative_pose_publisher = node_handle.advertise<geometry_msgs::Pose>("/landing_pad/relative_pose", 1000);
	landing_pad_global_pose_publisher = node_handle.advertise<geometry_msgs::Pose>("/landing_pad/global_pose", 1000);

	ros::Subscriber local_position_pose_subscriber = node_handle.subscribe("/mavros/local_position/pose", 1000, local_position_pose_callback);
	setpoint_position_local_publisher = node_handle.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 1000);

	// /landing_pad/global_position is for the landing_pad_estimate_plugin
//	landing_pad_position_publisher = node_handle.advertise<geometry_msgs::Vector3>("/landing_pad/global_position", 1000);
//	landing_pad_position_publisher = node_handle.advertise<geometry_msgs::Vector3>("/landing_pad/global_pose", 1000);

	ros::Rate loop_rate(20);
	while( ros::ok() )
	{
		// callbacks
		ros::spinOnce();
	
		tf2::Quaternion NED_quaternion;
		NED_quaternion.setRPY(0, -3.1415926/2, 0);
		tf2::Transform NED;
		NED.setRotation(NED_quaternion);
		double cam_pitch, cam_roll, cam_yaw;
		tf2::Matrix3x3(camera_imu_orientation).getEulerYPR(cam_yaw, cam_pitch, cam_roll);
		double imu_pitch, imu_roll, imu_yaw;
		tf2::Matrix3x3(imu_orientation).getEulerYPR(imu_yaw, imu_pitch, imu_roll);

/*
		ROS_INFO("iris imu YPR: <%0.3f, %0.3f, %0.3f>", imu_yaw, imu_pitch, imu_roll);
		ROS_INFO(" cam imu YPR: <%0.3f, %0.3f, %0.3f>", cam_yaw, cam_pitch, cam_roll);
*/

		// Construct relevant transforms
		tf2::Transform camera_to_world( camera_imu_orientation);//, tf2::Vector3(0, 0, 0.3) );
		tf2::Transform world_to_drone( imu_orientation.inverse() );
		tf2::Transform gimbal_to_drone = camera_to_world;// * world_to_drone;

		// necessary overhead for tf2::doTransform
		geometry_msgs::TransformStamped gimbal_to_drone_stamped;
	       	gimbal_to_drone_stamped.transform = tf2::toMsg(gimbal_to_drone);

		// transform visual marker detection location from camera to drone coordinate frame
		tf2::doTransform(landing_pad_camera_pose, landing_pad_relative_pose, gimbal_to_drone_stamped);

		if( ros::Time::now() - last_detection_time < abort_time )
		{
			landing_pad_relative_pose_publisher.publish(landing_pad_relative_pose);
//			std::cout << "(" << landing_pad_relative_pose.orientation.w << ", " << landing_pad_relative_pose.orientation.x << ", " << landing_pad_relative_pose.orientation.y << ", " << landing_pad_relative_pose.orientation.z << ")" << std::endl;

			send_target_position_local_pose_stamped();

			// for visual update: mavros uses ENU, gazebo uses NWD
			landing_pad_global_pose.position.x = local_position_pose_stamped.pose.position.y + landing_pad_relative_pose.position.x;
			landing_pad_global_pose.position.y = -local_position_pose_stamped.pose.position.x + landing_pad_relative_pose.position.y;
			landing_pad_global_pose.position.z = max(local_position_pose_stamped.pose.position.z + landing_pad_relative_pose.position.z, 0.05);
		//	landing_pad_global_pose.orientation = landing_pad_relative_pose.orientation;

			//tf2::Quaternion landing_pad_orientation = tf2::Quaternion(landing_pad_relative_pose.orientation.w, landing_pad_relative_pose.orientation.x, landing_pad_relative_pose.orientation.y, landing_pad_relative_pose.orientation.z);

			tf2::Quaternion landing_pad_orientation = tf2::Quaternion(1, 0, 0, 0);


			// pose estimation does not quite work yet. there are still issues to deal with in the simulation itself (camera flips upside-down sometimes)
/*
			tf2::Quaternion final_rotation;
							//YPR
			final_rotation.setEuler(3.1415925/2.0, 0, imu_yaw - cam_yaw);
			landing_pad_orientation = final_rotation * landing_pad_orientation;
			landing_pad_relative_pose.orientation.w = landing_pad_orientation.w();
			landing_pad_relative_pose.orientation.x = landing_pad_orientation.y();
			landing_pad_relative_pose.orientation.y = landing_pad_orientation.z();
			landing_pad_relative_pose.orientation.z = landing_pad_orientation.x();
			landing_pad_global_pose.orientation = landing_pad_relative_pose.orientation;
*/

/*
			landing_pad_global_pose.orientation.w = landing_pad_relative_pose.orientation.w;
			landing_pad_global_pose.orientation.x = landing_pad_relative_pose.orientation.x;
			landing_pad_global_pose.orientation.y = landing_pad_relative_pose.orientation.y;
			landing_pad_global_pose.orientation.z = landing_pad_relative_pose.orientation.z;
*/
			landing_pad_global_pose_publisher.publish(landing_pad_global_pose);

/*
			// for visual update
			geometry_msgs::Vector3 position;
			position.x = iris_pose.position.x + landing_pad_relative_pose.position.x;
			position.y = iris_pose.position.y + landing_pad_relative_pose.position.y;
			position.z = max(iris_pose.position.z + landing_pad_relative_pose.position.z, 0.1);
			landing_pad_position_publisher.publish(position);
*/

			// don't use this because the landing_target protocol leads to unpredictable behavior
//			send_landing_target_message();
		}

		loop_rate.sleep();
	}

	return 0;
}
