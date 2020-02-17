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
ros::Publisher landing_pad_position_publisher;
ros::Publisher true_landing_pad_relative_pose_publisher;

geometry_msgs::Pose landing_pad_camera_pose;
geometry_msgs::Pose landing_pad_relative_pose;

// bit IDs of landing pad 
int landing_pad_id[2] = {1, 2};

// timing
ros::Time last_detection_time(0);
ros::Duration abort_time(0.5);

// for Gazebo simulation
tf2::Transform iris_base_link_transform;
tf2::Transform gimbal_base_link_transform;
tf2::Transform gimbal_tilt_link_transform;
const std::string ground_plane_link_name = "ground_plane::link";
const std::string iris_base_link_name    = "iris_demo::iris_demo::iris::base_link";
const std::string gimbal_base_link_name  = "iris_demo::iris_demo::gimbal_small_2d::base_link";
const std::string gimbal_tilt_link_name  = "iris_demo::iris_demo::gimbal_small_2d::tilt_link";

tf2::Transform ground_plane_link_transform;
tf2::Quaternion imu_orientation;
tf2::Transform imu_transform;
tf2::Quaternion camera_imu_orientation;
tf2::Transform camera_imu_transform;

geometry_msgs::Pose iris_pose;

gazebo_msgs::LinkState link_state_message;
ros::Publisher landing_pad_estimate_publisher;

void imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
	tf2::convert(msg->orientation, imu_orientation);
	imu_transform.setRotation(imu_orientation);
}

void camera_imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
	tf2::convert(msg->orientation, camera_imu_orientation);
	//camera_imu_orientation = camera_imu_orientation.normalize();

	//double yaw, pitch, roll;
	//tf2::Matrix3x3(camera_imu_orientation).getEulerYPR(yaw, pitch, roll);
	//tf2::Quaternion yaw_quaternion;
	//yaw_quaternion.setRPY(0, 0, yaw);
	//yaw_quaternion = yaw_quaternion.normalize();

	//camera_imu_orientation = (camera_imu_orientation * yaw_quaternion.inverse()).normalize();

	camera_imu_transform.setRotation(camera_imu_orientation);
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

void link_states_callback(const gazebo_msgs::LinkStates::ConstPtr& msg)
{
	for(int i = 0; i < msg->name.size(); i ++)
	{
		if( msg->name[i] == ground_plane_link_name )
		{
			tf2::fromMsg(msg->pose[i], ground_plane_link_transform);
		}
		else if( msg->name[i] == iris_base_link_name )
		{
			tf2::fromMsg(msg->pose[i], iris_base_link_transform);

			iris_pose = msg->pose[i];
		}
		else if( msg->name[i] == gimbal_base_link_name )
		{
			tf2::fromMsg(msg->pose[i], gimbal_base_link_transform);
		}
		else if( msg->name[i] == gimbal_tilt_link_name )
		{
			tf2::fromMsg(msg->pose[i], gimbal_tilt_link_transform);
		}
	}
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
		landing_pad_camera_pose.position.z = buffer.position.y;
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
	ros::Subscriber link_state_subscriber = node_handle.subscribe("/gazebo/link_states", 1000, link_states_callback);
	landing_target_raw_publisher = node_handle.advertise<mavros_msgs::LandingTarget>("/mavros/landing_target/raw", 1000);
	
	landing_pad_relative_pose_publisher = node_handle.advertise<geometry_msgs::Pose>("/landing_pad/relative_pose", 1000);

	// /landing_pad/global_position is for the landing_pad_estimate_plugin
//	landing_pad_position_publisher = node_handle.advertise<geometry_msgs::Vector3>("/landing_pad/global_position", 1000);
	landing_pad_position_publisher = node_handle.advertise<geometry_msgs::Vector3>("/landing_pad/global_pose", 1000);
//	landing_pad_estimate_publisher = node_handle.advertise<gazebo_msgs::LinkState>("/gazebo/set_link_state", 1000);

	ros::Rate loop_rate(20);
	while( ros::ok() )
	{
		// callbacks
		ros::spinOnce();
	
		tf2::Quaternion NED_quaternion;
		NED_quaternion.setRPY(0, -3.1415926/2, 0);
		tf2::Transform NED;
		NED.setRotation(NED_quaternion);

		double imu_pitch, imu_roll, imu_yaw;
		double cam_pitch, cam_roll, cam_yaw;
		tf2::Matrix3x3(imu_orientation).getEulerYPR(imu_yaw, imu_pitch, imu_roll);
		tf2::Matrix3x3(camera_imu_orientation).getEulerYPR(cam_yaw, cam_pitch, cam_roll);

		ROS_INFO("iris imu YPR: <%0.3f, %0.3f, %0.3f>", imu_yaw, imu_pitch, imu_roll);
		ROS_INFO(" cam imu YPR: <%0.3f, %0.3f, %0.3f>", cam_yaw, cam_pitch, cam_roll);

//		tf2::Quaternion target_orientation = camera_imu_orientation;

/*
		tf2::Quaternion target_orientation;
		target_orientation.setEuler(imu_yaw, 0, 0);
		//target_orientation.setEuler(0, 0, 0);
		target_orientation = target_orientation.normalize();
		target_orientation *= camera_imu_orientation;
*/

/*
		tf2::Quaternion yaw;
		yaw.setEuler(0, imu_yaw, 0);
		yaw = yaw.normalize();
*/
//		tf2::Quaternion target_orientation = camera_imu_orientation * imu_orientation;

		// I think this one works.
		tf2::Quaternion target_orientation = imu_orientation.inverse() * camera_imu_orientation;

//		tf2::Quaternion target_orientation = camera_imu_orientation * (imu_orientation * camera_imu_orientation.inverse());
		
		double target_yaw, target_pitch, target_roll;
		tf2::Matrix3x3(target_orientation).getEulerYPR(target_yaw, target_pitch, target_roll);
		ROS_INFO("target_orientation YPR: <%0.3f, %0.3f, %0.3f>", target_yaw, target_pitch, target_roll);

//		tf2::Quaternion target_orientation = camera_imu_orientation * imu_orientation;

		// camera then yaw seems not to work at all
//		tf2::Quaternion target_orientation = camera_imu_orientation * yaw;
//		tf2::Quaternion target_orientation = yaw * camera_imu_orientation;

		// create transform from camera to drone coordinate frame
//		tf2::Transform gimbal_to_drone = camera_imu_transform;
		tf2::Transform gimbal_to_drone;
		gimbal_to_drone.setRotation( target_orientation );
//		tf2::Transform gimbal_to_drone = camera_imu_transform.inverse();// * NED;
//		tf2::Transform gimbal_to_drone = camera_imu_transform * imu_transform * ground_plane_link_transform;

		// necessary overhead for tf2::doTransform
		geometry_msgs::TransformStamped gimbal_to_drone_stamped;
	       	gimbal_to_drone_stamped.transform = tf2::toMsg(gimbal_to_drone);

		// transform visual marker detection location from camera to drone coordinate frame
		tf2::doTransform(landing_pad_camera_pose, landing_pad_relative_pose, gimbal_to_drone_stamped);

		if( ros::Time::now() - last_detection_time < abort_time )
		{
			landing_pad_relative_pose_publisher.publish(landing_pad_relative_pose);

			// publish topic for marker estimate update
			geometry_msgs::Vector3 position;
			position.x = iris_pose.position.x + landing_pad_relative_pose.position.x;
			position.y = iris_pose.position.y + landing_pad_relative_pose.position.y;
			position.z = max(iris_pose.position.z + landing_pad_relative_pose.position.z, 0.1);
			//position.x = landing_pad_relative_pose.position.x;
			//position.y = landing_pad_relative_pose.position.y;
			//position.z = max(iris_pose.position.z + landing_pad_relative_pose.position.z, 0.1);

			// for visual update
			landing_pad_position_publisher.publish(position);

//			link_state_message.link_name = "ground_plane::landing_pad_estimate_link";
//			link_state_message.pose.position = landing_pad_relative_pose.position;
//			link_state_message.reference_frame = iris_base_link_name;
//			link_state_message.pose.position.x = position.x;
//			link_state_message.pose.position.y = position.y;
//			link_state_message.pose.position.z = position.z;
//			landing_pad_estimate_publisher.publish(link_state_message);

			send_landing_target_message();
		}

		loop_rate.sleep();
	}

	return 0;
}
