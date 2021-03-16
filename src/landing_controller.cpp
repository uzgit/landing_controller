#include <landing_controller.h>

using namespace std;

// ( ͡° ͜ʖ ͡°)

void apriltag3_visual_callback( const apriltag_ros::AprilTagDetectionArray::ConstPtr msg )
{
	int i = 0;
	while( i < msg->detections.size() && msg->detections[i].name != "landing_pad" ) i ++;

	if( i < msg->detections.size () )
	{
		landing_pad_relative_pose_stamped.pose.position = msg->detections[i].position_target_enu;
		yaw_displacement = msg->detections[i].yaw;

		landing_pad_relative_pose_stamped.header.stamp = ros::Time::now();
		landing_pad_relative_pose_stamped_publisher.publish( landing_pad_relative_pose_stamped );
	}
}

void mavros_state_callback(const mavros_msgs::State::ConstPtr msg)
{
	if( msg->connected && msg->armed && msg->guided )
	{
		ENABLE_LANDING = true;
	}
	else
	{
		ENABLE_LANDING = false;
	}

	ENABLE_LANDING_msg.data = ENABLE_LANDING;
	enable_landing_publisher.publish(ENABLE_LANDING_msg);
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

// set 3D position target with respect to the drone's ENU frame
void set_position_target_neuy( geometry_msgs::Point target_position, double yaw )
{
	mavros_msgs::PositionTarget buffer;

	buffer.header.stamp = ros::Time::now();
	buffer.header.frame_id = "world";
	buffer.coordinate_frame = 9;

	buffer.type_mask = 3064;

	buffer.position.x = target_position.y;
	buffer.position.y = -target_position.x;
	buffer.position.z = target_position.z;

	setpoint_raw_local_publisher.publish(buffer);
}

// set position target without changing altitude
void set_position_target_ney( geometry_msgs::Point target_position, double yaw )
{
	mavros_msgs::PositionTarget buffer;

	buffer.header.stamp = ros::Time::now();
	buffer.header.frame_id = "world";
	buffer.coordinate_frame = 9;

	buffer.type_mask = 3064;

	buffer.position.x = target_position.y;
	buffer.position.y = -target_position.x;
	buffer.position.z = 0;

	setpoint_raw_local_publisher.publish(buffer);
}

// set 3D position target with respect to the drone's ENU frame
void set_position_target_neu( geometry_msgs::Point target_position )
{
	mavros_msgs::PositionTarget buffer;

	buffer.header.stamp = ros::Time::now();
	buffer.header.frame_id = "world";
	buffer.coordinate_frame = 9;

	buffer.type_mask = 4088;

	buffer.position.x = target_position.y;
	buffer.position.y = -target_position.x;
	buffer.position.z = target_position.z;

	setpoint_raw_local_publisher.publish(buffer);
}

// set position target without changing altitude
void set_position_target_ne( geometry_msgs::Point target_position )
{
	mavros_msgs::PositionTarget buffer;

	buffer.header.stamp = ros::Time::now();
	buffer.header.frame_id = "world";
	buffer.coordinate_frame = 9;

	buffer.type_mask = 4088;

	buffer.position.x = target_position.y;
	buffer.position.y = -target_position.x;
	buffer.position.z = 0;

	setpoint_raw_local_publisher.publish(buffer);
}

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

void set_velocity_target_neu( geometry_msgs::Vector3 _target_velocity, double _target_yaw_rate )
{
	mavros_msgs::PositionTarget buffer;

	buffer.header.stamp = ros::Time::now();
	buffer.header.frame_id = "world";
	buffer.coordinate_frame = 8; // FRAME_BODY_NED
	buffer.type_mask = 1991; // ignore everything except velocity arguments x, y

	buffer.velocity.x = _target_velocity.x;
	buffer.velocity.y = _target_velocity.y;
	buffer.velocity.z = _target_velocity.z;

	buffer.yaw_rate = _target_yaw_rate;

	setpoint_raw_local_publisher.publish(buffer);
}

void set_velocity_target_neu( double _target_velocity_x, double _target_velocity_y, double _target_velocity_z, double _target_yaw_rate )
{
	mavros_msgs::PositionTarget buffer;

	buffer.header.stamp = ros::Time::now();
	buffer.header.frame_id = "world";
	buffer.coordinate_frame = 8; // FRAME_BODY_NED
	buffer.type_mask = 1991; // ignore everything except velocity arguments x, y

	buffer.velocity.x = _target_velocity_x;
	buffer.velocity.y = _target_velocity_y;
	buffer.velocity.z = _target_velocity_z;

	buffer.yaw_rate = _target_yaw_rate;

	setpoint_raw_local_publisher.publish(buffer);
}

void stay_still()
{
	mavros_msgs::PositionTarget buffer;

	buffer.header.stamp = ros::Time::now();
	buffer.header.frame_id = "world";
	buffer.coordinate_frame = 8; // FRAME_BODY_NED
	buffer.type_mask = 1991;

	buffer.velocity.x = 0;
	buffer.velocity.y = 0;
	buffer.velocity.z = 0;

	buffer.yaw_rate = 0;

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

double plane_distance_to(geometry_msgs::Pose pose)
{
	return sqrt( pow(pose.position.x, 2) + pow(pose.position.y, 2) );
}

double plane_distance_to(geometry_msgs::PoseStamped pose)
{
	return plane_distance_to(pose.pose);
}

void sigint_handler(int signal)
{
	ROS_WARN("Sending 0-velocities...");
	stay_still();
	ROS_WARN("Shutting down landing_controller!");
	ros::shutdown();
}

int main(int argc, char** argv)
{
	// init and say hi
	ros::init(argc, argv, "landing_controller", ros::init_options::NoSigintHandler);
	ROS_INFO("Started landing_controller!");
	signal(SIGINT, sigint_handler);

	// init
	std_msgs_true.data  = true;
	std_msgs_false.data = false;
	std_msgs_zero.data  = 0;

	// create node handle
	ros::NodeHandle node_handle;
	
	// create subscribers
//	ros::Subscriber landing_pad_camera_pose_subscriber = node_handle.subscribe("/landing_pad/camera_pose", 1000, landing_pad_camera_pose_callback);
//	ros::Subscriber local_position_pose_subscriber = node_handle.subscribe("/mavros/local_position/pose", 1000, local_position_pose_callback);
	ros::Subscriber apriltag3_subscriber = node_handle.subscribe("/tag_detections", 1000, apriltag3_visual_callback);

	//**************************************************************************************************************************
	ros::Subscriber mavros_state_subscriber = node_handle.subscribe("/mavros/state", 1000, mavros_state_callback);
//	ros::Subscriber landing_enable_subscriber = node_handle.subscribe("/mavros/rc/out", 1000, landing_enable_callback);
	//**************************************************************************************************************************

	// create publishers
	enable_landing_publisher = node_handle.advertise<std_msgs::Bool>("/landing_controller/enabled", 1000);
	landing_phase_publisher = node_handle.advertise<std_msgs::UInt8>("/landing_phase", 1000);
	landing_pad_camera_pose_publisher = node_handle.advertise<geometry_msgs::PoseStamped>("/landing_pad/camera_pose", 1);
	landing_pad_relative_pose_stamped_publisher = node_handle.advertise<geometry_msgs::PoseStamped>("/landing_pad/relative_pose_stamped", 1000);
	plane_displacement_publisher = node_handle.advertise<std_msgs::Float64>("/landing_pad/plane_displacement", 1000);
	setpoint_raw_local_publisher = node_handle.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 1000);

	LANDING_PHASE = NOT_LANDING;

	// main loop
	ros::Rate loop_rate(10);
	while( ros::ok() )
	{
		// execute callbacks
		ros::spinOnce();

		if( ros::Time::now() - landing_pad_relative_pose_stamped.header.stamp < abort_time )
		{
			// calculate x/y distance to landing pad (leave out altitude)
			plane_distance_to_landing_pad = plane_distance_to( landing_pad_relative_pose_stamped );
			plane_distance_to_landing_pad_msg.data = plane_distance_to_landing_pad;
			plane_displacement_publisher.publish( plane_distance_to_landing_pad_msg );
			
			// determine maximum x/y distance to landing pad within which the drone is allowed to descend according to the descent region
			descent_distance = 0.1 * exp(0.36 * abs( landing_pad_relative_pose_stamped.pose.position.z ));
			
			within_descent_region = plane_distance_to_landing_pad < descent_distance;

			// **************************
			// * landing control policy *
			// **************************
			// if we are in landing mode then we can act according to the landing control policy
			if( ENABLE_LANDING )
			{

				if( within_descent_region && abs( landing_pad_relative_pose_stamped.pose.position.z ) < 0.3 )
				{
					LANDING_PHASE = LANDED;
					
					// clamp to ground
					set_velocity_target_neu(0, 0, -0.1, 0);
				}
				else if( plane_distance_to_landing_pad <= descent_distance )
				{
					LANDING_PHASE = DESCENT;

					// approach in 3D
					set_position_target_neu( landing_pad_relative_pose_stamped.pose.position );
//					set_position_target_neuy( landing_pad_relative_pose_stamped.pose.position, -yaw_displacement );
				}
				else
				{
					LANDING_PHASE = APPROACH;

					// approach in the plane
					set_position_target_ne(landing_pad_relative_pose_stamped.pose.position);
//					set_position_target_ney(landing_pad_relative_pose_stamped.pose.position, -yaw_displacement );
				}
			}
		}
		else if( LANDING_PHASE != NOT_LANDING ) // if the landing pad has just been lost from sight
		{
			// abort the landing
			LANDING_PHASE = NOT_LANDING;

			// stop the drone from moving
			stay_still();
		}

		LANDING_PHASE_msg.data = LANDING_PHASE;
		landing_phase_publisher.publish(LANDING_PHASE_msg);
		loop_rate.sleep();
	}

	return 0;
}
