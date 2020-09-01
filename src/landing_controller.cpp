#include <landing_controller.h>

using namespace std;

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

// ( ͡° ͜ʖ ͡°)

void local_position_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	local_position_pose_stamped = *msg;
}

void landing_pad_camera_pose_callback(const geometry_msgs::PoseStamped::ConstPtr msg)
{
	landing_pad_camera_pose = *msg;
}

void landing_pad_whycon_pose_callback(const geometry_msgs::PoseStamped::ConstPtr msg)
{
	landing_pad_whycon_pose = *msg;
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

	enable_landing_publisher.publish(ENABLE_LANDING);
}

/*
void landing_enable_callback(const mavros_msgs::RCOut::ConstPtr msg)
{
	if( msg->channels[ENABLE_LANDING_CHANNEL] > 1750 )
	{
		ENABLE_LANDING = true;
	}
	else
	{
		ENABLE_LANDING = false;
	}
	
	enable_landing_publisher.publish(ENABLE_LANDING);	
}
*/

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

double plane_distance_to(geometry_msgs::PoseStamped pose)
{
	return sqrt( pow(pose.pose.position.x, 2) + pow(pose.pose.position.y, 2) );
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
	ros::Subscriber landing_pad_camera_pose_subscriber = node_handle.subscribe("/landing_pad/camera_pose", 1000, landing_pad_camera_pose_callback);
	ros::Subscriber local_position_pose_subscriber = node_handle.subscribe("/mavros/local_position/pose", 1000, local_position_pose_callback);

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

	// for transforms
	static tf2_ros::TransformListener transform_listener(transform_buffer);

	LANDING_PHASE = NOT_LANDING;

	// main loop
	ros::Rate loop_rate(10);
	while( ros::ok() )
	{
		// execute callbacks
		ros::spinOnce();

		if( ros::Time::now() - landing_pad_camera_pose.header.stamp < abort_time )
		{
			// transform pose in camera frame to pose relative to the drone in ENU
			try
			{
				// transform detected pose from camera_frame to body_enu (this takes into consideration the gimbal yaw angle)
				landing_pad_relative_pose_stamped 		= transform_buffer.transform(landing_pad_camera_pose, "body_enu", ros::Duration(0.1));

				// transform from body_enu to body_enu_straightened
				// (this puts the drone's position in terms of the landing pad's coordinate system with the assumption that the landing pad is level)
				landing_pad_relative_pose_stamped_straightened	= straighten_pose(landing_pad_relative_pose_stamped);

				// don't land directly on the landing pad (in order to keep it in view during the entire descent)
				landing_pad_relative_pose_stamped.pose.position.y -= 0.5;

				// publish relative pose
				landing_pad_relative_pose_stamped_publisher.publish(landing_pad_relative_pose_stamped);
			}
			catch( tf2::TransformException &exception )
			{
				ROS_WARN("Transform exception in landing_controller main loop: %s", exception.what());
			}

			// calculate x/y distance to landing pad (leave out altitude)
			plane_distance_to_landing_pad = plane_distance_to( landing_pad_relative_pose_stamped );
			plane_displacement_publisher.publish( plane_distance_to_landing_pad );
			
			// determine maximum x/y distance to landing pad within which the drone is allowed to descend according to the descent region
			descent_distance = 0.24 * exp(0.36 * abs( landing_pad_relative_pose_stamped.pose.position.z ));
			
			within_descent_region = plane_distance_to_landing_pad < descent_distance;

			// **************************
			// * landing control policy *
			// **************************
			// if we are in landing mode then we can act according to the landing control policy
			if( ENABLE_LANDING )
			{
				if( within_descent_region && abs( landing_pad_relative_pose_stamped.pose.position.z ) < 0.09 )
				{
					LANDING_PHASE = LANDED;
					
					// clamp to ground
					set_velocity_target_neu(0, 0, -0.5, 0);
				}
				else if( plane_distance_to_landing_pad <= descent_distance )
				{
					LANDING_PHASE = DESCENT;

					// approach in 3D
					set_position_target_neu( landing_pad_relative_pose_stamped.pose.position );
				}
				else
				{
					LANDING_PHASE = APPROACH;

					// approach in the plane
					set_position_target_ne(landing_pad_relative_pose_stamped.pose.position);
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

		landing_phase_publisher.publish(LANDING_PHASE);
		loop_rate.sleep();
	}

	return 0;
}
