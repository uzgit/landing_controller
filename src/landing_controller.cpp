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

void link_states_callback(const gazebo_msgs::LinkStates::ConstPtr& msg)
{
	int i = 0;
	while( i < msg->name.size() && msg->name[i] != "iris_demo::iris_demo::gimbal_small_2d::tilt_link")
	{
		i ++;
	}

	camera_twist = msg->twist[i];

	i = 0;
	while( i < msg->name.size() && msg->name[i] != "landing_pad_combo::landing_pad::link" )
	{
		i ++;
	}

	landing_pad_gazebo_pose  = msg->pose[i];
	landing_pad_gazebo_twist = msg->twist[i];
}

void model_states_callback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
	int i = 0;
	while( i < msg->name.size() && msg->name[i] != "iris_demo")
	{
		i ++;
	}

	iris_pose  = msg->pose[i];
	iris_twist = msg->twist[i];

	tf2::Quaternion rotation;
	tf2::fromMsg(msg->pose[i].orientation, rotation);
	tf2::Matrix3x3(rotation).getEulerYPR(iris_yaw, iris_pitch, iris_roll);
}

void imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
	static tf2_ros::TransformBroadcaster transform_broadcaster;
	geometry_msgs::TransformStamped transform_stamped_message;
	transform_stamped_message.header.stamp = ros::Time::now();
	transform_stamped_message.header.frame_id = "body_enu";
	transform_stamped_message.child_frame_id = "global_enu";
	
	tf2::Quaternion orientation, yaw;
	tf2::fromMsg(msg->orientation, orientation);
//	orientation *= tf2::Quaternion(0, 0, 1, 0);
	orientation = orientation.inverse();
	geometry_msgs::Quaternion orientation_message = tf2::toMsg(orientation);

/*	
	transform_stamped_message.transform.translation.x = -iris_pose.position.x;
	transform_stamped_message.transform.translation.y = -iris_pose.position.y;
	transform_stamped_message.transform.translation.z = -iris_pose.position.z;
*/	
	transform_stamped_message.transform.rotation = orientation_message;
	transform_broadcaster.sendTransform(transform_stamped_message);
}

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

void control_effort_yaw_callback(const std_msgs::Float64::ConstPtr &msg)
{
	target_yaw_rate = msg->data;
}

void yaw_displacement_callback(const std_msgs::Float64::ConstPtr &msg)
{
	yaw_displacement = msg->data;
}

void pid_parameters_1_callback(const geometry_msgs::Vector3::ConstPtr &msg)
{
	pid_parameters_1.x = msg->x;
	pid_parameters_1.y = msg->y;
	pid_parameters_1.z = msg->z;

	ROS_INFO("Reconfiguring pid parameters to (%0.2f %0.2f %0.2f) for phase 1.", msg->x, msg->y, msg->z);
}

void pid_parameters_2_callback(const geometry_msgs::Vector3::ConstPtr &msg)
{
	pid_parameters_2.x = msg->x;
	pid_parameters_2.y = msg->y;
	pid_parameters_2.z = msg->z;

	ROS_INFO("Reconfiguring pid parameters to (%0.2f %0.2f %0.2f) for phase 2.", msg->x, msg->y, msg->z);
}

void pid_parameters_3_callback(const geometry_msgs::Vector3::ConstPtr &msg)
{
	pid_parameters_3.x = msg->x;
	pid_parameters_3.y = msg->y;
	pid_parameters_3.z = msg->z;

	ROS_INFO("Reconfiguring pid parameters to (%0.2f %0.2f %0.2f) for phase 3.", msg->x, msg->y, msg->z);
}

void pid_parameters_u_callback(const geometry_msgs::Vector3::ConstPtr &msg)
{
	pid_parameters_u.x = msg->x;
	pid_parameters_u.y = msg->y;
	pid_parameters_u.z = msg->z;

	pid_reconfigure_u_publisher.publish(pid_parameters_u);
	ROS_INFO("Reconfiguring pid parameters to (%0.2f %0.2f %0.2f) for up dimension.", msg->x, msg->y, msg->z);
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

void landing_pad_camera_pose_callback(const geometry_msgs::PoseStamped::ConstPtr msg)
{
	landing_pad_camera_pose = *msg;
}

void landing_pad_whycon_pose_callback(const geometry_msgs::PoseStamped::ConstPtr msg)
{
	whycon_detected = true;
	landing_pad_whycon_pose = *msg;

	try
	{
		geometry_msgs::PoseStamped buffer = transform_buffer.transform(landing_pad_whycon_pose, "body_enu", ros::Duration(0.2));

		whycon_odometry_message.header = buffer.header;
		whycon_odometry_message.child_frame_id = buffer.header.frame_id;
		whycon_odometry_message.pose.pose = buffer.pose;
		for(int i = 0; i < 36; i ++)
		{
			whycon_odometry_message.pose.covariance[i] = 0;//1e-10;
		}
	}
	catch( tf2::TransformException &exception )
	{
		ROS_WARN(exception.what());
	}
}

void landing_pad_apriltag_pose_callback(const geometry_msgs::PoseStamped::ConstPtr msg)
{
	apriltag_detected = true;
	landing_pad_apriltag_pose = *msg;
	
	try
	{
		geometry_msgs::PoseStamped buffer = transform_buffer.transform(landing_pad_apriltag_pose, "body_enu", ros::Duration(0.1));
		buffer = straighten_pose(buffer);
	}
	catch( ... )
	{
		ROS_WARN("Transform exception in apriltag callback.");
	}
}

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

void set_velocity_target_neu( geometry_msgs::Vector3 _target_velocity )
{
	mavros_msgs::PositionTarget buffer;

	buffer.header.stamp = ros::Time::now();
	buffer.header.frame_id = "world";
	buffer.coordinate_frame = 8; // FRAME_BODY_NED
	buffer.type_mask = 4039; // ignore everything except velocity arguments x, y
//	buffer.type_mask = 1991; // ignore everything except velocity arguments x, y

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
//	buffer.type_mask = 4039; // ignore everything except velocity arguments x, y
	buffer.type_mask = 1991; // ignore everything except velocity arguments x, y

	buffer.velocity.x = _target_velocity.x;
	buffer.velocity.y = _target_velocity.y;
	buffer.velocity.z = _target_velocity.z;

	buffer.yaw_rate = _target_yaw_rate;

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

//	geometry_msgs::Vector3 pid_parameters;

	pid_parameters_1.x = -0.7;
	pid_parameters_1.y = -0.0;
	pid_parameters_1.z = -0.4;
	pid_parameters_2.x = -0.3;
	pid_parameters_2.y = -0.0;
	pid_parameters_2.z = -0.8;
	pid_parameters_3.x = -0.3;
	pid_parameters_3.y = -0.0;
	pid_parameters_3.z = -0.95;
	
	// create node handle
	ros::NodeHandle node_handle;
	
	// create subscribers
	ros::Subscriber imu_subscriber = node_handle.subscribe("/imu", 1000, imu_callback);
	ros::Subscriber landing_pad_camera_pose_subscriber = node_handle.subscribe("/landing_pad/camera_pose", 1000, landing_pad_camera_pose_callback);
	ros::Subscriber landing_pad_whycon_pose_subscriber = node_handle.subscribe("/landing_pad/whycon_pose", 1000, landing_pad_whycon_pose_callback);
	ros::Subscriber control_effort_n_subscriber = node_handle.subscribe("/pid/control_effort/n", 1000, control_effort_n_callback);
	ros::Subscriber control_effort_e_subscriber = node_handle.subscribe("/pid/control_effort/e", 1000, control_effort_e_callback);
	ros::Subscriber control_effort_u_subscriber = node_handle.subscribe("/pid/control_effort/u", 1000, control_effort_u_callback);
	ros::Subscriber control_effort_yaw_subscriber = node_handle.subscribe("/pid/control_effort/yaw_rate", 1000, control_effort_yaw_callback);
	ros::Subscriber yaw_displacement_subscriber = node_handle.subscribe("/landing_pad/yaw_displacement", 1000, yaw_displacement_callback);
	ros::Subscriber model_states_subscriber = node_handle.subscribe("/gazebo/model_states", 1000, model_states_callback);
	ros::Subscriber link_states_subscriber  = node_handle.subscribe("/gazebo/link_states", 1000, link_states_callback);
	ros::Subscriber landing_enable_subscriber = node_handle.subscribe("/mavros/rc/out", 1000, landing_enable_callback);
	ros::Subscriber pid_parameters_1_subscriber = node_handle.subscribe("/landing_controller/pid_parameters_1", 1000, pid_parameters_1_callback);
	ros::Subscriber pid_parameters_2_subscriber = node_handle.subscribe("/landing_controller/pid_parameters_2", 1000, pid_parameters_2_callback);
	ros::Subscriber pid_parameters_3_subscriber = node_handle.subscribe("/landing_controller/pid_parameters_3", 1000, pid_parameters_3_callback);
	ros::Subscriber pid_parameters_u_subscriber = node_handle.subscribe("/landing_controller/pid_parameters_u", 1000, pid_parameters_u_callback);

	// create publishers
	enable_landing_publisher = node_handle.advertise<std_msgs::Bool>("/landing_controller/enabled", 1000);
	landing_phase_publisher = node_handle.advertise<std_msgs::UInt8>("/landing_phase", 1000);
	landing_pad_camera_pose_publisher = node_handle.advertise<geometry_msgs::PoseStamped>("/landing_pad/camera_pose", 1000);
	landing_pad_apriltag_global_pose_stamped_publisher = node_handle.advertise<geometry_msgs::PoseStamped>("/landing_pad/apriltag_global_pose_stamped", 1000);
	landing_pad_whycon_global_pose_stamped_publisher = node_handle.advertise<geometry_msgs::PoseStamped>("/landing_pad/whycon_global_pose_stamped", 1000);
	landing_pad_global_pose_stamped_publisher = node_handle.advertise<geometry_msgs::PoseStamped>("/landing_pad/global_pose_stamped", 1000);
	landing_pad_relative_pose_stamped_publisher = node_handle.advertise<geometry_msgs::PoseStamped>("/landing_pad/relative_pose_stamped", 1000);
	plane_displacement_publisher = node_handle.advertise<std_msgs::Float64>("/landing_pad/plane_displacement", 1000);
	setpoint_raw_local_publisher = node_handle.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 1000);
	displacement_n_publisher = node_handle.advertise<std_msgs::Float64>("/landing_pad/body_neu/displacement/n", 1000);
	displacement_e_publisher = node_handle.advertise<std_msgs::Float64>("/landing_pad/body_neu/displacement/e", 1000);
	displacement_u_publisher = node_handle.advertise<std_msgs::Float64>("/landing_pad/body_neu/displacement/u", 1000);
	displacement_yaw_publisher = node_handle.advertise<std_msgs::Float64>("/landing_pad/body_neu/displacement/yaw", 1000);
	displacement_n_setpoint_publisher = node_handle.advertise<std_msgs::Float64>("/pid/body_neu/setpoint/n", 1000);
	displacement_e_setpoint_publisher = node_handle.advertise<std_msgs::Float64>("/pid/body_neu/setpoint/e", 1000);
	displacement_u_setpoint_publisher = node_handle.advertise<std_msgs::Float64>("/pid/body_neu/setpoint/u", 1000);
	displacement_yaw_setpoint_publisher = node_handle.advertise<std_msgs::Float64>("/pid/body_neu/setpoint/yaw_rate", 1000);
	pid_enable_n_publisher = node_handle.advertise<std_msgs::Bool>("/pid/pid_enable/n/", 1000);
	pid_enable_e_publisher = node_handle.advertise<std_msgs::Bool>("/pid/pid_enable/e/", 1000);
	pid_enable_u_publisher = node_handle.advertise<std_msgs::Bool>("/pid/pid_enable/u/", 1000);
	pid_enable_yaw_publisher = node_handle.advertise<std_msgs::Bool>("/pid/pid_enable/yaw_rate/", 1000);
	pid_reconfigure_e_publisher = node_handle.advertise<geometry_msgs::Vector3>("/pid/reconfigure_topic/e", 1000);
	pid_reconfigure_n_publisher = node_handle.advertise<geometry_msgs::Vector3>("/pid/reconfigure_topic/n", 1000);
	pid_reconfigure_u_publisher = node_handle.advertise<geometry_msgs::Vector3>("/pid/reconfigure_topic/u", 1000);
	pid_reconfigure_yaw_publisher = node_handle.advertise<geometry_msgs::Vector3>("/pid/reconfigure_topic/yaw", 1000);

	// for transforms
	static tf2_ros::TransformListener transform_listener(transform_buffer);

	LANDING_PHASE = NOT_LANDING;
	pid_enable_u_publisher.publish( std_msgs_false );

	ros::Rate loop_rate(40);

	// main loop
	while( ros::ok() )
	{
		// callbacks
		ros::spinOnce();

		if( ros::Time::now() - landing_pad_camera_pose.header.stamp < abort_time )
		{
			last_target_send_time = ros::Time::now();
			// transform pose in camera frame to pose relative to the drone in ENU
			try
			{
				// determine how to react to the landing pad detection
				landing_pad_relative_pose_stamped 		= transform_buffer.transform(landing_pad_camera_pose, "body_enu", ros::Duration(0.1));
				landing_pad_relative_pose_stamped_straightened	= straighten_pose(landing_pad_relative_pose_stamped);

				// publish relative pose
				landing_pad_relative_pose_stamped_publisher.publish(landing_pad_relative_pose_stamped);

				landing_pad_global_pose_stamped = transform_buffer.transform(landing_pad_camera_pose, "global_enu", ros::Duration(0.1));
				landing_pad_global_pose_stamped_publisher.publish(landing_pad_global_pose_stamped);
			}
			catch( tf2::TransformException &exception )
			{
				ROS_WARN("main loop: %s", exception.what());
			}

			try
			{
				if( apriltag_detected )
				{
					landing_pad_apriltag_global_pose_stamped = transform_buffer.transform(landing_pad_apriltag_pose, "global_enu", ros::Duration(0.1));
					landing_pad_apriltag_global_pose_stamped_publisher.publish(landing_pad_apriltag_global_pose_stamped);
				}
			}
			catch( tf2::TransformException &exception )
			{
				ROS_WARN("main loop: %s", exception.what());
			}
			try
			{
				if( whycon_detected )
				{
					landing_pad_whycon_global_pose_stamped = transform_buffer.transform(landing_pad_whycon_pose, "global_enu", ros::Duration(0.1));
					landing_pad_whycon_global_pose_stamped_publisher.publish(landing_pad_whycon_global_pose_stamped);
				}
			}
			catch( tf2::TransformException &exception )
			{
				ROS_WARN("main loop: %s", exception.what());
			}


			// Direct the drone towards the landing pad
			
			plane_distance_to_landing_pad = plane_distance_to( landing_pad_relative_pose_stamped );
			plane_displacement_publisher.publish(plane_distance_to_landing_pad);
			// tight descent
			descent_distance = 0.24 * exp(0.36 * abs(landing_pad_relative_pose_stamped.pose.position.z));
//			descent_distance = 0.12 * exp(0.36 * abs(landing_pad_relative_pose_stamped.pose.position.z)); // initial
//			descent_distance = 0.15 * exp(0.4 * abs(landing_pad_relative_pose_stamped.pose.position.z)); // better
			double close_approach_distance = 3.0 * descent_distance;

			if( ENABLE_LANDING )
			{
				// control policy
				// commit to the landing entirely
				if( LANDING_PHASE == DESCENT && abs(landing_pad_relative_pose_stamped.pose.position.z) < 0.14 )
				{
					pid_enable_yaw_publisher.publish( std_msgs_false );
					pid_enable_e_publisher.publish( std_msgs_false );
					pid_enable_n_publisher.publish( std_msgs_false );
					pid_enable_u_publisher.publish( std_msgs_false );

					target_yaw_rate = 0;
					target_velocity.z = -10;

					LANDING_PHASE = LANDED;
					ROS_WARN("LANDING_PHASE: LANDED");
				}	
				else if( LANDING_PHASE == YAW_ALIGNMENT && plane_distance_to_landing_pad < descent_distance && abs(yaw_displacement) < 0.0872665 && abs(landing_pad_relative_pose_stamped.pose.position.z) < 5)
				{
					pid_enable_u_publisher.publish( std_msgs_true );
					pid_enable_yaw_publisher.publish( std_msgs_true );
					target_yaw_rate = 0;
					
					LANDING_PHASE = DESCENT;
					ROS_INFO_STREAM(yaw_displacement);
					ROS_WARN("LANDING_PHASE: DESCENT");
				}
				else if( LANDING_PHASE == CLOSE_APPROACH && plane_distance_to_landing_pad < descent_distance )
				{
					pid_enable_yaw_publisher.publish( std_msgs_true );
//					pid_enable_e_publisher.publish( std_msgs_true );
//					pid_enable_n_publisher.publish( std_msgs_true );
//					pid_enable_u_publisher.publish( std_msgs_true );

					LANDING_PHASE = YAW_ALIGNMENT;
//					ROS_INFO_STREAM(yaw_displacement);
					ROS_WARN("LANDING_PHASE: YAW_ALIGNMENT");
	
					pid_parameters.x = -0.3;
					pid_parameters.y = -0.0; // 0 m/s
//					pid_parameters.y = -0.022;// 1 m/s
					pid_parameters.z = -0.95; //-0.7;

/*
					pid_parameters.x = -0.3;
					pid_parameters.y = -0.0; // 0 m/s
					pid_parameters.z = -0.8; //-0.7;
*/				
//					double tmp = pid_parameters_3.y;
//					pid_parameters_3.y = 0;
					pid_reconfigure_e_publisher.publish( pid_parameters_3 );

//					pid_parameters_3.y = tmp;
					pid_reconfigure_n_publisher.publish( pid_parameters_3 );
				}
				else if( LANDING_PHASE == APPROACH && plane_distance_to_landing_pad < close_approach_distance)
				{
					pid_enable_e_publisher.publish( std_msgs_true );
					pid_enable_n_publisher.publish( std_msgs_true );
					pid_enable_u_publisher.publish( std_msgs_false );
					target_velocity.z = 0;
					pid_enable_yaw_publisher.publish( std_msgs_false );

// better
					pid_parameters.x = -0.3;//-0.45;
					pid_parameters.y = -0.0;// -0.0;
					pid_parameters.z = -0.8;//-0.9; //-0.7;
/*
					pid_parameters.x = -0.3;//-0.45;
					pid_parameters.y = -0.0;// -0.0;
					pid_parameters.z = -0.7;//-0.9; //-0.7;
*/
//					double tmp = pid_parameters_2.y;
//					pid_parameters_2.y = 0;
					pid_reconfigure_e_publisher.publish( pid_parameters_2 );

//					pid_parameters_2.y = tmp;
					pid_reconfigure_n_publisher.publish( pid_parameters_2 );

					LANDING_PHASE = CLOSE_APPROACH;
					ROS_WARN("LANDING_PHASE: CLOSE_APPROACH");
				}
				else if( LANDING_PHASE > APPROACH )
				{
					pid_enable_e_publisher.publish( std_msgs_true );
					pid_enable_n_publisher.publish( std_msgs_true );
					pid_enable_u_publisher.publish( std_msgs_false );
					target_velocity.z = 0;
					pid_enable_yaw_publisher.publish( std_msgs_false );

					pid_parameters.x = -0.7; //-0.7;
					pid_parameters.y = -0.0;// -0.0;
					pid_parameters.z = -0.4;//-0.35; //-0.7;

					pid_reconfigure_e_publisher.publish( pid_parameters_1 );
					pid_reconfigure_n_publisher.publish( pid_parameters_1 );
/*
					pid_reconfigure_e_publisher.publish( pid_parameters_3 );
					pid_reconfigure_n_publisher.publish( pid_parameters_3 );
*/

					LANDING_PHASE = APPROACH;
					ROS_WARN("LANDING_PHASE: APPROACH");
				} // end LANDING_PHASE selection

				// handle descent and yaw correction during normal landing phases
				if( LANDING_PHASE != LANDED )
				{
					// descend if within descent region
					if( plane_distance_to_landing_pad < descent_distance )
					{
						pid_enable_u_publisher.publish( std_msgs_true );
						DESCENDING = true;
					}
					// stop descent if not within descent region
					else
					{
						pid_enable_u_publisher.publish( std_msgs_false );
						target_velocity.z = 0;
						DESCENDING = false;
					}

					// stop yaw correction if aligned
/*
					if( abs(yaw_displacement) < 0.0349066 ) // 2 degrees // 0.0875665 ) // 5 degrees
					{
						pid_enable_yaw_publisher.publish(std_msgs_false);
						target_yaw_rate = 0;
					}
*/
/*
					else if( plane_distance_to_landing_pad < 0.5 * descent_distance )
					{
						pid_enable_yaw_publisher.publish(std_msgs_true);
					}
*/
				}
	//			ROS_INFO_STREAM(plane_distance_to_landing_pad);
/*
				if( LANDING_PHASE == YAW_ALIGNMENT )
				{
					if( abs(landing_pad_relative_pose_stamped.pose.position.z) < 5 )
					{
						pid_enable_u_publisher.publish( std_msgs_false );
						target_velocity.z = 0;
					}
					else
					{
						pid_enable_u_publisher.publish( std_msgs_true );
					}
				}
*/

				// publish PID states
				displacement_n_publisher.publish( landing_pad_relative_pose_stamped.pose.position.y );
				displacement_e_publisher.publish( landing_pad_relative_pose_stamped.pose.position.x );
				displacement_u_publisher.publish( landing_pad_relative_pose_stamped.pose.position.z );
				// yaw displacement is published by gimbal controller

				// publish PID setpoints (always 0)
				displacement_n_setpoint_publisher.publish( std_msgs_zero );
				displacement_e_setpoint_publisher.publish( std_msgs_zero );
				displacement_u_setpoint_publisher.publish( std_msgs_zero );
				displacement_yaw_setpoint_publisher.publish( std_msgs_zero );
			
				set_velocity_target_neu( target_velocity, target_yaw_rate );
			} //endif( ENABLE_LANDING )
			else
			{
				if( LANDING_PHASE != NOT_LANDING )
				{
					pid_enable_n_publisher.publish( std_msgs_false );
					pid_enable_e_publisher.publish( std_msgs_false );
					pid_enable_u_publisher.publish( std_msgs_false );

					target_velocity.x = 0;
					target_velocity.y = 0;
					target_velocity.z = 0;
					
					set_velocity_target_neu( target_velocity, target_yaw_rate );

					LANDING_PHASE = NOT_LANDING;
				}
			}
		}
		else
		{
			if( LANDING_PHASE != NOT_LANDING )
			{
				pid_enable_n_publisher.publish( std_msgs_false );
				pid_enable_e_publisher.publish( std_msgs_false );
				pid_enable_u_publisher.publish( std_msgs_false );

				target_velocity.x = 0;
				target_velocity.y = 0;
				target_velocity.z = 0;
				
				set_velocity_target_neu( target_velocity, target_yaw_rate );

				LANDING_PHASE = NOT_LANDING;
			}
		}
		landing_phase_publisher.publish(LANDING_PHASE);
		loop_rate.sleep();
	}

	return 0;
}
