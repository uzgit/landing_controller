#ifndef LANDING_CONTROLLER_H
#define LANDING_CONTROLLER_H

#define LOG 1

#if LOG
#include <stdio.h>
#include <iomanip>
#include <stdlib.h>
#include <fstream>
#include <ctime>
#include <sstream>
#endif

#include <ros/ros.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/RCOut.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt8.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/LinkStates.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/transform_datatypes.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf2/LinearMath/Transform.h>
#include <cmath>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

ros::Publisher landing_phase_publisher;
ros::Publisher landing_pad_camera_pose_publisher;
ros::Publisher landing_pad_relative_pose_stamped_publisher;
ros::Publisher landing_pad_global_pose_stamped_publisher;
ros::Publisher landing_pad_apriltag_global_pose_stamped_publisher;
ros::Publisher landing_pad_whycon_global_pose_stamped_publisher;
ros::Publisher plane_displacement_publisher;
ros::Publisher setpoint_raw_local_publisher;
ros::Publisher displacement_n_publisher;
ros::Publisher displacement_e_publisher;
ros::Publisher displacement_u_publisher;
ros::Publisher displacement_yaw_publisher;
ros::Publisher displacement_n_setpoint_publisher;
ros::Publisher displacement_e_setpoint_publisher;
ros::Publisher displacement_u_setpoint_publisher;
ros::Publisher displacement_yaw_setpoint_publisher;
ros::Publisher pid_enable_n_publisher;
ros::Publisher pid_enable_e_publisher;
ros::Publisher pid_enable_u_publisher;
ros::Publisher pid_enable_yaw_publisher;
ros::Publisher pid_reconfigure_e_publisher;
ros::Publisher pid_reconfigure_n_publisher;
ros::Publisher pid_reconfigure_u_publisher;
ros::Publisher pid_reconfigure_yaw_publisher;
ros::Publisher vo_whycon_publisher;
ros::Publisher vo_apriltag_publisher;
ros::Publisher landing_pad_pose_filtered_publisher;
ros::Publisher enable_landing_publisher;

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
geometry_msgs::PoseStamped landing_pad_relative_pose_filtered;
geometry_msgs::PoseStamped landing_pad_relative_pose_stamped_straightened;
geometry_msgs::PoseStamped landing_pad_relative_pose_absolute_yaw_stamped;
geometry_msgs::PoseStamped landing_pad_global_pose_stamped;
geometry_msgs::PoseStamped landing_pad_apriltag_global_pose_stamped;
geometry_msgs::PoseStamped landing_pad_whycon_global_pose_stamped;
geometry_msgs::PoseStamped local_position_pose_stamped;
geometry_msgs::Vector3 target_velocity;
double target_yaw_rate = 0;
double yaw_displacement = std::nan("1");
nav_msgs::Odometry apriltag_odometry_message;
nav_msgs::Odometry whycon_odometry_message;

tf2_ros::Buffer transform_buffer;

bool stopped = false;

// timing
ros::Time last_detection_time(0);
ros::Duration abort_time(0.5);
ros::Time last_target_send_time(0);
ros::Duration target_send_interval(0.5);

ros::Publisher landing_pad_estimate_publisher;

geometry_msgs::PoseStamped straighten_pose( const geometry_msgs::PoseStamped & );
void control_effort_n_callback( const std_msgs::Float64::ConstPtr );
void control_effort_e_callback( const std_msgs::Float64::ConstPtr );
void control_effort_u_callback( const std_msgs::Float64::ConstPtr );
void landing_pad_whycon_pose_callback(   const geometry_msgs::PoseStamped::ConstPtr );
void landing_pad_camera_pose_callback(   const geometry_msgs::PoseStamped::ConstPtr );
void landing_pad_apriltag_pose_callback( const geometry_msgs::PoseStamped::ConstPtr );
void set_velocity_target_neu( geometry_msgs::Vector3 );
double plane_distance_to( geometry_msgs::PoseStamped );
double descent_distance = 0;

enum LANDING_PHASES
{
	NOT_LANDING	= 5,
	APPROACH	= 4,
	CLOSE_APPROACH	= 3,
	YAW_ALIGNMENT	= 2,
	DESCENT		= 1,
	LANDED		= 0
};

geometry_msgs::Vector3 pid_parameters;
geometry_msgs::Vector3 pid_parameters_1;
geometry_msgs::Vector3 pid_parameters_2;
geometry_msgs::Vector3 pid_parameters_3;
geometry_msgs::Vector3 pid_parameters_u;

int LANDING_PHASE = NOT_LANDING;
double plane_distance_to_landing_pad = std::nan("1");
bool ENABLE_LANDING = false;
const int ENABLE_LANDING_CHANNEL = 10; // (1-based indexing)

#if LOG
// for analysis
bool whycon_detected = false;
bool apriltag_detected = false;
geometry_msgs::Pose iris_pose;
geometry_msgs::Twist iris_twist;
geometry_msgs::Twist camera_twist;
geometry_msgs::PoseStamped whycon_pose_temp;
geometry_msgs::PoseStamped apriltag_pose_temp;
sensor_msgs::Imu imu_message;
double iris_pitch = 0;
double iris_roll = 0;
double iris_yaw = 0;

geometry_msgs::Pose  landing_pad_gazebo_pose;
geometry_msgs::Twist landing_pad_gazebo_twist;
bool DESCENDING = false;
int energy_consumed = 0;

double latitude;
double longitude;
double altitude;

#endif

#endif
