#ifndef LANDING_CONTROLLER_H
#define LANDING_CONTROLLER_H

#include <signal.h>
#include <stdio.h>

#include <ros/ros.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/RCOut.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Transform.h>
#include <cmath>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <apriltag_ros/AprilTagDetectionArray.h>

ros::Publisher landing_phase_publisher;
ros::Publisher landing_pad_camera_pose_publisher;
ros::Publisher landing_pad_relative_pose_stamped_publisher;
ros::Publisher plane_displacement_publisher;
ros::Publisher setpoint_raw_local_publisher;
ros::Publisher enable_landing_publisher;
ros::Publisher landing_pad_pixel_displacement_x_publisher;
ros::Publisher landing_pad_yaw_tracking_pid_enable_publisher;
ros::Publisher landing_pad_pixel_displacement_y_publisher;
ros::Publisher landing_pad_yaw_tracking_pid_setpoint_publisher;

std_msgs::Float64 std_msgs_zero;
std_msgs::Bool std_msgs_true;
std_msgs::Bool std_msgs_false;
std_msgs::Float64 std_msgs_float64_msg;

geometry_msgs::Point position_target;
geometry_msgs::PoseStamped landing_pad_camera_pose;
geometry_msgs::PoseStamped landing_pad_relative_pose_stamped;
geometry_msgs::PoseStamped landing_pad_relative_pose_stamped_straightened;
geometry_msgs::PoseStamped local_position_pose_stamped;
geometry_msgs::Vector3 target_velocity;
double target_yaw_rate = 0;
double yaw_displacement = std::nan("1");
double yaw_tracking_control_effort = 0;

double yaw_target = 0;
double normalized_pixel_displacement_x;

double landing_radius = 0.06; 
double descent_region_exponential_coefficient = 0.75;

tf2_ros::Buffer transform_buffer;

// timing
ros::Time last_detection_time(0);
ros::Duration abort_time(0.5);
ros::Duration flex_time(4.0);


geometry_msgs::PoseStamped straighten_pose( const geometry_msgs::PoseStamped & );
void control_effort_n_callback( const std_msgs::Float64::ConstPtr );
void control_effort_e_callback( const std_msgs::Float64::ConstPtr );
void control_effort_u_callback( const std_msgs::Float64::ConstPtr );
void landing_pad_camera_pose_callback(   const geometry_msgs::PoseStamped::ConstPtr );
void set_velocity_target_neu( geometry_msgs::Vector3 );
double plane_distance_to( geometry_msgs::PoseStamped );
double descent_distance = 0;

enum LANDING_PHASES
{
	NOT_LANDING	= 3,
	APPROACH	= 2,
	DESCENT		= 1,
	LANDED		= 0
};
int LANDING_PHASE;

double plane_distance_to_landing_pad = std::nan("1");
bool ENABLE_LANDING = false;
std_msgs::Bool ENABLE_LANDING_msg;
const int ENABLE_LANDING_CHANNEL = 10; // (1-based indexing)

std_msgs::UInt8 LANDING_PHASE_msg;
std_msgs::Float64 plane_distance_to_landing_pad_msg;

bool within_descent_region = false;

#endif
