#ifndef LANDING_CONTROLLER_H
#define LANDING_CONTROLLER_H

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

ros::Publisher landing_pad_camera_pose_publisher;
ros::Publisher landing_pad_relative_pose_stamped_publisher;
ros::Publisher landing_pad_global_pose_stamped_publisher;
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
double target_yaw_rate = 0;
double yaw_displacement = 0;

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

#endif
