#pragma once

#include <string>
#include <iostream>
#include <vector>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <unitree_legged_msgs/LowCmd.h>
#include <unitree_legged_msgs/MotorState.h>

#include "RobotState.h"
#include "Utils.h"

using namespace std;
using namespace robot;

class GazeboInterface {
   public:
    GazeboInterface(ros::NodeHandle& nh, string robot_name);

    void ctrl_update();
    void fbk_update();
    void send_cmd();
    void debug_broadcast();

    // Robot state
    RobotState robot_state;

   private:
    // ROS publishers & subscribers
    ros::Publisher pub_joint_cmd[ACT_JOINTS];
    ros::Publisher pub_wheel_tau_cmd[2];
    ros::Publisher pub_torso_cmd;
    ros::Publisher foot_pos_world_pub[4];

    ros::Subscriber sub_joint_states[ACT_JOINTS];
    ros::Subscriber sub_torso_com_odom;
    ros::Subscriber sub_torso_imu;
    ros::Subscriber sub_wheel_vel;
    ros::Subscriber sub_joy;
    ros::Subscriber sub_falling_pose;

    // ROS messages
    unitree_legged_msgs::LowCmd low_cmd;
    nav_msgs::Odometry torso_com_cmd;
    std_msgs::Float64 wheel_tau_cmd_msg[2];
    geometry_msgs::Point foot_pos_world_msg[4];

    // Callback functions
    void torso_com_odom_callback(const nav_msgs::Odometry::ConstPtr& odom_msg);
    void torso_imu_callback(const sensor_msgs::Imu::ConstPtr& imu_msg);
    void wheel_vel_callback(const sensor_msgs::JointState::ConstPtr& joint_state_msg);
    void joy_callback(const sensor_msgs::Joy::ConstPtr& joy_msg);
    void falling_pose_callback(const geometry_msgs::Pose::ConstPtr& pose_msg);
};
