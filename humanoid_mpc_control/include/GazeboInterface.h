#pragma once

#include <string>
#include <iostream>
#include <vector>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Imu.h>
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
    ros::Publisher pub_torso_cmd;
    ros::Publisher foot_pos_world_pub[4];

    ros::Subscriber sub_joint_states[ACT_JOINTS];
    ros::Subscriber sub_torso_com_odom;
    ros::Subscriber sub_torso_imu;
    ros::Subscriber sub_joy;

    // ROS messages
    unitree_legged_msgs::LowCmd low_cmd;
    nav_msgs::Odometry torso_com_cmd;
    geometry_msgs::Point foot_pos_world_msg[4];

    // Callback functions
    void torso_com_odom_callback(const nav_msgs::Odometry::ConstPtr& odom_msg);
    void torso_imu_callback(const sensor_msgs::Imu::ConstPtr& imu_msg);
    void joy_callback(const sensor_msgs::Joy::ConstPtr& joy_msg);

    void left_hip_yaw_state_callback(const unitree_legged_msgs::MotorState& joint_state_msg);
    void left_hip_abad_state_callback(const unitree_legged_msgs::MotorState& joint_state_msg);
    void left_hip_pitch_state_callback(const unitree_legged_msgs::MotorState& joint_state_msg);
    void left_knee_state_callback(const unitree_legged_msgs::MotorState& joint_state_msg);
    void left_ankle_state_callback(const unitree_legged_msgs::MotorState& joint_state_msg);

    void right_hip_yaw_state_callback(const unitree_legged_msgs::MotorState& joint_state_msg);
    void right_hip_abad_state_callback(const unitree_legged_msgs::MotorState& joint_state_msg);
    void right_hip_pitch_state_callback(const unitree_legged_msgs::MotorState& joint_state_msg);
    void right_knee_state_callback(const unitree_legged_msgs::MotorState& joint_state_msg);
    void right_ankle_state_callback(const unitree_legged_msgs::MotorState& joint_state_msg);
};
