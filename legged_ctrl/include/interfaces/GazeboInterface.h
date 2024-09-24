#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/WrenchStamped.h>
#include <unitree_legged_msgs/MotorState.h>
#include <unitree_legged_msgs/MotorCmd.h>
#include <unitree_legged_msgs/LowCmd.h>

#include "LeggedState.h"
#include "interfaces/BaseInterface.h"
#include "utils/MovingWindowFilter.hpp"
#include "utils/LeggedSafetyChecker.hpp"

namespace legged {

    class GazeboInterface : public BaseInterface {
    public:
        GazeboInterface(ros::NodeHandle &_nh, int robot_type);

        bool ctrl_update(double dt);

        bool fbk_update(double t, double dt);

        bool send_cmd();

        // callback functions
        void gt_pose_callback(const nav_msgs::Odometry::ConstPtr &odom);

        void imu_callback(const sensor_msgs::Imu::ConstPtr &imu);

        void joy_callback(const sensor_msgs::Joy::ConstPtr &joy_msg);

        void FL_hip_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state);

        void FL_thigh_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state);

        void FL_calf_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state);

        void FR_hip_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state);

        void FR_thigh_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state);

        void FR_calf_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state);

        void RL_hip_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state);

        void RL_thigh_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state);

        void RL_calf_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state);

        void RR_hip_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state);

        void RR_thigh_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state);

        void RR_calf_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state);

        void FL_foot_contact_callback(const geometry_msgs::WrenchStamped &force);

        void FR_foot_contact_callback(const geometry_msgs::WrenchStamped &force);

        void RL_foot_contact_callback(const geometry_msgs::WrenchStamped &force);

        void RR_foot_contact_callback(const geometry_msgs::WrenchStamped &force);

    private:
        // 0,  1,  2: FL_hip, FL_thigh, FL_calf
        // 3,  4,  5: FR_hip, FR_thigh, FR_calf
        // 6,  7,  8: RL_hip, RL_thigh, RL_calf
        // 9, 10, 11: RR_hip, RR_thigh, RR_calf
        ros::Publisher pub_joint_cmd[12];
        ros::Subscriber sub_joint_msg[12];
        ros::Publisher pub_euler_d;

        // 0, 1, 2, 3: FL, FR, RL, RR
        ros::Subscriber sub_foot_contact_msg[4];
        ros::Subscriber sub_gt_pose_msg;
        ros::Subscriber sub_imu_msg;

        unitree_legged_msgs::LowCmd low_cmd;

        // filters for IMU
        MovingWindowFilter acc_x;
        MovingWindowFilter acc_y;
        MovingWindowFilter acc_z;
        MovingWindowFilter gyro_x;
        MovingWindowFilter gyro_y;
        MovingWindowFilter gyro_z;

        LeggedSafetyChecker safety_checker;

        // simulate opti track
        int DROP_COUNT = 10;   // drop the first 10 data
        int current_count = 0;
        bool first_mocap_received = false;
        double opti_t_prev;
        Eigen::Vector3d initial_opti_euler; // the data we input to the filter is the difference between the current and initial euler angles
        Eigen::Vector3d initial_opti_pos; // the data we input to the filter is the difference between the current and initial position
    };

}  // namespace legged
