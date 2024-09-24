#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <unitree_legged_msgs/MotorState.h>
#include <unitree_legged_msgs/MotorCmd.h>
#include <unitree_legged_msgs/LowCmd.h>

#include "unitree_legged_sdk/unitree_legged_sdk.h"

#include "LeggedState.h"
#include "interfaces/BaseInterface.h"
#include "utils/MovingWindowFilter.hpp"
#include "utils/LeggedSafetyChecker.hpp"

#define FOOT_FILTER_WINDOW_SIZE 40

namespace legged {

    class HardwareInterface : public BaseInterface {
    public:
        HardwareInterface(ros::NodeHandle &_nh);

        bool ctrl_update(double dt);

        bool fbk_update(double t, double dt);

        bool send_cmd();

    private:
        ros::Publisher pub_joint_angle;
        ros::Publisher pub_imu;
        sensor_msgs::JointState joint_foot_msg;
        sensor_msgs::Imu imu_msg;

        // hardware
        UNITREE_LEGGED_SDK::UDP udp;
        UNITREE_LEGGED_SDK::Safety safe;
        UNITREE_LEGGED_SDK::LowState unitree_state = {0};
        UNITREE_LEGGED_SDK::LowCmd cmd = {0};

        void udp_init_send();

        void receive_low_state(double dt);

        // for hardware, switch foot order
        Eigen::Matrix<int, NUM_DOF, 1> swap_joint_indices;
        Eigen::Matrix<int, NUM_LEG, 1> swap_foot_indices;

        // hardware foot force filter
        MovingWindowFilter foot_force_filters[NUM_LEG];

        // hardware joint vel filter
        MovingWindowFilter joint_vel_filters[NUM_DOF];

        // hardware safety checker
        LeggedSafetyChecker safety_checker;

        // process mocap data
        int DROP_COUNT = 10; // drop the first 10 data
        int current_count = 0;
        bool first_mocap_received = false;
        double opti_t_prev;
        Eigen::Vector3d initial_opti_euler; // the data we input to the filter is the difference between the current and initial euler angles
        Eigen::Vector3d initial_opti_pos; // the data we input to the filter is the difference between the current and initial position

        ros::Subscriber mocap_sub;

        void opti_callback(const geometry_msgs::PoseStamped::ConstPtr &opti_msg);
    };

}  // namespace legged
