#pragma once

#include <string>

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float64MultiArray.h>

#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>

#include "LeggedState.h"
#include "estimation/BasicKF.h"
#include "utils/Utils.h"
#include "utils/A1Kinematics.h"
#include "utils/Go1Dynamics.h"
#include "utils/MovingWindowFilter.hpp"
#include "casadi_ekf/A1KFCombineLOWithFootTerrain.h"
#include "casadi_ekf/A1KFCombineLOWithFoot.h"

namespace legged {

    class BaseInterface {
    public:
        BaseInterface(ros::NodeHandle &_nh);

        virtual ~BaseInterface() {}

        virtual bool ctrl_update(double dt) = 0;

        virtual bool fbk_update(double t, double dt) = 0;

        virtual bool send_cmd() = 0;

        LeggedState &get_legged_state() { return legged_state; };

        void joy_callback(const sensor_msgs::Joy::ConstPtr &joy_msg);

        // void gain_callback(const std_msgs::Float64MultiArray &gain_msg);

        // process joystick data
        bool joy_update(double dt);

        // process sensor data
        bool sensor_update(double t, double dt);

        // update state estimation
        bool estimation_update(double t, double dt);

        // basic joint torque controller
        bool tau_ctrl_update(double dt);

        ros::NodeHandle nh;
        LeggedState legged_state;

        bool estimator_init = false;

        // KF state estimator
        BasicKF kf;

        // CasADi EKF state estimator
        A1SensorData ekf_data;
        // A1KFCombineLOWithFoot ekf;
        A1KFCombineLOWithFootTerrain ekf;

    private:        
        ros::Subscriber sub_joy_msg;
        ros::Subscriber low_level_gains_msg;

        // old a1 kinematics
        double leg_offset_x[4] = {};
        double leg_offset_y[4] = {};

        // for each leg, there is an offset between body frame and hip motor (fx, fy)
        double motor_offset[4] = {};
        double upper_leg_length[4] = {};
        double lower_leg_length[4] = {};
        std::vector <Eigen::VectorXd> rho_fix_list;
        std::vector <Eigen::VectorXd> rho_opt_list;
        A1Kinematics a1_kin;
        Go1Dynamics go1_dyn;

        // for joint torque control strategy
        Eigen::Matrix<double, 3, NUM_LEG> foot_forces_grf_rel; // reach ground reaction force
        Eigen::Matrix<double, 3, NUM_LEG> foot_pos_target_body;
        Eigen::Matrix<double, 3, NUM_LEG> foot_vel_target_body;
        Eigen::Matrix<double, 3, NUM_LEG> foot_acc_target_body;
        Eigen::Matrix<double, 3, NUM_LEG> foot_pos_error_rel;
        Eigen::Matrix<double, 3, NUM_LEG> foot_vel_error_rel;
        Eigen::Matrix<double, 3, NUM_LEG> foot_forces_kin; // reach target foot location

        // raibert heuristic
        double k;
        Eigen::Vector3d raibert_delta_rel;
        Eigen::Vector3d raibert_delta_abs;

        // filter for terrain adaptation
        MovingWindowFilter recent_contact_pos_x_filter[NUM_LEG];
        MovingWindowFilter recent_contact_pos_y_filter[NUM_LEG];
        MovingWindowFilter recent_contact_pos_z_filter[NUM_LEG];
    };
}  // namespace legged
