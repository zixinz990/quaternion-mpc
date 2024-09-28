#pragma once

#include <algorithm>

#include <Eigen/Dense>
#include <ros/ros.h>

#include "LeggedParams.h"

/*
 * State variables naming rule: {part}_{physical quantity}_{frame}_{others}
 * World frame: a global reference frame
 * Body frame: take the center of the torso as the origin, orients in line with the torso
 * Absolute frame: take the center of the torso as the origin, orients in line with the world frame
 * Relative frame: take the center of the torso as the origin, the orientation considers only Yaw of the torso
 * COM frame: take the COM of torso as the origin, orients in line with the torso
 */

namespace legged {
    class LeggedFeedback {
    public:
        LeggedFeedback() { reset(); }

        void reset();

        // Torso position
        Eigen::Vector3d torso_pos_world; // from state estimator

        // Torso attitude
        // Definition of attitude: rotation from body frame to world frame, v_w = R * v_b
        Eigen::Quaterniond torso_quat; // from torso IMU
        Eigen::Vector3d torso_euler;
        Eigen::Matrix3d torso_rot_mat;
        Eigen::Matrix3d torso_rot_mat_z; // rotation matrix considering only Yaw of the torso

        // Torso linear velocity & acceleration
        Eigen::Vector3d torso_lin_vel_world; // from state estimator
        Eigen::Vector3d torso_lin_vel_rel;
        Eigen::Vector3d torso_lin_vel_body;
        Eigen::Vector3d torso_lin_acc_body; // from IMU

        // Torso angular velocity & acceleration
        Eigen::Vector3d torso_ang_vel_world; // from state estimator
        Eigen::Vector3d torso_ang_vel_body; // from IMU
        
        // Motor state
        Eigen::Matrix<double, NUM_DOF, 1> joint_pos;
        Eigen::Matrix<double, NUM_DOF, 1> joint_vel;
        Eigen::Matrix<double, NUM_DOF, 1> joint_tau;

        // Foot tip force & contact
        Eigen::Vector4d foot_force; // from foot pressure sensors
        Eigen::Matrix<double, 3, NUM_LEG> foot_force_est; // estimated foot tip force
        bool record_foot_force_bias = false;
        Eigen::Vector4d foot_sensor_bias;
        Eigen::Vector4d foot_contact_flag;
        bool estimated_contact[NUM_LEG]; // from state estimator

        // Foot position
        Eigen::Matrix<double, LEG_DOF, NUM_LEG> foot_pos_body; // from forward kinematics
        Eigen::Matrix<double, LEG_DOF, NUM_LEG> foot_pos_com;
        Eigen::Matrix<double, LEG_DOF, NUM_LEG> foot_pos_abs;
        Eigen::Matrix<double, LEG_DOF, NUM_LEG> foot_pos_abs_com;
        Eigen::Matrix<double, LEG_DOF, NUM_LEG> foot_pos_abs_prev;
        Eigen::Matrix<double, LEG_DOF, NUM_LEG> foot_pos_world;

        // Foot velocity
        Eigen::Matrix<double, LEG_DOF, NUM_LEG> foot_lin_vel_abs;
        Eigen::Matrix<double, LEG_DOF, NUM_LEG> foot_lin_vel_rel;
        Eigen::Matrix<double, LEG_DOF, NUM_LEG> foot_lin_vel_world;

        // Foot Jacobian matrix
        Eigen::Matrix<double, 3, NUM_LEG * LEG_DOF> jac_foot; // from forward kinematics

        // MPC
        double mpc_time;
    };

    class LeggedCtrl {
    public:
        LeggedCtrl() { reset(); }

        void reset();

        // Gait control
        Eigen::Vector4d gait_counter; // use time directly, value in gait_counter is phase time in gait (0-1)
        Eigen::Vector4d curr_gait_time; // how long in seconds does current phase lasts

        // desired state
        Eigen::Vector3d torso_pos_d_world;
        Eigen::Vector3d torso_pos_d_body;
        Eigen::Vector3d torso_euler_d;
        Eigen::Quaterniond torso_quat_d = Eigen::Quaterniond::Identity();
        Eigen::Vector3d torso_lin_vel_d_body;
        Eigen::Vector3d torso_lin_vel_d_rel;
        Eigen::Vector3d torso_lin_vel_d_world;
        Eigen::Vector3d torso_ang_vel_d_body;
        Eigen::Vector3d torso_ang_vel_d_rel;
        Eigen::Vector3d torso_ang_vel_d_world;

        // target foot position
        Eigen::Matrix<double, 3, NUM_LEG> foot_pos_target_world;
        Eigen::Matrix<double, 3, NUM_LEG> foot_pos_target_abs;
        Eigen::Matrix<double, 3, NUM_LEG> foot_pos_target_rel;
        Eigen::Matrix<double, 3, NUM_LEG> foot_vel_target_rel;

        bool plan_contacts[NUM_LEG]; // flag to decide leg in stance/swing

        // terrain adaptation
        double terrain_pitch_angle; // estimated terrain angle on pitch direction

        // MPC output
        Eigen::Matrix<double, 6 + 3 * NUM_LEG, 1> optimized_state;  // [position, orientation(ZYX Euler angles), foot position]
        Eigen::Matrix<double, 9 * NUM_LEG, 1> optimized_input;  // [3*4 contact force, 3*4 foot vel, 3*4 foot acc]
        Eigen::Matrix<double, 3 * NUM_LEG, 1> mpc_grf_world;

        // joint control
        Eigen::Matrix<double, NUM_DOF, 1> joint_ang_tgt;
        Eigen::Matrix<double, NUM_DOF, 1> joint_vel_tgt;
        Eigen::Matrix<double, NUM_DOF, 1> joint_tau_tgt;

        Eigen::Matrix<double, NUM_DOF, 1> prev_joint_ang_tgt; // used to calculate joint_vel_tgt

        double movement_mode = 0;
    };

    class LeggedJoyCmd {
    public:
        LeggedJoyCmd() {}

        // joystick command
        double velx = 0.0;
        double vely = 0.0;
        double velz = 0.0;

        double pitch_rate = 0.0;
        double roll_rate = 0.0;
        double yaw_rate = 0.0;

        double pitch_ang = 0.0;
        double roll_ang = 0.0;

        double body_x = 0.0;
        double body_y = 0.0;
        double body_height = 0.05;

        // 0 is standing, 1 is walking
        int ctrl_state = 0;
        bool set_default_pos = false;

        bool ctrl_state_change_request = false;
        bool default_pos_request = false;
        bool zero_torque_request = false;
        
        int prev_ctrl_state = 0;
        bool exit = false;
        bool sin_ang_vel = false;
    };

    class LeggedParam {
    public:
        LeggedParam() {}

        bool load(ros::NodeHandle &_nh);

        int env_type; // simulation(0) or hardware(1)
        int robot_type; // a1(0) or go1(1)
        int controller_type; // lci_mpc (0) or convex mpc (1)
        int kf_type;

        double gait_freq; // delta time
        Eigen::Matrix<double, 3, NUM_LEG> default_foot_pos_rel;

        // open terrain adaptation or not
        int terrain_adpt_state;

        // MPC parameters
        double mpc_update_period;
        int mpc_horizon;
        double w;
        Eigen::VectorXd q_weights;
        Eigen::VectorXd r_weights;

        // swing leg parameters
        double kp_hip, kp_thigh, kp_calf;
        double kd_hip, kd_thigh, kd_calf;
        Eigen::Vector3d kp_joint;
        Eigen::Vector3d kd_joint;

        double robot_mass;
        Eigen::Matrix3d trunk_inertia;
        double mu;
        double fz_max;

        // joystick mapping
        int joystick_left_updown_axis;
        int joystick_left_horiz_axis;
        int joystick_right_updown_axis;
        int joystick_right_horiz_axis;
        int joystick_cross_left_right_axis;
        int joystick_cross_up_down_axis;

        int joystick_mode_switch_button;
        int joystick_default_pos_button;
        int joystick_zero_torque_button;
        int sin_ang_vel_button;
        int joystick_exit_button;

        // joystick parameters
        double joystick_velx_scale;
        double joystick_vely_scale;
        double joystick_height_vel;
        double joystick_max_height;
        double joystick_min_height;

        double joystick_yaw_rate_scale;
        double joystick_roll_rate_scale;
        double joystick_pitch_rate_scale;

        // contact detection flags
        double foot_sensor_max_value;
        double foot_sensor_min_value;
        double foot_sensor_ratio;
        Eigen::Vector4d foot_force_contact_threshold;

        // CasADi EKF parameters
        double ekf_inital_cov;
        double ekf_noise_process_pos_xy;
        double ekf_noise_process_pos_z;
        double ekf_noise_process_vel_xy;
        double ekf_noise_process_vel_z;
        double ekf_noise_process_rot;
        double ekf_noise_process_foot;

        double ekf_noise_measure_fk;
        double ekf_noise_measure_vel;
        double ekf_noise_measure_height;

        double ekf_noise_opti_pos;
        double ekf_noise_opti_vel;
        double ekf_noise_opti_roll;
        double ekf_noise_opti_pitch;
        double ekf_noise_opti_yaw;
    };

    class LeggedState {
    public:
        LeggedState() {
            ctrl.reset();
            fbk.reset();
        }

        LeggedCtrl ctrl;
        LeggedFeedback fbk;
        LeggedJoyCmd joy;
        LeggedParam param;

        // put other unclassified variables here
        // Need to be aware of deadlock in thread 1 and thread 2
        bool estimator_init = false;
    };

}  // namespace legged
