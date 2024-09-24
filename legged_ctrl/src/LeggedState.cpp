/*
 *  Important shared variables between the robot interface and the cotroller 
 */

#include <Eigen/Dense>
#include <ros/ros.h>

#include "LeggedParams.h"
#include "LeggedState.h"

namespace legged {
    void LeggedFeedback::reset() {
        // Torso position
        torso_pos_world.setZero(); // from state estimator

        // Torso attitude
        // Definition of attitude: rotation from body frame to world frame, v_w = R * v_b
        torso_quat.setIdentity(); // from torso IMU
        torso_euler.setZero();
        torso_rot_mat.setZero();
        torso_rot_mat_z.setZero(); // rotation matrix considering only Yaw of the torso

        // Torso linear velocity & acceleration
        torso_lin_vel_world.setZero(); // from state estimator
        torso_lin_vel_rel.setZero();
        torso_lin_vel_body.setZero();
        torso_lin_acc_body.setZero(); // from IMU

        // Torso angular velocity & acceleration
        torso_ang_vel_world.setZero(); // from state estimator
        torso_ang_vel_body.setZero(); // from IMU
        
        // Motor state
        joint_pos.setZero();
        joint_vel.setZero();
        joint_tau.setZero();

        // Foot tip force & contact
        foot_force.setZero(); // from foot pressure sensors
        foot_force_est.setZero(); // estimated foot tip force
        record_foot_force_bias = false;
        foot_sensor_bias.setZero();
        foot_contact_flag.setZero();
        std::fill(estimated_contact, estimated_contact + NUM_LEG, true); // from state estimator

        // Foot position
        foot_pos_body.setZero(); // from forward kinematics
        foot_pos_com.setZero();
        foot_pos_abs.setZero();
        foot_pos_abs_com.setZero();
        foot_pos_abs_prev.setZero();
        foot_pos_world.setZero();

        // Foot velocity
        foot_lin_vel_abs.setZero();
        foot_lin_vel_rel.setZero();
        foot_lin_vel_world.setZero();

        // Foot Jacobian matrix
        jac_foot.setZero(); // from forward kinematics
    }

    void LeggedCtrl::reset() {
    }

    bool LeggedParam::load(ros::NodeHandle &_nh) {
        // critical parameters, if not found, return false
        if (!_nh.getParam("/env_type", env_type)) return false;
        if (!_nh.getParam("/robot_type", robot_type)) return false;
        if (!_nh.getParam("/controller_type", controller_type)) return false;
        if (!_nh.getParam("/kf_type", kf_type)) return false;

        // terrain adaptation
        _nh.param("terrain_adpt_state", terrain_adpt_state, 0);

        // params that are not critical, have default values
        _nh.param("/gait_freq", gait_freq, 3.0);

        double default_foot_pos_FL_x;
        double default_foot_pos_FL_y;
        double default_foot_pos_FL_z;

        double default_foot_pos_FR_x;
        double default_foot_pos_FR_y;
        double default_foot_pos_FR_z;

        double default_foot_pos_RL_x;
        double default_foot_pos_RL_y;
        double default_foot_pos_RL_z;

        double default_foot_pos_RR_x;
        double default_foot_pos_RR_y;
        double default_foot_pos_RR_z;

        _nh.param("default_foot_pos_FL_x", default_foot_pos_FL_x, 0.25);
        _nh.param("default_foot_pos_FL_y", default_foot_pos_FL_y, 0.15);
        _nh.param("default_foot_pos_FL_z", default_foot_pos_FL_z, -0.33);

        _nh.param("default_foot_pos_FR_x", default_foot_pos_FR_x, 0.25);
        _nh.param("default_foot_pos_FR_y", default_foot_pos_FR_y, -0.15);
        _nh.param("default_foot_pos_FR_z", default_foot_pos_FR_z, -0.33);

        _nh.param("default_foot_pos_RL_x", default_foot_pos_RL_x, -0.17);
        _nh.param("default_foot_pos_RL_y", default_foot_pos_RL_y, 0.15);
        _nh.param("default_foot_pos_RL_z", default_foot_pos_RL_z, -0.33);

        _nh.param("default_foot_pos_RR_x", default_foot_pos_RR_x, -0.17);
        _nh.param("default_foot_pos_RR_y", default_foot_pos_RR_y, -0.15);
        _nh.param("default_foot_pos_RR_z", default_foot_pos_RR_z, -0.33);

        default_foot_pos_rel << default_foot_pos_FL_x, default_foot_pos_FR_x, default_foot_pos_RL_x, default_foot_pos_RR_x,
                default_foot_pos_FL_y, default_foot_pos_FR_y, default_foot_pos_RL_y, default_foot_pos_RR_y,
                default_foot_pos_FL_z, default_foot_pos_FR_z, default_foot_pos_RL_z, default_foot_pos_RR_z;
        
        _nh.param("mpc_update_period", mpc_update_period, 10.0);
        _nh.param("mpc_horizon", mpc_horizon, 30);

        double q_weights_0, q_weights_1, q_weights_2, q_weights_3, q_weights_4, q_weights_5, q_weights_6, q_weights_7, q_weights_8, q_weights_9, q_weights_10, q_weights_11, q_weights_12;

        _nh.param("q_weights_0", q_weights_0, 0.0);
        _nh.param("q_weights_1", q_weights_1, 0.0);
        _nh.param("q_weights_2", q_weights_2, 50.0);

        _nh.param("q_weights_3", q_weights_3, 0.0);
        _nh.param("q_weights_4", q_weights_4, 0.0);
        _nh.param("q_weights_5", q_weights_5, 0.0);
        _nh.param("q_weights_6", q_weights_6, 0.0);

        _nh.param("q_weights_7", q_weights_7, 1.0);
        _nh.param("q_weights_8", q_weights_8, 1.0);
        _nh.param("q_weights_9", q_weights_9, 1.0);

        _nh.param("q_weights_10", q_weights_10, 0.5);
        _nh.param("q_weights_11", q_weights_11, 0.5);
        _nh.param("q_weights_12", q_weights_12, 0.5);
        
        if (controller_type == 2) {
                _nh.param("w", w, 50.0);
                q_weights.resize(13);
                q_weights << q_weights_0, q_weights_1, q_weights_2,
                             q_weights_3, q_weights_4, q_weights_5, q_weights_6,
                             q_weights_7, q_weights_8, q_weights_9,
                             q_weights_10, q_weights_11, q_weights_12;
        } else {
                q_weights.resize(12);
                q_weights << q_weights_0, q_weights_1, q_weights_2,
                             q_weights_3, q_weights_4, q_weights_5,
                             q_weights_6, q_weights_7, q_weights_8,
                             q_weights_9, q_weights_10, q_weights_11;
        }

        r_weights.resize(12);
        double r_weights_0, r_weights_1, r_weights_2, r_weights_3, r_weights_4, r_weights_5, r_weights_6, r_weights_7, r_weights_8, r_weights_9, r_weights_10, r_weights_11, r_weights_rw_1, r_weights_rw_2;

        _nh.param("r_weights_0", r_weights_0, 1e-5);
        _nh.param("r_weights_1", r_weights_1, 1e-5);
        _nh.param("r_weights_2", r_weights_2, 1e-6);

        _nh.param("r_weights_3", r_weights_3, 1e-5);
        _nh.param("r_weights_4", r_weights_4, 1e-5);
        _nh.param("r_weights_5", r_weights_5, 1e-6);

        _nh.param("r_weights_6", r_weights_6, 1e-5);
        _nh.param("r_weights_7", r_weights_7, 1e-5);
        _nh.param("r_weights_8", r_weights_8, 1e-6);

        _nh.param("r_weights_9", r_weights_9, 1e-5);
        _nh.param("r_weights_10", r_weights_10, 1e-5);
        _nh.param("r_weights_11", r_weights_11, 1e-6);

        r_weights << r_weights_0, r_weights_1, r_weights_2,
                r_weights_3, r_weights_4, r_weights_5,
                r_weights_6, r_weights_7, r_weights_8,
                r_weights_9, r_weights_10, r_weights_11;

        _nh.param("kp_hip", kp_hip, 100.0);
        _nh.param("kp_thigh", kp_thigh, 100.0);
        _nh.param("kp_calf", kp_calf, 100.0);

        _nh.param("kd_hip", kd_hip, 0.0);
        _nh.param("kd_thigh", kd_thigh, 0.0);
        _nh.param("kd_calf", kd_calf, 0.0);

        kp_joint << kp_hip, kp_thigh, kp_calf;
        kd_joint << kd_hip, kd_thigh, kd_calf;
        
        _nh.param("robot_mass", robot_mass, 13.0);

        double trunk_inertia_xx;
        double trunk_inertia_xy;
        double trunk_inertia_xz;
        double trunk_inertia_yz;
        double trunk_inertia_yy;
        double trunk_inertia_zz;

        _nh.param("trunk_inertia_xx", trunk_inertia_xx, 0.0158533);
        _nh.param("trunk_inertia_xy", trunk_inertia_xy, 0.0);
        _nh.param("trunk_inertia_xz", trunk_inertia_xz, 0.0);
        _nh.param("trunk_inertia_yz", trunk_inertia_yz, 0.0);
        _nh.param("trunk_inertia_yy", trunk_inertia_yy, 0.0377999);
        _nh.param("trunk_inertia_zz", trunk_inertia_zz, 0.0456542);

        trunk_inertia << trunk_inertia_xx, trunk_inertia_xy, trunk_inertia_xz,
                trunk_inertia_xy, trunk_inertia_yy, trunk_inertia_yz,
                trunk_inertia_xz, trunk_inertia_yz, trunk_inertia_zz;

        _nh.param("mu", mu, 0.7);
        _nh.param("fz_max", fz_max, 700.0);

        // joystick mapping
        _nh.param("/joystick_left_updown_axis", joystick_left_updown_axis, 1);
        _nh.param("/joystick_left_horiz_axis", joystick_left_horiz_axis, 0);
        _nh.param("/joystick_right_updown_axis", joystick_right_updown_axis, 4);
        _nh.param("/joystick_right_horiz_axis", joystick_right_horiz_axis, 3);
        _nh.param("/joystick_cross_left_right_axis", joystick_cross_left_right_axis, 6);
        _nh.param("/joystick_cross_up_down_axis", joystick_cross_up_down_axis, 7);

        _nh.param("/joystick_mode_switch_button", joystick_mode_switch_button, 1);
        _nh.param("/joystick_default_pos_button", joystick_default_pos_button, 2);
        _nh.param("/joystick_zero_torque_button", joystick_zero_torque_button, 0);
        _nh.param("/sin_ang_vel_button", sin_ang_vel_button, 3);
        _nh.param("/joystick_exit_button", joystick_exit_button, 4);

        // joystick parameters
        _nh.param("/joystick_velx_scale", joystick_velx_scale, 2.5);
        _nh.param("/joystick_vely_scale", joystick_vely_scale, 0.4);
        _nh.param("/joystick_height_vel", joystick_height_vel, 0.1);
        _nh.param("/joystick_max_height", joystick_max_height, 0.3);
        _nh.param("/joystick_min_height", joystick_min_height, 0.03);
        _nh.param("/joystick_yaw_rate_scale", joystick_yaw_rate_scale, 0.8);
        _nh.param("/joystick_roll_rate_scale", joystick_roll_rate_scale, 0.4);
        _nh.param("/joystick_pitch_rate_scale", joystick_pitch_rate_scale, 0.4);

        // contact detection flags
        _nh.param("/foot_sensor_max_value", foot_sensor_max_value, 300.0);
        _nh.param("/foot_sensor_min_value", foot_sensor_min_value, 0.0);
        _nh.param("/foot_sensor_ratio", foot_sensor_ratio, 0.5);

        // CasADi EKF parameters
        _nh.param("/ekf_inital_cov", ekf_inital_cov, 0.001);
        _nh.param("/ekf_noise_process_pos_xy", ekf_noise_process_pos_xy, 0.001);
        _nh.param("/ekf_noise_process_pos_z", ekf_noise_process_pos_z, 0.001);
        _nh.param("/ekf_noise_process_vel_xy", ekf_noise_process_vel_xy, 0.001);
        _nh.param("/ekf_noise_process_vel_z", ekf_noise_process_vel_z, 0.01);
        _nh.param("/ekf_noise_process_rot", ekf_noise_process_rot, 1e-6);
        _nh.param("/ekf_noise_process_foot", ekf_noise_process_foot, 0.001);

        _nh.param("/ekf_noise_measure_fk", ekf_noise_measure_fk, 0.01);
        _nh.param("/ekf_noise_measure_vel", ekf_noise_measure_vel, 0.01);
        _nh.param("/ekf_noise_measure_height", ekf_noise_measure_height, 0.0001);
        
        _nh.param("/ekf_noise_opti_pos", ekf_noise_opti_pos, 0.001);
        _nh.param("/ekf_noise_opti_vel", ekf_noise_opti_vel, 999.0);
        _nh.param("/ekf_noise_opti_roll", ekf_noise_opti_roll, 0.001);
        _nh.param("/ekf_noise_opti_pitch", ekf_noise_opti_pitch, 0.001);
        _nh.param("/ekf_noise_opti_yaw", ekf_noise_opti_yaw, 0.01);

        return true;
    }

}  // namespace legged
