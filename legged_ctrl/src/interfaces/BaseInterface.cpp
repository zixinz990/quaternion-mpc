#include "interfaces/BaseInterface.h"

namespace legged {

    BaseInterface::BaseInterface(ros::NodeHandle &_nh) {
        nh = _nh;
        sub_joy_msg = nh.subscribe("/joy", 1000, &BaseInterface::joy_callback, this);
        // low_level_gains_msg = nh.subscribe("/a1_debug/low_level_gains", 100, &BaseInterface::gain_callback, this);

        // Go1 Kinematics
        // leg order: 0-FL  1-FR  2-RL  3-RR
        leg_offset_x[0] = 0.1881;
        leg_offset_x[1] = 0.1881;
        leg_offset_x[2] = -0.1881;
        leg_offset_x[3] = -0.1881;
        leg_offset_y[0] = 0.04675;
        leg_offset_y[1] = -0.04675;
        leg_offset_y[2] = 0.04675;
        leg_offset_y[3] = -0.04675;
        motor_offset[0] = 0.0812;
        motor_offset[1] = -0.0812;
        motor_offset[2] = 0.0812;
        motor_offset[3] = -0.0812;
        upper_leg_length[0] = upper_leg_length[1] = upper_leg_length[2] = upper_leg_length[3] = UPPER_LEG_LENGTH;
        lower_leg_length[0] = lower_leg_length[1] = lower_leg_length[2] = lower_leg_length[3] = LOWER_LEG_LENGTH;

        for (int i = 0; i < NUM_LEG; i++) {
            Eigen::VectorXd rho_fix(5);
            rho_fix << leg_offset_x[i], leg_offset_y[i], motor_offset[i], upper_leg_length[i], lower_leg_length[i];
            Eigen::VectorXd rho_opt(3);
            rho_opt << 0.0, 0.0, 0.0;
            rho_fix_list.push_back(rho_fix);
            rho_opt_list.push_back(rho_opt);
        }

        // load parameter is very important
        legged_state.param.load(_nh);

        // raibert heuristic
        k = 0;
        raibert_delta_rel.setZero();
        raibert_delta_abs.setZero();

        // set filters for terrain adaptation
        for (int i = 0; i < NUM_LEG; i++) {
            recent_contact_pos_x_filter[i] = MovingWindowFilter(1000.0 / MPC_UPDATE_PERIOD * 0.5);
            recent_contact_pos_y_filter[i] = MovingWindowFilter(1000.0 / MPC_UPDATE_PERIOD * 0.5);
            recent_contact_pos_z_filter[i] = MovingWindowFilter(1000.0 / MPC_UPDATE_PERIOD * 0.5);
        }

        // set CasADi EKF
        ekf.set_noise_params(
            legged_state.param.ekf_inital_cov,
            legged_state.param.ekf_noise_process_pos_xy,
            legged_state.param.ekf_noise_process_pos_z,
            legged_state.param.ekf_noise_process_vel_xy,
            legged_state.param.ekf_noise_process_vel_z,
            legged_state.param.ekf_noise_process_rot,
            legged_state.param.ekf_noise_process_foot,
            legged_state.param.ekf_noise_measure_fk,
            legged_state.param.ekf_noise_measure_vel,
            legged_state.param.ekf_noise_measure_height,
            legged_state.param.ekf_noise_opti_pos,
            legged_state.param.ekf_noise_opti_vel,
            // legged_state.param.ekf_noise_opti_roll,
            // legged_state.param.ekf_noise_opti_pitch,
            legged_state.param.ekf_noise_opti_yaw
        );
    }

    void BaseInterface::joy_callback(const sensor_msgs::Joy::ConstPtr &joy_msg) {
        // LB
        if (joy_msg->buttons[legged_state.param.joystick_exit_button] == 1) {
            std::cout << "You have pressed the exit button!" << std::endl;
            legged_state.joy.exit = true;
        }
        // Y
        if (joy_msg->buttons[legged_state.param.sin_ang_vel_button]) legged_state.joy.sin_ang_vel = true;
        else if (joy_msg->buttons[legged_state.param.sin_ang_vel_button] == 0) legged_state.joy.sin_ang_vel = false;
        // A
        if (joy_msg->buttons[legged_state.param.joystick_mode_switch_button] == 1) {
            std::cout << std::endl << "You have requested to change legged_state!" << std::endl << std::endl;
            legged_state.joy.ctrl_state_change_request = true;
        }
        // B
        if (joy_msg->buttons[legged_state.param.joystick_default_pos_button] == 1) {
            legged_state.joy.default_pos_request = true;
        }
        // X
        if (joy_msg->buttons[legged_state.param.joystick_zero_torque_button] == 1) {
            legged_state.joy.zero_torque_request = true;
        }

        // linear velocity x: right updown
        legged_state.joy.velx = joy_msg->axes[legged_state.param.joystick_right_updown_axis] * legged_state.param.joystick_velx_scale;
        // linear velocity y: right horizon
        legged_state.joy.vely = joy_msg->axes[legged_state.param.joystick_right_horiz_axis] * legged_state.param.joystick_vely_scale;
        // linear velocity z: left updown
        legged_state.joy.velz = joy_msg->axes[legged_state.param.joystick_left_updown_axis] * legged_state.param.joystick_height_vel;

        // Euler angular velocity Roll: cross left and right
        switch ((int)joy_msg->axes[legged_state.param.joystick_cross_left_right_axis]) {
            case -1:
                legged_state.joy.roll_rate = legged_state.param.joystick_roll_rate_scale;
                break;
            case 0:
                legged_state.joy.roll_rate = 0.0;
                break;
            case 1:
                legged_state.joy.roll_rate = -legged_state.param.joystick_roll_rate_scale;
                break;
            default:
                legged_state.joy.roll_rate = 0.0;
                break;
        }
        // Euler angular velocity Pitch: cross up and down
        switch ((int)joy_msg->axes[legged_state.param.joystick_cross_up_down_axis]) {
            case -1:
                legged_state.joy.pitch_rate = -legged_state.param.joystick_pitch_rate_scale;
                break;
            case 0:
                legged_state.joy.pitch_rate = 0.0;
                break;
            case 1:
                legged_state.joy.pitch_rate = legged_state.param.joystick_pitch_rate_scale;
                break;
            default:
                legged_state.joy.pitch_rate = 0.0;
                break;
        }
        // Euler angular velocity Yaw: left horizon
        legged_state.joy.yaw_rate = joy_msg->axes[legged_state.param.joystick_left_horiz_axis] * legged_state.param.joystick_yaw_rate_scale;        
    }

    bool BaseInterface::joy_update(double dt) {
        if (legged_state.joy.exit) {
            return false;
        }

        // process joy cmd data to get desired state
        // save the result into legged_state
        Eigen::Vector3d joy_vel_cmd(legged_state.joy.velx, legged_state.joy.vely, legged_state.joy.velz);
        Eigen::Vector3d joy_vel_cmd_rel = legged_state.fbk.torso_rot_mat_z * joy_vel_cmd;

        legged_state.joy.body_x += joy_vel_cmd_rel[0] * dt;
        legged_state.joy.body_y += joy_vel_cmd_rel[1] * dt;
        legged_state.joy.body_height += joy_vel_cmd_rel[2] * dt;

        if (legged_state.joy.body_height >= legged_state.param.joystick_max_height) {
            legged_state.joy.body_height = legged_state.param.joystick_max_height;
        }
        if (legged_state.joy.body_height <= legged_state.param.joystick_min_height) {
            legged_state.joy.body_height = legged_state.param.joystick_min_height;
        }

        legged_state.joy.prev_ctrl_state = legged_state.joy.ctrl_state;

        if (legged_state.joy.ctrl_state_change_request) {
            // toggle legged_state.joy.ctrl_state
            legged_state.joy.ctrl_state = legged_state.joy.ctrl_state + 1;
            legged_state.joy.ctrl_state = legged_state.joy.ctrl_state % 2; //TODO: how to toggle more states?
            legged_state.joy.ctrl_state_change_request = false; //erase this change request;
        }

        if (legged_state.joy.default_pos_request) {
            if (!legged_state.joy.set_default_pos) legged_state.joy.set_default_pos = true;
            else legged_state.joy.set_default_pos = false;

            legged_state.joy.default_pos_request = false;

            legged_state.joy.zero_torque_mode = false;
        }

        if (legged_state.joy.zero_torque_request) {
            if (!legged_state.joy.zero_torque_mode) legged_state.joy.zero_torque_mode = true;
            else legged_state.joy.zero_torque_mode = false;
            
            legged_state.joy.zero_torque_request = false;

            legged_state.joy.set_default_pos = false;
        }

        // update movement mode
        if (legged_state.joy.ctrl_state == 1) {
            // walking mode, in this mode the robot should execute gait
            legged_state.ctrl.movement_mode = 1;
        } else if (legged_state.joy.ctrl_state == 0 && legged_state.joy.prev_ctrl_state == 1) {
            // leave walking mode
            // lock current position, should just happen for one instance
            legged_state.ctrl.movement_mode = 0;
        } else if (legged_state.joy.ctrl_state == 2) {
            legged_state.ctrl.movement_mode = 2;
        } else {
            legged_state.ctrl.movement_mode = 0;
        }

        return true;
    }

    bool BaseInterface::sensor_update(double t, double dt) {
        // calculate several useful variables
        legged_state.fbk.torso_rot_mat = legged_state.fbk.torso_quat.toRotationMatrix();
        legged_state.fbk.torso_euler = Utils::quat_to_euler(legged_state.fbk.torso_quat);
        double yaw_angle = legged_state.fbk.torso_euler[2];
        legged_state.fbk.torso_ang_vel_world = legged_state.fbk.torso_rot_mat * legged_state.fbk.torso_ang_vel_body;
        legged_state.fbk.torso_rot_mat_z = Eigen::AngleAxisd(yaw_angle, Eigen::Vector3d::UnitZ());

        // use old a1 kinematics
        // FL, FR, RL, RR
        for (int i = 0; i < NUM_LEG; ++i) {
            legged_state.fbk.foot_pos_body.block<3, 1>(0, i) = a1_kin.fk(
                    legged_state.fbk.joint_pos.segment<3>(3 * i),
                    rho_opt_list[i], rho_fix_list[i]);
            Eigen::Matrix3d jac = a1_kin.jac(
                    legged_state.fbk.joint_pos.segment<3>(3 * i),
                    rho_opt_list[i], rho_fix_list[i]);

            legged_state.fbk.jac_foot.block<3, 3>(0, 3 * i) = jac;
            Eigen::Matrix3d tmp_mtx = legged_state.fbk.jac_foot.block<3, 3>(0, 3 * i);
            Eigen::Vector3d tmp_vec = legged_state.fbk.joint_vel.segment<3>(3 * i);

            legged_state.fbk.foot_lin_vel_rel.block<3, 1>(0, i) = tmp_mtx * tmp_vec;

            legged_state.fbk.foot_pos_com(0, i) = legged_state.fbk.foot_pos_body(0, i);
            legged_state.fbk.foot_pos_com(1, i) = legged_state.fbk.foot_pos_body(1, i);
            legged_state.fbk.foot_pos_com(2, i) = legged_state.fbk.foot_pos_body(2, i);

            legged_state.fbk.foot_pos_abs.block<3, 1>(0, i) = legged_state.fbk.torso_rot_mat * legged_state.fbk.foot_pos_body.block<3, 1>(0, i);
            legged_state.fbk.foot_pos_abs_com.block<3, 1>(0, i) = legged_state.fbk.torso_rot_mat * legged_state.fbk.foot_pos_com.block<3, 1>(0, i);

            legged_state.fbk.foot_lin_vel_abs.block<3, 1>(0, i) = legged_state.fbk.torso_rot_mat * legged_state.fbk.foot_lin_vel_rel.block<3, 1>(0, i);

            legged_state.fbk.foot_pos_world.block<3, 1>(0, i) = legged_state.fbk.foot_pos_abs.block<3, 1>(0, i) + legged_state.fbk.torso_pos_world;
            legged_state.fbk.foot_lin_vel_world.block<3, 1>(0, i) = legged_state.fbk.foot_lin_vel_abs.block<3, 1>(0, i)
                                                                + legged_state.fbk.torso_lin_vel_world
                                                                + legged_state.fbk.torso_rot_mat * Utils::skew(legged_state.fbk.torso_ang_vel_body) * legged_state.fbk.foot_pos_body.block<3, 1>(0, i);
        }

        // a dynamic model for foot contact thresholding
        for (int i = 0; i < NUM_LEG; ++i) {
            double force_mag = legged_state.fbk.foot_force[i];


            legged_state.param.foot_force_contact_threshold[i] = legged_state.param.foot_sensor_min_value
                                                               + legged_state.param.foot_sensor_ratio * (legged_state.param.foot_sensor_max_value - legged_state.param.foot_sensor_min_value);

            legged_state.fbk.foot_contact_flag[i] = 1.0 / (1 + exp(-10 * (force_mag - legged_state.param.foot_force_contact_threshold[i])));

            if (legged_state.fbk.foot_contact_flag[i] >= 0.5) {
                // record last contact position
                legged_state.fbk.foot_pos_abs_prev.block<3, 1>(0, i) << recent_contact_pos_x_filter[i].CalculateAverage(legged_state.fbk.foot_pos_abs(0, i)),
                        recent_contact_pos_y_filter[i].CalculateAverage(legged_state.fbk.foot_pos_abs(1, i)),
                        recent_contact_pos_z_filter[i].CalculateAverage(legged_state.fbk.foot_pos_abs(2, i));
            }
        }

        // after we got leg jacobian, convert tauEst to foot force estimation
        // tau = J^T * F, so F = J^T^-1 * tau
        for (int i = 0; i < NUM_LEG; ++i) {
            Eigen::Matrix3d jac = legged_state.fbk.jac_foot.block<3, 3>(0, 3 * i).transpose();
            Eigen::Vector3d measure_tau = legged_state.fbk.joint_tau.segment<3>(3 * i);

            // robot frame foot force estimation
            Eigen::Vector3d force_rel = jac.lu().solve(measure_tau);
            // world frame foot force estimation
            legged_state.fbk.foot_force_est.block<3, 1>(0, i) = legged_state.fbk.torso_rot_mat * force_rel;
        }

        estimation_update(t, dt);

        // Raibert heuristic
        legged_state.fbk.torso_lin_vel_rel = legged_state.fbk.torso_rot_mat_z.transpose() * legged_state.fbk.torso_lin_vel_world;
        k = std::sqrt(std::abs(legged_state.fbk.torso_pos_world(2)) / 9.81);

        raibert_delta_rel[0] = k * (legged_state.fbk.torso_lin_vel_rel(0) - legged_state.ctrl.torso_lin_vel_d_rel(0))
                               + (1.0 / legged_state.param.gait_freq) / 2.0 * legged_state.ctrl.torso_lin_vel_d_rel(0);
        if (raibert_delta_rel[0] < -FOOT_DELTA_X_LIMIT) raibert_delta_rel[0] = -FOOT_DELTA_X_LIMIT;
        if (raibert_delta_rel[0] > FOOT_DELTA_X_LIMIT) raibert_delta_rel[0] = FOOT_DELTA_X_LIMIT;

        raibert_delta_rel[1] = k * (legged_state.fbk.torso_lin_vel_rel(1) - legged_state.ctrl.torso_lin_vel_d_rel(1))
                               + (1.0 / legged_state.param.gait_freq) / 2.0 * legged_state.ctrl.torso_lin_vel_d_rel(1);
        if (raibert_delta_rel[1] < -FOOT_DELTA_Y_LIMIT) raibert_delta_rel[1] = -FOOT_DELTA_Y_LIMIT;
        if (raibert_delta_rel[1] > FOOT_DELTA_Y_LIMIT) raibert_delta_rel[1] = FOOT_DELTA_Y_LIMIT;

        raibert_delta_abs = legged_state.fbk.torso_rot_mat_z * raibert_delta_rel;
        legged_state.ctrl.foot_pos_target_abs = legged_state.fbk.torso_rot_mat_z * legged_state.param.default_foot_pos_rel;

        for (int i = 0; i < NUM_LEG; ++i) {
            legged_state.ctrl.foot_pos_target_abs(0, i) += raibert_delta_abs[0];
            legged_state.ctrl.foot_pos_target_abs(1, i) += raibert_delta_abs[1];

            legged_state.ctrl.foot_pos_target_rel.block<3, 1>(0, i) = legged_state.fbk.torso_rot_mat.transpose() * legged_state.ctrl.foot_pos_target_abs.block<3, 1>(0, i);
            legged_state.ctrl.foot_pos_target_world.block<3, 1>(0, i) = legged_state.ctrl.foot_pos_target_abs.block<3, 1>(0, i) + legged_state.fbk.torso_pos_world;
        }

        return true;
    }

    bool BaseInterface::estimation_update(double t, double dt) {
        // legged_state estimation KF
        if (legged_state.param.kf_type == 1) {
            if (!kf.is_inited()) {
                kf.init_state(legged_state);
                legged_state.estimator_init = true;
            } else {
                kf.update_estimation(legged_state, dt);
            }
        } else if (legged_state.param.kf_type == 2) {
            // new CasADi EKF
            Eigen::Matrix<double, NUM_LEG, 1> contacts;
            if (legged_state.ctrl.movement_mode == 0) {
                // stand
                for (int i = 0; i < NUM_LEG; ++i) contacts[i] = 1.0;
            } else {
                // walk
                for (int i = 0; i < NUM_LEG; ++i) {
                    contacts[i] = legged_state.fbk.foot_contact_flag[i];
                }
            }
            ekf_data.input_dt(dt);
            ekf_data.input_imu(legged_state.fbk.torso_lin_acc_body, legged_state.fbk.torso_ang_vel_body);
            ekf_data.input_leg(legged_state.fbk.joint_pos, legged_state.fbk.joint_vel, contacts);

            if (t < 0.1) {
                std::cout << "do not run filter at beginning when ekf_data is not filled! " << std::endl;
                return true;
            }
            if(!ekf.is_inited()) {
                ekf.init_filter(ekf_data);
                legged_state.estimator_init = true;
            } else {
                ekf.update_filter(ekf_data);
                // ekf.update_filter_with_opti(ekf_data);
            }
            // get result
            Eigen::Matrix<double, EKF_STATE_SIZE, 1> kf_state = ekf.get_state();
            // Eigen::Vector3d root_euler_estimation = kf_state.segment<3>(6);
            // Eigen::Quaterniond root_quat_estimation = Utils::euler_to_quat(root_euler_estimation);
            // legged_state.fbk.torso_quat = root_quat_estimation;
            // legged_state.fbk.torso_pos_world = kf_state.segment<3>(0);
            // legged_state.fbk.torso_pos_world.segment<2>(0) = kf_state.segment<2>(0);

            legged_state.fbk.torso_lin_vel_world = kf_state.segment<3>(3);
            // legged_state.fbk.torso_euler = kf_state.segment<3>(6);
        }

        return true;
    }

    bool BaseInterface::tau_ctrl_update(double dt) {
        for (int i = 0; i < NUM_LEG; ++i) {
            Eigen::Matrix3d jac = legged_state.fbk.jac_foot.block<3, 3>(0, 3 * i);
            if (legged_state.ctrl.movement_mode > 0) {
                // Target joint position
                foot_pos_target_body.block<3, 1>(0, i) = legged_state.fbk.torso_rot_mat.transpose() * (legged_state.ctrl.optimized_state.segment<3>(6 + 3 * i) - legged_state.fbk.torso_pos_world);
                Eigen::Vector3d joint_ang_tgt = a1_kin.inv_kin(foot_pos_target_body.block<3, 1>(0, i), legged_state.fbk.joint_pos.segment<3>(i * 3), rho_opt_list[i], rho_fix_list[i]);
                if ((isnan(joint_ang_tgt[0])) || (isnan(joint_ang_tgt[1])) || (isnan(joint_ang_tgt[2]))) {
                    legged_state.ctrl.joint_ang_tgt.segment<3>(i * 3) = legged_state.fbk.joint_pos.segment<3>(i * 3);
                } else {
                    legged_state.ctrl.joint_ang_tgt.segment<3>(i * 3) = joint_ang_tgt;
                }

                // Target joint velocity
                foot_vel_target_body.block<3, 1>(0, i) = legged_state.fbk.torso_rot_mat.transpose() * (legged_state.ctrl.optimized_input.segment<3>(12 + 3 * i) - legged_state.fbk.torso_lin_vel_world);
                Eigen::Vector3d joint_vel_tgt = jac.lu().solve(foot_vel_target_body.block<3, 1>(0, i));
                if ((isnan(joint_vel_tgt[0])) || (isnan(joint_vel_tgt[1])) || (isnan(joint_vel_tgt[2]))) {
                    legged_state.ctrl.joint_vel_tgt.segment<3>(i * 3) = legged_state.fbk.joint_vel.segment<3>(i * 3);
                } else {
                    legged_state.ctrl.joint_vel_tgt.segment<3>(i * 3) = joint_vel_tgt;
                }

                // Target joint torque
                if (legged_state.ctrl.plan_contacts[i]) {
                    legged_state.ctrl.joint_tau_tgt.segment<3>(i * 3) = -jac.transpose() * legged_state.ctrl.optimized_input.segment<3>(i * 3);
                } else {
                    legged_state.ctrl.joint_tau_tgt.segment<3>(i * 3) = Eigen::Vector3d::Zero();
                    // Eigen::Vector3d tau;
                    // foot_acc_target_body.block<3, 1>(0, i) = legged_state.fbk.torso_rot_mat.transpose()
                    //                                          * legged_state.ctrl.optimized_input.segment<3>(24 + 3 * i);
                    // if (i == 0) {
                    //     go1_dyn.go1_FL_inverse_dynamics_task_space(joint_ang_tgt,
                    //                                                foot_vel_target_body.block<3, 1>(0, i), foot_acc_target_body.block<3, 1>(0, i),
                    //                                                tau);
                    // } else if (i == 1) {
                    //     go1_dyn.go1_FR_inverse_dynamics_task_space(joint_ang_tgt,
                    //                                                foot_vel_target_body.block<3, 1>(0, i), foot_acc_target_body.block<3, 1>(0, i),
                    //                                                tau);
                    // } else if (i == 2) {
                    //     go1_dyn.go1_RL_inverse_dynamics_task_space(joint_ang_tgt,
                    //                                                foot_vel_target_body.block<3, 1>(0, i), foot_acc_target_body.block<3, 1>(0, i),
                    //                                                tau);
                    // } else if (i == 3) {
                    //     go1_dyn.go1_RR_inverse_dynamics_task_space(joint_ang_tgt,
                    //                                                foot_vel_target_body.block<3, 1>(0, i), foot_acc_target_body.block<3, 1>(0, i),
                    //                                                tau);
                    // }
                    // legged_state.ctrl.joint_tau_tgt.segment<3>(i * 3) = tau + 0.2 * legged_state.ctrl.joint_vel_tgt.segment<3>(i * 3);
                }

                // Print sth
                // std::cout << "i: " << i << std::endl;
                // std::cout << "foot_contact_flag: " << legged_state.fbk.foot_contact_flag[i] << std::endl;
                // std::cout << "plan_contacts: " << legged_state.ctrl.plan_contacts[i] << std::endl;
                // std::cout << "joint_tau_tgt: " << legged_state.ctrl.joint_tau_tgt.segment<3>(i * 3) << std::endl;
                // std::cout << "GRF plan: " << legged_state.ctrl.optimized_input.segment<3>(i * 3) << std::endl;
            } else {
                legged_state.ctrl.joint_tau_tgt.segment<3>(i * 3) = -jac.transpose() * legged_state.ctrl.optimized_input.segment<3>(i * 3);
                legged_state.ctrl.joint_ang_tgt.segment<3>(i * 3) = legged_state.fbk.joint_pos.segment<3>(i * 3);
                legged_state.ctrl.joint_vel_tgt.segment<3>(i * 3) = legged_state.fbk.joint_vel.segment<3>(i * 3);
            }
        }

        return true;
    }
}  // namespace legged
