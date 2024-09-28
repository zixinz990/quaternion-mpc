#include "mpc/QuatMpc.h"


using namespace altro;

namespace legged {

    QuatMpc::QuatMpc(LeggedState &state) {
        for (int i = 0; i < 3; i++) {
            torso_lin_vel_d_body_filter[i] = MovingWindowFilter(100);
            torso_pos_d_body_filter[i] = MovingWindowFilter(100);
        }
        // Initialize desired torso position using current torso position
        state.ctrl.torso_pos_d_world = state.fbk.torso_pos_world;
        if (state.ctrl.torso_pos_d_world.norm() < 0.001) {
            torso_pos_d_world_init = false;
        } else {
            torso_pos_d_world_init = true;
        }

        opts.verbose = Verbosity::Silent;
        opts.iterations_max = 10;
        opts.use_backtracking_linesearch = true;
        opts.use_quaternion = true;
        opts.quat_start_index = 3;
        opts.penalty_scaling = 20.0;
        
        n = 13;
        m = 12;

        x_ref.resize(n);
        u_ref.resize(m);

        h = state.param.mpc_update_period;
        horizon = state.param.mpc_horizon;

        mu = state.param.mu;
        fz_max = state.param.fz_max;

        for (int i = 0; i < NUM_LEG; i++) {
            leg_FSM[i].reset_params(state, i);
        }
        attitude_traj_count = 0;

        model_ptr = std::make_shared<QuadrupedModel>();

        C_mat << 1, 0, -mu,   //  fx - mu * fz <= 0
                 -1, 0, -mu,  // -fx - mu * fz <= 0
                 0, 1, -mu,   //  fy - mu * fz <= 0
                 0, -1, -mu,  // -fy - mu * fz <= 0
                 0, 0, 1,     //  fz - fz_max  <= 0
                 0, 0, -1;    // -fz           <= 0
        
        x_init.resize(n);
    }

    bool QuatMpc::update(LeggedState &state) {
        goal_update(state);
        foot_update(state);
        grf_update(state);

        // state.ctrl.optimized_state.segment<3>(0) = state.ctrl.torso_pos_d_world;
        // state.ctrl.optimized_state.segment<3>(3) = state.ctrl.torso_euler_d;

        return true;
    }

    bool QuatMpc::goal_update(LeggedState &state) {
        if (state.estimator_init == false) {
            std::cout << "estimation not inited" << std::endl;
            return true;
        }

        if (torso_pos_d_world_init == false) {
            state.ctrl.torso_pos_d_world = state.fbk.torso_pos_world;
            torso_pos_d_world_init = true;
        }

        // Update linear velocity goal
        state.ctrl.torso_lin_vel_d_rel[0] = state.joy.velx;
        state.ctrl.torso_lin_vel_d_rel[1] = state.joy.vely;
        state.ctrl.torso_lin_vel_d_rel[2] = 0.0;

        state.ctrl.torso_lin_vel_d_world = state.fbk.torso_rot_mat_z * state.ctrl.torso_lin_vel_d_rel;
        state.ctrl.torso_lin_vel_d_body = state.fbk.torso_rot_mat.transpose() * state.ctrl.torso_lin_vel_d_world;
        torso_lin_vel_d_body_filtered.setZero();
        torso_lin_vel_d_body_filtered << torso_lin_vel_d_body_filter[0].CalculateAverage(state.ctrl.torso_lin_vel_d_body[0]),
                                         torso_lin_vel_d_body_filter[1].CalculateAverage(state.ctrl.torso_lin_vel_d_body[1]),
                                         torso_lin_vel_d_body_filter[2].CalculateAverage(state.ctrl.torso_lin_vel_d_body[2]);

        // Update angular velocity goal
        state.ctrl.torso_ang_vel_d_body[0] = state.joy.roll_rate;
        state.ctrl.torso_ang_vel_d_body[1] = state.joy.pitch_rate;
        state.ctrl.torso_ang_vel_d_body[2] = state.joy.yaw_rate;

        // Update desired torso position
        state.ctrl.torso_pos_d_world[0] += state.ctrl.torso_lin_vel_d_world[0] * 5.0 / 1000.0;
        state.ctrl.torso_pos_d_world[1] += state.ctrl.torso_lin_vel_d_world[1] * 5.0 / 1000.0;
        state.ctrl.torso_pos_d_world[2] = state.joy.body_height;

        state.ctrl.torso_pos_d_body = state.fbk.torso_rot_mat.transpose() * (state.ctrl.torso_pos_d_world - state.fbk.torso_pos_world);
        torso_pos_d_body_filtered.setZero();
        torso_pos_d_body_filtered << torso_pos_d_body_filter[0].CalculateAverage(state.ctrl.torso_pos_d_body[0]),
                                     torso_pos_d_body_filter[1].CalculateAverage(state.ctrl.torso_pos_d_body[1]),
                                     torso_pos_d_body_filter[2].CalculateAverage(state.ctrl.torso_pos_d_body[2]);        
        return true;
    }

    bool QuatMpc::grf_update(LeggedState &state) {
        auto t_start = std::chrono::high_resolution_clock::now();        

        /// REFERENCES ///
        // Clear vectors
        x_traj_ref.clear();
        u_traj_ref.clear();

        // Update reference of control input
        num_contacts = 0;
        for (int i = 0; i < NUM_LEG; i++) if (state.ctrl.plan_contacts[i]) num_contacts++;

        u_ref.setZero();
        u_ref << 0.0, 0.0, state.ctrl.plan_contacts[0] * state.param.robot_mass * 9.81 / num_contacts,
                 0.0, 0.0, state.ctrl.plan_contacts[1] * state.param.robot_mass * 9.81 / num_contacts,
                 0.0, 0.0, state.ctrl.plan_contacts[2] * state.param.robot_mass * 9.81 / num_contacts,
                 0.0, 0.0, state.ctrl.plan_contacts[3] * state.param.robot_mass * 9.81 / num_contacts;

        // Update torso_quat_d
        Eigen::Vector4d torso_quat_d_vec(state.ctrl.torso_quat_d.w(),
                                         state.ctrl.torso_quat_d.x(),
                                         state.ctrl.torso_quat_d.y(),
                                         state.ctrl.torso_quat_d.z());
        torso_quat_d_vec += 0.5 * QuaternionUtils::G(torso_quat_d_vec) * state.ctrl.torso_ang_vel_d_body * 5.0 / 1000.0;
        torso_quat_d_vec = torso_quat_d_vec / torso_quat_d_vec.norm();
        state.ctrl.torso_quat_d.w() = torso_quat_d_vec[0];
        state.ctrl.torso_quat_d.x() = torso_quat_d_vec[1];
        state.ctrl.torso_quat_d.y() = torso_quat_d_vec[2];
        state.ctrl.torso_quat_d.z() = torso_quat_d_vec[3];
        
        // For sin ang vel test
        if (state.joy.sin_ang_vel) {
            state.ctrl.torso_euler_d[0] = 3.14 / 8 * sin(2 * 3.14 / 900 * attitude_traj_count);
            state.ctrl.torso_euler_d[1] = 3.14 / 8 * sin(2 * 3.14 / 900 * attitude_traj_count);
            state.ctrl.torso_euler_d[2] = 3.14 / 8 * sin(2 * 3.14 / 900 * attitude_traj_count);
            attitude_traj_count += 1;
            state.ctrl.torso_quat_d = Utils::euler_to_quat(state.ctrl.torso_euler_d);
        }        

        for (int i = 0; i <= horizon; i++) {
            x_ref.setZero();
            
            // Position
            // x_ref[0] = state.ctrl.torso_pos_d_body[0] + state.ctrl.torso_lin_vel_d_body[0] * i * h / 1000.0;
            // x_ref[1] = state.ctrl.torso_pos_d_body[1] + state.ctrl.torso_lin_vel_d_body[1] * i * h / 1000.0;
            // x_ref[2] = state.ctrl.torso_pos_d_body[2];

            x_ref[0] = torso_pos_d_body_filtered[0] + torso_lin_vel_d_body_filtered[0] * i * h / 1000.0;
            x_ref[1] = torso_pos_d_body_filtered[1] + torso_lin_vel_d_body_filtered[1] * i * h / 1000.0;
            x_ref[2] = torso_pos_d_body_filtered[2];

            // Orientation
            // x_ref.segment<4>(3) = torso_quat_d_vec;
            x_ref[3] = state.ctrl.torso_quat_d.w();
            x_ref[4] = state.ctrl.torso_quat_d.x();
            x_ref[5] = state.ctrl.torso_quat_d.y();
            x_ref[6] = state.ctrl.torso_quat_d.z();

            // Linear velocity
            // x_ref.segment<3>(7) = state.ctrl.torso_lin_vel_d_body;
            x_ref.segment<3>(7) = torso_lin_vel_d_body_filtered;

            // Angular velocity
            // x_ref.tail(3) = state.ctrl.torso_ang_vel_d_body;
            
            x_traj_ref.emplace_back(x_ref);
            u_traj_ref.emplace_back(u_ref);
        }
        
        /// DYNAMICS ///
        // Robot inertia
        double torso_mass = 5.204;
        // Eigen::Matrix<double, 3, 3> inertia = (state.param.robot_mass / torso_mass) * state.param.trunk_inertia;
        Eigen::Matrix<double, 3, 3> inertia = 1.2 * state.param.trunk_inertia;

        ct_dyn = [&](double *x_dot, const double *x, const double *u) {
            model_ptr->ct_srb_quat_dynamics(x_dot, x, u, state.fbk.foot_pos_body, inertia, state.param.robot_mass, state.fbk.torso_rot_mat);
        };
        ct_jac = [&](double *jac, const double *x, const double *u) {
            model_ptr->ct_srb_quat_jacobian(jac, x, u, state.fbk.foot_pos_body, inertia, state.param.robot_mass);
        };
        dt_dyn = midpoint_dynamics(n, m, ct_dyn);
        dt_jac = midpoint_jacobian(n, m, ct_dyn, ct_jac);

        /// CONSTRAINTS ///
        friction_cone_con = [&](a_float *c, const a_float *x, const a_float *u) {
            (void)x;
            Eigen::Map<Eigen::Matrix<a_float, 24, 1>> C(c);
            Eigen::Map<const Eigen::VectorXd> u_vec(u, 12);
            
            C.setZero();
            for (int i = 0; i < 4; i++) {
                Eigen::Matrix<a_float, 6, 1> b_vec;
                b_vec << 0, 0, 0, 0, -fz_max * state.ctrl.plan_contacts[i], 0;
                C.segment<6>(i * 6) = C_mat * state.fbk.torso_rot_mat * u_vec.segment<3>(i * 3) + b_vec;
            }
        };

        friction_cone_jac = [&](a_float *jac, const a_float *x, const a_float *u) {
            (void)x;
            (void)u;
            Eigen::Map<Eigen::Matrix<a_float, 24, 24>> J(jac);
            J.setZero();
            for (int i = 0; i < 4; i++) {
                J.block<6, 3>(i * 6, 12 + i * 3) = C_mat * state.fbk.torso_rot_mat;
            }
        };

        /// SETUP ///
        ALTROSolver solver(horizon);

        solver.SetOptions(opts);

        solver.SetDimension(n, m);
        solver.SetExplicitDynamics(dt_dyn, dt_jac);
        solver.SetTimeStep(h / 1000.0);

        for (int i = 0; i <= horizon; i++) {
            solver.SetQuaternionCost(n, m, state.param.q_weights.data(), state.param.r_weights.data(), state.param.w, x_traj_ref.at(i).data(), u_traj_ref.at(i).data(), i, 0);
        }
        solver.SetConstraint(friction_cone_con, friction_cone_jac, 24, ConstraintType::INEQUALITY, "friction cone", 0, horizon);

        state.fbk.torso_lin_vel_body = state.fbk.torso_rot_mat.transpose() * state.fbk.torso_lin_vel_world;
        x_init.setZero();
        x_init << 0.0,
                  0.0,
                  0.0,
                  state.fbk.torso_quat.w(),
                  state.fbk.torso_quat.x(),
                  state.fbk.torso_quat.y(),
                  state.fbk.torso_quat.z(),
                  state.fbk.torso_lin_vel_body[0],
                  state.fbk.torso_lin_vel_body[1],
                  state.fbk.torso_lin_vel_body[2];
                  state.fbk.torso_ang_vel_body[0],
                  state.fbk.torso_ang_vel_body[1],
                  state.fbk.torso_ang_vel_body[2];
        solver.SetInitialState(x_init.data(), n);
        solver.Initialize();

        // Initial guesses
        for (int i = 0; i <= horizon; i++) {
            solver.SetState(x_traj_ref.at(i).data(), n, i);
        }
        solver.SetInput(u_traj_ref.at(0).data(), m);

        /// SOLVE ///
        solver.Solve();
        auto t_end = std::chrono::high_resolution_clock::now();
        using SecondsDouble = std::chrono::duration<double, std::ratio<1>>;
        SecondsDouble t_total = std::chrono::duration_cast<SecondsDouble>(t_end - t_start);
        // fmt::print("Total time = {} ms\n", t_total * 1000);
        state.fbk.mpc_time = t_total.count() * 1000;

        /// SAVE ///
        Eigen::VectorXd u(m);
        solver.GetInput(u.data(), 0);

        for (int i = 0; i < NUM_LEG; i++) {
            state.ctrl.mpc_grf_world.segment<3>(i * 3) = state.fbk.torso_rot_mat * u.segment<3>(i * 3);
            state.ctrl.optimized_input.segment<3>(3 * i) = u.segment<3>(i * 3);
            state.ctrl.optimized_state.segment<3>(6 + 3 * i) = leg_FSM[i].FSM_foot_pos_target_world;
            state.ctrl.optimized_input.segment<3>(12 + 3 * i) = leg_FSM[i].FSM_foot_vel_target_world;
            state.ctrl.optimized_input.segment<3>(24 + 3 * i) = leg_FSM[i].FSM_foot_acc_target_world;
        }

        return true;
    }

    bool QuatMpc::foot_update(LeggedState &state) {
        // in low level loop, a raibert strategy generates foot_pos_target_world
        // stored in state.ctrl.foot_pos_target_world

        // reset LegFSM in movement_mode 0
        if (state.ctrl.movement_mode == 0) {
            for (int i = 0; i < NUM_LEG; i++) {
                leg_FSM[i].reset();
                // movement_mode 0: only do torque control so foot positions are not important
                // we only reset leg FSM for safety
                state.ctrl.plan_contacts[i] = true;
            }
        } else {
            for (int i = 0; i < NUM_LEG; i++) {
                state.ctrl.gait_counter[i] = leg_FSM[i].update(5.0 / 1000.0, state.param.gait_freq,
                                                               state.fbk.foot_pos_world.block<3, 1>(0, i),
                                                               state.ctrl.foot_pos_target_world.block<3, 1>(0, i),
                                                               state.fbk.foot_contact_flag[i]);

                // TODO: gait phase of each leg is individually controlled, so we may need to occasionally synchrinize them, for example if all leg are in contact, average their gait phase and set them to that value
            }
            for (int i = 0; i < NUM_LEG; i++) {
                state.ctrl.plan_contacts[i] = leg_FSM[i].get_contact_state();
            }
        }

        return true;
    }

    bool QuatMpc::terrain_update(LeggedState &state) {
        // Eigen::Vector3d walk_surf_coef = Utils::get_walk_surf_coef(state.fbk.foot_pos_abs_prev);
        // Eigen::Vector3d flat_ground_coef;
        // flat_ground_coef << 0, 0, 1;

        // double terrain_angle = 0.0;

        // // only do terrain adaption in walking mode && body height > 0.1 m
        // if (state.ctrl.movement_mode > 0 && state.fbk.torso_pos_world[2] > 0.1) {
        //     // terrain_angle = Utils::cal_dihedral_angle(flat_ground_coef, walk_surf_coef);
        //     terrain_angle = terrain_angle_filter.CalculateAverage(Utils::cal_dihedral_angle(flat_ground_coef, walk_surf_coef));

        //     // prevent excessive angle
        //     if (terrain_angle > 0.349) {
        //         terrain_angle = 0.349;
        //     }
        //     if (terrain_angle < -0.349) {
        //         terrain_angle = -0.349;
        //     }

        //     // get desired pitch angle
        //     double F_R_diff = state.fbk.foot_pos_abs_prev(2, 0) + state.fbk.foot_pos_abs_prev(2, 1) - state.fbk.foot_pos_abs_prev(2, 2) - state.fbk.foot_pos_abs_prev(2, 3); // FL, FR, RL, RR
        //     if (F_R_diff > 0.05) {
        //         state.ctrl.torso_euler_d[1] = -terrain_angle;
        //     } else {
        //         state.ctrl.torso_euler_d[1] = terrain_angle;
        //     }
        // }

        return true;
    }

}  // namespace legged
