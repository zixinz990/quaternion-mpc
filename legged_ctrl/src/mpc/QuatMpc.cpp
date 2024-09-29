#include "mpc/QuatMpc.h"


using namespace altro;

namespace legged {

    QuatMpc::QuatMpc(LeggedState &state) {
        opts.verbose = Verbosity::Silent;
        opts.iterations_max = 20;
        opts.use_backtracking_linesearch = true;
        opts.use_quaternion = true;
        opts.quat_start_index = 3;
        
        n = 13;
        en = 12;
        m = 12;
        em = 12;
        h = state.param.mpc_update_period;
        horizon = state.param.mpc_horizon;

        mu = state.param.mu;
        fz_min = 0.0;
        fz_max = 1000.0;

        for (int i = 0; i < NUM_LEG; i++) {
            leg_FSM[i].reset_params(state, i);
        }
        attitude_traj_count = 0;

        // adjust filter strength
        torso_lin_vel_d_rel_filter_x = MovingWindowFilter(int(1000.0 / h * 0.5));
        torso_lin_vel_d_rel_filter_y = MovingWindowFilter(int(1000.0 / h * 0.5));

        terrain_angle_filter = MovingWindowFilter(1000.0 / h * 0.5);

        model_ptr = std::make_shared<QuadrupedModel>();

        C_mat_1 << 1, 0, -mu,  //  fx <= mu*fz
                   -1, 0, -mu, // -fx <= mu*fz
                   0, 1, -mu,  //  fy <= mu*fz
                   0, -1, -mu, // -fy <= mu*fz
                   0, 0, 1,    //  fz <= fz_max
                   0, 0, -1;   // -fz <= 0
        
        C_mat_2 << 1, 0, mu,  //  fx <= -mu*fz
                   -1, 0, mu, // -fx <= -mu*fz
                   0, 1, mu,  //  fy <= -mu*fz
                   0, -1, mu, // -fy <= -mu*fz
                   0, 0, -1,  // -fz <= fz_max
                   0, 0, 1;   //  fz <= 0
        
        x_init.resize(n);
    }

    bool QuatMpc::update(LeggedState &state) {
        goal_update(state);
        foot_update(state);
        grf_update(state);
        if (state.param.terrain_adpt_state == 1) {
            terrain_update(state);
        }

        state.ctrl.optimized_state.segment<3>(0) = state.ctrl.torso_pos_d;
        state.ctrl.optimized_state.segment<3>(3) = state.ctrl.torso_euler_d;

        // TODO: set this flag to false if the MPC solver fails
        return true;
    }

    bool QuatMpc::goal_update(LeggedState &state) {
        if (state.estimation_inited == false) {
            std::cout << "estimation not inited" << std::endl;
            return true;
        }
        // read joy command 
        
        // TODO: add a filter here for lin_vel_d_rel
        if (state.ctrl.torso_lin_vel_d_rel[0] < state.joy.velx) {
            state.ctrl.torso_lin_vel_d_rel[0] += 1.0 * h / 1000.0;
        } else if (state.ctrl.torso_lin_vel_d_rel[0] > state.joy.velx) {
            state.ctrl.torso_lin_vel_d_rel[0] -= 1.0 * h / 1000.0;
        }

        if (state.ctrl.torso_lin_vel_d_rel[1] < state.joy.vely) {
            state.ctrl.torso_lin_vel_d_rel[1] += 1.0 * h / 1000.0;
        } else if (state.ctrl.torso_lin_vel_d_rel[1] > state.joy.vely) {
            state.ctrl.torso_lin_vel_d_rel[1] -= 1.0 * h / 1000.0;
        }

        state.ctrl.torso_lin_vel_d_rel[2] = 0.0;
        state.ctrl.torso_lin_vel_d_world = state.fbk.torso_rot_mat_z * state.ctrl.torso_lin_vel_d_rel;

        state.ctrl.torso_pos_d[0] = state.fbk.torso_pos_world[0] + state.ctrl.torso_lin_vel_d_world[0] * h * horizon / 1000.0;
        state.ctrl.torso_pos_d[1] = state.fbk.torso_pos_world[1] + state.ctrl.torso_lin_vel_d_world[1] * h * horizon / 1000.0;
        state.ctrl.torso_pos_d[2] = state.joy.body_height;

        state.ctrl.torso_ang_vel_d_rel[0] = 0.0;
        state.ctrl.torso_ang_vel_d_rel[1] = 0.0;
        state.ctrl.torso_ang_vel_d_rel[2] = state.joy.yaw_rate;

        state.ctrl.torso_ang_vel_d_body[0] = state.joy.roll_rate;
        state.ctrl.torso_ang_vel_d_body[1] = state.joy.pitch_rate;
        state.ctrl.torso_ang_vel_d_body[2] = state.joy.yaw_rate;

        if (state.joy.sin_ang_vel) {
            state.ctrl.torso_ang_vel_d_body[0] = 3.14 / 4 * cos(2 * 3.14 / 150 * attitude_traj_count);
            state.ctrl.torso_ang_vel_d_body[1] = 3.14 / 4 * cos(2 * 3.14 / 150 * attitude_traj_count);
            state.ctrl.torso_ang_vel_d_body[2] = 3.14 / 4 * cos(2 * 3.14 / 150 * attitude_traj_count);
            attitude_traj_count += 1;
        }
        
        state.ctrl.torso_euler_d[2] += state.joy.yaw_rate * h / 1000.0;

        Eigen::Vector4d torso_quat_d_vec;
        torso_quat_d_vec << state.ctrl.torso_quat_d.w(), state.ctrl.torso_quat_d.x(), state.ctrl.torso_quat_d.y(), state.ctrl.torso_quat_d.z();
        torso_quat_d_vec = torso_quat_d_vec + 0.5 * QuaternionUtils::G(torso_quat_d_vec) * state.ctrl.torso_ang_vel_d_body * h / 1000.0;
        torso_quat_d_vec = torso_quat_d_vec / torso_quat_d_vec.norm();
        state.ctrl.torso_quat_d.w() = torso_quat_d_vec[0];
        state.ctrl.torso_quat_d.x() = torso_quat_d_vec[1];
        state.ctrl.torso_quat_d.y() = torso_quat_d_vec[2];
        state.ctrl.torso_quat_d.z() = torso_quat_d_vec[3];
        
        return true;
    }

    bool QuatMpc::grf_update(LeggedState &state) {
        // when entering grf update, foot contacts (plan_contacts) should be updated
        // by the foot update function and leg_FSMs

        // TODO: pass legFSM into the convex MPC solver to predict contact 

        

        
        auto t_start = std::chrono::high_resolution_clock::now();
        

        ALTROSolver solver(horizon);

        /// REFERENCES ///
        Eigen::Vector3d r_desired = Eigen::Vector3d::Zero();
        Eigen::Vector4d q_desired = Eigen::Vector4d::Zero();
        Eigen::Vector3d v_desired = Eigen::Vector3d::Zero();
        Eigen::Vector3d w_desired = Eigen::Vector3d::Zero();
        
        // relative frame: center at body, only rotate yaw angle from world frame
        // absolute frame: center at body, orientation is the same as world frame
        // all the desired values should be in body frame

        // Walk on flat ground
        // r_desired = state.fbk.torso_rot_mat.transpose() * (state.ctrl.torso_pos_d - state.fbk.torso_pos_world); // convert to body frame
        // q_desired << state.ctrl.torso_quat_d.w(),
        //              state.ctrl.torso_quat_d.x(),
        //              state.ctrl.torso_quat_d.y(),
        //              state.ctrl.torso_quat_d.z();
        // v_desired = state.fbk.torso_rot_mat.transpose() * state.fbk.torso_rot_mat_z * state.ctrl.torso_lin_vel_d_rel; // convert to body frame
        // w_desired = state.ctrl.torso_ang_vel_d_body;

        // Spider man demo
        r_desired = state.ctrl.torso_lin_vel_d_rel * h * horizon / 1000.0;
        r_desired[2] = state.joy.velz * h * horizon / 1000.0;
        q_desired << state.ctrl.torso_quat_d.w(),
                     state.ctrl.torso_quat_d.x(),
                     state.ctrl.torso_quat_d.y(),
                     state.ctrl.torso_quat_d.z();
        v_desired = state.ctrl.torso_lin_vel_d_rel;
        w_desired = state.ctrl.torso_ang_vel_d_body;
        // w_desired.setZero();

        Eigen::VectorXd x_ref(n);
        Eigen::VectorXd u_ref(m);
        x_ref.setZero();
        u_ref.setZero();

        x_ref.head(3) = r_desired;
        x_ref.segment<4>(3) = q_desired;
        x_ref.segment<3>(7) = v_desired;
        x_ref.tail(3) = w_desired;

        num_contacts = 0;
        for (int i = 0; i < 4; i++) {
            if (state.ctrl.plan_contacts[i]) num_contacts += 1;
        }

        // Walk on flat ground
        // u_ref << 0.0, 0.0, state.ctrl.plan_contacts[0] * 12.84 * 9.81 / num_contacts,
        //          0.0, 0.0, state.ctrl.plan_contacts[1] * 12.84 * 9.81 / num_contacts,
        //          0.0, 0.0, state.ctrl.plan_contacts[2] * 12.84 * 9.81 / num_contacts,
        //          0.0, 0.0, state.ctrl.plan_contacts[3] * 12.84 * 9.81 / num_contacts;
        
        // std::cout << "plan_contacts: " << state.ctrl.plan_contacts[0] << state.ctrl.plan_contacts[1] << state.ctrl.plan_contacts[2] << state.ctrl.plan_contacts[3] << std::endl;
        // std::cout << "num_contacts: " << num_contacts << std::endl;
        // Spider man demo
        if (num_contacts == 4) {
            u_ref << 12.84 * 9.81 / 4, 0.0, 12.84 * 9.81 / 4 / mu,
                     12.84 * 9.81 / 4, 0.0, -12.84 * 9.81 / 4 / mu,
                     12.84 * 9.81 / 4, 0.0, -12.84 * 9.81 / 4 / mu,
                     12.84 * 9.81 / 4, 0.0, 12.84 * 9.81 / 4 / mu;
        } else if (state.ctrl.plan_contacts[0] && state.ctrl.plan_contacts[1] && !state.ctrl.plan_contacts[2] && !state.ctrl.plan_contacts[3]) {
            u_ref << 12.84 * 9.81 / 2, -12.84 * 9.81 / 4, 12.84 * 9.81 / 2,
                     12.84 * 9.81 / 2, 12.84 * 9.81 / 4, -12.84 * 9.81 / 2,
                     0.0, 0.0, 0.0,
                     0.0, 0.0, 0.0;
        } else if (!state.ctrl.plan_contacts[0] && !state.ctrl.plan_contacts[1] && state.ctrl.plan_contacts[2] && state.ctrl.plan_contacts[3]) {
            u_ref << 0.0, 0.0, 0.0,
                     0.0, 0.0, 0.0,
                     12.84 * 9.81 / 2, -12.84 * 9.81 / 4, -12.84 * 9.81 / 2,
                     12.84 * 9.81 / 2, 12.84 * 9.81 / 4, 12.84 * 9.81 / 2;
        }

        x_traj_ref.clear();
        u_traj_ref.clear();
        for (int k = 0; k <= horizon; k++) {
            x_traj_ref.emplace_back(x_ref);
            u_traj_ref.emplace_back(u_ref);
        }

        // std::cout << x_ref.transpose() << std::endl;
        // std::cout << u_ref.transpose() << std::endl;

        /// DYNAMICS ///
        // std::cout << "foot_pos_body" << std::endl << state.fbk.foot_pos_body << std::endl;
        

        // we should use foot position in the body frame
        ct_dyn = [&](double *x_dot, const double *x, const double *u) {
            model_ptr->ct_srb_quat_dynamics(x_dot, x, u, state.fbk.foot_pos_body, state.param.trunk_inertia, state.fbk.torso_rot_mat);
        };
        ct_jac = [&](double *jac, const double *x, const double *u) {
            model_ptr->ct_srb_quat_jacobian(jac, x, u, state.fbk.foot_pos_body, state.param.trunk_inertia);
        };
        dt_dyn = midpoint_dynamics(n, m, ct_dyn);
        dt_jac = midpoint_jacobian(n, m, ct_dyn, ct_jac);

        /// CONSTRAINTS ///
        friction_cone_con = [&](a_float *c, const a_float *x, const a_float *u) {
            (void)x;
            Eigen::Map<Eigen::Matrix<a_float, 24, 1>> C(c);
            Eigen::Map<const Eigen::VectorXd> u_vec(u, 12);
            C.setZero();

            // Walk on flat ground
            // for (int i = 0; i < 4; i++) {
            //     Eigen::Matrix<a_float, 6, 1> b_vec;
            //     b_vec << 0, 0, 0, 0, -fz_max * state.ctrl.plan_contacts[i], fz_min * state.ctrl.plan_contacts[i];
            //     C.segment<6>(i * 6) = C_mat_1 * state.fbk.torso_rot_mat * u_vec.segment<3>(i * 3) + b_vec;
            // }

            // Spider man demo
            for (int i = 0; i < 4; i++) {
                Eigen::Matrix<a_float, 6, 1> b_vec;
                if (state.ctrl.plan_contacts[i]) b_vec << 0, 0, 0, 0, -fz_max, 0;
                else b_vec << 0, 0, 0, 0, 0, 0;

                if (i == 0 || i == 3) {
                    // FL or RR, fz must be positive
                    C.segment<6>(i * 6) = C_mat_1 * u_vec.segment<3>(i * 3) + b_vec;
                } else if (i == 1 || i == 2) {
                    // FR or RL, fz should be negative
                    C.segment<6>(i * 6) = C_mat_2 * u_vec.segment<3>(i * 3) + b_vec;
                }
            }
        };

        friction_cone_jac = [&](a_float *jac, const a_float *x, const a_float *u) {
            (void)x;
            (void)u;
            Eigen::Map<Eigen::Matrix<a_float, 24, 24>> J(jac);
            J.setZero();

            // Walk on flat ground
            // for (int i = 0; i < 4; i++) {
            //     J.block<6, 3>(i * 6, 12 + i * 3) = C_mat_1 * state.fbk.torso_rot_mat;
            // }

            // Spider man demo
            for (int i = 0; i < 4; i++) {
                if (i == 0 || i == 3) {
                    J.block<6, 3>(i * 6, 12 + i * 3) = C_mat_1;
                } else if (i == 1 || i == 2) {
                    J.block<6, 3>(i * 6, 12 + i * 3) = C_mat_2;
                }
            }
        };

        /// SETUP ///        
        solver.SetOptions(opts);

        solver.SetDimension(n, m);
        solver.SetExplicitDynamics(dt_dyn, dt_jac);
        solver.SetTimeStep(h / 1000.0);

        for (int i = 0; i <= horizon; i++) {
            // solver.SetQuaternionCost(n, m, Qd.data(), Rd.data(), w, x_traj_ref.at(i).data(), u_traj_ref.at(i).data(), i, 0);
            solver.SetQuaternionCost(n, m, state.param.q_weights.data(), state.param.r_weights.data(), state.param.w, x_traj_ref.at(i).data(), u_traj_ref.at(i).data(), i, 0);
        }
        // solver.SetConstraint(friction_cone_con, friction_cone_jac, 24, ConstraintType::INEQUALITY, "friction cone", 0, horizon + 1);

        
        state.fbk.torso_lin_vel_body = state.fbk.torso_rot_mat.transpose() * state.fbk.torso_lin_vel_world;
        x_init << 0.0,
                  0.0,
                  0.0,
                  state.fbk.torso_quat.w(),
                  state.fbk.torso_quat.x(),
                  state.fbk.torso_quat.y(),
                  state.fbk.torso_quat.z(),
                  state.fbk.torso_lin_vel_body[0],
                  state.fbk.torso_lin_vel_body[1],
                  state.fbk.torso_lin_vel_body[2],
                  state.fbk.torso_ang_vel_body[0],
                  state.fbk.torso_ang_vel_body[1],
                  state.fbk.torso_ang_vel_body[2];
        // std::cout << "x_init = " << x_init.transpose() << std::endl;
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

        /// SAVE ///
        Eigen::VectorXd u(m);
        solver.GetInput(u.data(), 0);

        for (int i = 0; i < NUM_LEG; i++) {
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
                // std::cout << "foot_pos_target_world: " << state.ctrl.foot_pos_target_world.block<3, 1>(0, i).transpose() << std::endl;
                // std::cout << "foot_pos_world: " << state.fbk.foot_pos_world.block<3, 1>(0, i).transpose() << std::endl;
                state.ctrl.gait_counter[i] = leg_FSM[i].update(h / 1000.0, state.param.gait_freq,
                                                               state.fbk.foot_pos_world.block<3, 1>(0, i),
                                                               state.ctrl.foot_pos_target_world.block<3, 1>(0, i),
                                                               state.fbk.foot_contact_flag[i]);

                // TODO: gait phase of each leg is individually controlled, so we may need to occasionally synchrinize them, for example if all leg are in contact, average their gait phase and set them to that value
            }
            std::cout << std::endl;
            for (int i = 0; i < NUM_LEG; i++) {
                state.ctrl.plan_contacts[i] = leg_FSM[i].get_contact_state();
            }
        }

        return true;
    }

    bool QuatMpc::terrain_update(LeggedState &state) {
        Eigen::Vector3d walk_surf_coef = Utils::get_walk_surf_coef(state.fbk.foot_pos_abs_prev);
        Eigen::Vector3d flat_ground_coef;
        flat_ground_coef << 0, 0, 1;

        double terrain_angle = 0.0;

        // only do terrain adaption in walking mode && body height > 0.1 m
        if (state.ctrl.movement_mode > 0 && state.fbk.torso_pos_world[2] > 0.1) {
            // terrain_angle = Utils::cal_dihedral_angle(flat_ground_coef, walk_surf_coef);
            terrain_angle = terrain_angle_filter.CalculateAverage(Utils::cal_dihedral_angle(flat_ground_coef, walk_surf_coef));

            // prevent excessive angle
            if (terrain_angle > 0.349) {
                terrain_angle = 0.349;
            }
            if (terrain_angle < -0.349) {
                terrain_angle = -0.349;
            }

            // get desired pitch angle
            double F_R_diff = state.fbk.foot_pos_abs_prev(2, 0) + state.fbk.foot_pos_abs_prev(2, 1) - state.fbk.foot_pos_abs_prev(2, 2) - state.fbk.foot_pos_abs_prev(2, 3); // FL, FR, RL, RR
            if (F_R_diff > 0.05) {
                state.ctrl.torso_euler_d[1] = -terrain_angle;
            } else {
                state.ctrl.torso_euler_d[1] = terrain_angle;
            }
        }

        return true;
    }

}  // namespace legged
