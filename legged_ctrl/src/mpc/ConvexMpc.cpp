#include "mpc/ConvexMpc.h"

using namespace altro;
namespace legged {
    ConvexMpc::ConvexMpc(LeggedState &state) {
        n = 12;
        m = 12;
        h = state.param.mpc_update_period;
        horizon = state.param.mpc_horizon;
        for (int i = 0; i < NUM_LEG; i++) {
            leg_FSM[i].reset_params(state, i);
        }
        num_contacts = 0;
        model_ptr = std::make_shared<QuadrupedModel>();
        friction_cone_jac = [&](a_float *dt_jac, const a_float *x, const a_float *u) {
            (void) x;
            (void) u;
            Eigen::Map<Eigen::Matrix<a_float, 24, 24>> J(dt_jac);
            J.setZero();

            for (int i = 0; i < 4; i++) {
                J(0 + i * 6, 12 + i * 3) = 1;               // dc0/dfx
                J(0 + i * 6, 14 + i * 3) = -state.param.mu; // dc0/dfz
                J(1 + i * 6, 12 + i * 3) = -1;              // dc1/dfx
                J(1 + i * 6, 14 + i * 3) = -state.param.mu; // dc1/dfz
                J(2 + i * 6, 13 + i * 3) = 1;               // dc2/dfy
                J(2 + i * 6, 14 + i * 3) = -state.param.mu; // dc2/dfz
                J(3 + i * 6, 13 + i * 3) = -1;              // dc3/dfy
                J(3 + i * 6, 14 + i * 3) = -state.param.mu; // dc3/dfz
                J(4 + i * 6, 14 + i * 3) = 1;               // dc4/dfz
                J(5 + i * 6, 14 + i * 3) = -1;              // dc5/dfz
            }
        };
        x_init.resize(n);
        x_init.setZero();
        opts.verbose = Verbosity::Inner;
        opts.iterations_max = 5;
        opts.use_backtracking_linesearch = true;
    }

    bool ConvexMpc::update(LeggedState &state) {
        goal_update(state);
        foot_update(state);
        grf_update(state);
        if (state.param.terrain_adpt_state == 1) {
            terrain_update(state);
        }
        return true;
    }

    bool ConvexMpc::goal_update(LeggedState &state) {
        if (state.estimator_init == false) {
            std::cout << "Estimator is not initialized!" << std::endl;
            return true;
        }
        
        state.ctrl.torso_pos_d_world[0] = state.joy.body_x;
        state.ctrl.torso_pos_d_world[1] = state.joy.body_y;       
        state.ctrl.torso_pos_d_world[2] = state.joy.body_height;

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
        state.ctrl.torso_ang_vel_d_body[2] = state.joy.yaw_rate;
        return true;
    }

    bool ConvexMpc::grf_update(LeggedState &state) {
        // TODO: pass legFSM into the convex MPC solver to predict contact 

        auto t_start = std::chrono::high_resolution_clock::now();
        ALTROSolver solver(horizon);

        ///////////////////////////// References /////////////////////////////
        x_traj_ref.clear();
        u_traj_ref.clear();

        num_contacts = state.ctrl.plan_contacts[0] + state.ctrl.plan_contacts[1] + state.ctrl.plan_contacts[2] + state.ctrl.plan_contacts[3];

        Eigen::VectorXd x_(n);
        Eigen::VectorXd u_(m);
        for (int i = 0; i <= horizon; i++) {
            x_ << 0.0, // roll, always 0
                  0.0, // pitch, always 0
                  state.fbk.torso_euler[2] + state.ctrl.torso_ang_vel_d_body[2] * h / 1000.0 * i, // yaw
                  state.ctrl.torso_pos_d_world[0], // x, don't track
                  state.ctrl.torso_pos_d_world[1], // y, don't track
                  state.ctrl.torso_pos_d_world[2],  // z
                  0.0, // ang vel x, always 0, don't track
                  0.0, // ang vel y, always 0, don't track
                  state.ctrl.torso_ang_vel_d_body[2], // ang vel z
                  state.ctrl.torso_lin_vel_d_world[0], // vx
                  state.ctrl.torso_lin_vel_d_world[1], // vy
                  0.0; // vz, always 0
            u_ << 0.0, 0.0, state.param.robot_mass * 9.81 / num_contacts * state.ctrl.plan_contacts[0],
                  0.0, 0.0, state.param.robot_mass * 9.81 / num_contacts * state.ctrl.plan_contacts[1],
                  0.0, 0.0, state.param.robot_mass * 9.81 / num_contacts * state.ctrl.plan_contacts[2],
                  0.0, 0.0, state.param.robot_mass * 9.81 / num_contacts * state.ctrl.plan_contacts[3];
            x_traj_ref.emplace_back(x_);
            u_traj_ref.emplace_back(u_);
        }

        ///////////////////////////// Dynamics /////////////////////////////
        ct_dyn = [&](double *x_dot, const double *x, const double *u) {
            model_ptr->ct_srb_dynamics(x_dot, x, u, state.fbk.foot_pos_abs_com);
        };
        ct_jac = [&](double *dt_jac, const double *x, const double *u) {
            model_ptr->ct_srb_jacobian(dt_jac, x, u, state.fbk.foot_pos_abs_com);
        };

        dt_dyn = midpoint_dynamics(n, m, ct_dyn);
        dt_jac = midpoint_jacobian(n, m, ct_dyn, ct_jac);
        // dt_dyn = forward_euler_dynamics(n, m, ct_dyn);
        // dt_jac = forward_euler_jacobian(n, m, ct_dyn, ct_jac);

        ///////////////////////////// Constraints /////////////////////////////
        friction_cone_con = [&](a_float *c, const a_float *x, const a_float *u) {
            (void) x;
            for (int i = 0; i < 4; i++) {
                c[0 + i * 6] = u[0 + i * 3] - state.param.mu * u[2 + i * 3]; // fx - mu*fz <= 0
                c[1 + i * 6] = -u[0 + i * 3] - state.param.mu * u[2 + i * 3]; // -fx - mu*fz <= 0
                c[2 + i * 6] = u[1 + i * 3] - state.param.mu * u[2 + i * 3]; // fy - mu*fz <= 0
                c[3 + i * 6] = -u[1 + i * 3] - state.param.mu * u[2 + i * 3]; // -fy - mu*fz <= 0
                c[4 + i * 6] = u[2 + i * 3] - state.param.fz_max * state.ctrl.plan_contacts[i]; // fz <= fz_max
                c[5 + i * 6] = -u[2 + i * 3]; // -fz + 10 <= 0
            }
        };

        ///////////////////////////// Setup /////////////////////////////
        solver.SetDimension(n, m); // Set Dimension
        solver.SetExplicitDynamics(dt_dyn, dt_jac);  // Set Dynamics
        solver.SetTimeStep(h / 1000.0); // Set Time Step
        // Set cost
        for (int k = 0; k <= horizon; k++) {
            solver.SetLQRCost(n, m,
                              state.param.q_weights.data(), state.param.r_weights.data(),
                              x_traj_ref.at(k).data(), u_traj_ref.at(k).data(), k);
        }
        // Set constraints
        solver.SetConstraint(friction_cone_con, friction_cone_jac, 24, ConstraintType::INEQUALITY, "friction cone", 0,
                             horizon + 1);
        // Set initial state
        x_init << state.fbk.torso_euler[0],
                  state.fbk.torso_euler[1],
                  state.fbk.torso_euler[2],
                  state.fbk.torso_pos_world[0],
                  state.fbk.torso_pos_world[1],
                  state.fbk.torso_pos_world[2],
                  state.fbk.torso_ang_vel_world[0],
                  state.fbk.torso_ang_vel_world[1],
                  state.fbk.torso_ang_vel_world[2],
                  state.fbk.torso_lin_vel_world[0],
                  state.fbk.torso_lin_vel_world[1],
                  state.fbk.torso_lin_vel_world[2];

        solver.SetInitialState(x_init.data(), n);
        // Initialize solver
        solver.Initialize();
        // Set initial state guess
        for (int k = 0; k <= horizon; k++) {
            solver.SetState(x_traj_ref.at(k).data(), n, k);
        }
        // Set initial input guess
        solver.SetInput(u_traj_ref.at(0).data(), m);

        ///////////////////////////// Solve /////////////////////////////        
        solver.SetOptions(opts);
        solver.Solve();

        auto t_end = std::chrono::high_resolution_clock::now();
        using SecondsDouble = std::chrono::duration<double, std::ratio<1>>;
        SecondsDouble t_total = std::chrono::duration_cast<SecondsDouble>(t_end - t_start);
        // fmt::print("Total time = {} ms\n", t_total * 1000);

        Eigen::VectorXd u(m);
        solver.GetInput(u.data(), 0);
        for (int i = 0; i < NUM_LEG; ++i) {
            state.ctrl.optimized_input.segment<3>(3 * i) = state.fbk.torso_rot_mat.transpose() * u.segment<3>(i * 3);
        }

        state.ctrl.optimized_state.segment<3>(0) = state.ctrl.torso_pos_d_world;
        state.ctrl.optimized_state.segment<3>(3) = state.ctrl.torso_euler_d;

        return true;
    }

    bool ConvexMpc::foot_update(LeggedState &state) {
        if (state.ctrl.movement_mode == 0) {
            for (int i = 0; i < NUM_LEG; i++) {
                leg_FSM[i].reset();
                state.ctrl.plan_contacts[i] = true;
            }
        } else {
            for (int i = 0; i < NUM_LEG; i++) {
                state.ctrl.gait_counter[i] = leg_FSM[i].update(h / 1000.0, state.param.gait_freq,
                                                               state.fbk.foot_pos_world.block<3, 1>(0, i),
                                                               state.ctrl.foot_pos_target_world.block<3, 1>(0, i),
                                                               state.fbk.foot_contact_flag[i]);
            }
            for (int i = 0; i < NUM_LEG; i++) {
                state.ctrl.plan_contacts[i] = leg_FSM[i].get_contact_state();
            }
        }
        for (int i = 0; i < NUM_LEG; i++) {
            state.ctrl.optimized_state.segment<3>(6 + 3 * i) = leg_FSM[i].FSM_foot_pos_target_world;
            state.ctrl.optimized_input.segment<3>(12 + 3 * i) = leg_FSM[i].FSM_foot_vel_target_world;
            state.ctrl.optimized_input.segment<3>(24 + 3 * i) = leg_FSM[i].FSM_foot_acc_target_world;
        }
        return true;
    }

    bool ConvexMpc::terrain_update(LeggedState &state) {
        return true;
    }

}  // namespace legged