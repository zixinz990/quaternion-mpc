#include "GyrostatEulerMpc.h"

namespace mpc {

GyrostatEulerMpc::GyrostatEulerMpc(RobotState &robot_state) {
    h = robot_state.params.mpc_dt;
    horizon = robot_state.params.mpc_horizon;

    opts.verbose = Verbosity::Silent;
    opts.iterations_max = 15;
    opts.use_backtracking_linesearch = true;
    opts.penalty_scaling = 10.0;

    model_ptr = make_shared<GyrostatModel>();

    u_traj_ref.clear();
    Eigen::Matrix<double, GYROSTAT_MPC_INPUT_DIM, 1> u_ref;
    u_ref << 0.0, 0.0;
    for (int i = 0; i <= horizon; i++) {
        u_traj_ref.push_back(u_ref);
    }
}

void GyrostatEulerMpc::update(RobotState &robot_state) {
    goal_update(robot_state);
    ctrl_update(robot_state);    
}

void GyrostatEulerMpc::goal_update(RobotState &robot_state) {        
    // Update angular velocity goal
    robot_state.ctrl.torso_ang_vel_d_body[0] = robot_state.joy_cmd.joy_roll_vel;
    robot_state.ctrl.torso_ang_vel_d_body[1] = robot_state.joy_cmd.joy_pitch_vel;
    robot_state.ctrl.torso_ang_vel_d_body[2] = robot_state.joy_cmd.joy_yaw_vel;
    
    // Falling cat experiment
    Eigen::Vector4d torso_quat_d_vec = robot_state.ctrl.best_falling_pose; // [w, x, y, z]
    Eigen::Vector3d torso_euler_d = Utils::quat_to_euler(Eigen::Quaterniond(torso_quat_d_vec[0], torso_quat_d_vec[1], torso_quat_d_vec[2], torso_quat_d_vec[3]));
    
    // Update reference state trajectory
    x_traj_ref.clear();
    Eigen::Matrix<double, GYROSTAT_EULER_MPC_STATE_DIM, 1> x_ref;
    for (int i = 0; i <= horizon; i++) {
        x_ref.setZero();

        x_ref.head(3) = torso_euler_d;
        x_ref.block<3, 1>(3, 0) = robot_state.ctrl.torso_ang_vel_d_body;
        x_ref.tail(2) << 0, 0;

        x_traj_ref.push_back(x_ref);
    }
}

void GyrostatEulerMpc::ctrl_update(RobotState &robot_state) {
    /// DYNAMICS ///
    ct_dyn = [&](double *x_dot, const double *x, const double *u) {
        model_ptr->ct_euler_dyn(x_dot, x, u,
                               robot_state.params.robot_inertia);
    };
    ct_jac = [&](double *jac, const double *x, const double *u) {
        model_ptr->ct_euler_jac(jac, x, u,
                               robot_state.params.robot_inertia);
    };
    dt_dyn = Utils::midpoint_dyn(GYROSTAT_EULER_MPC_STATE_DIM, GYROSTAT_MPC_INPUT_DIM, ct_dyn);
    dt_jac = Utils::midpoint_jac(GYROSTAT_EULER_MPC_STATE_DIM, GYROSTAT_MPC_INPUT_DIM, ct_dyn, ct_jac);

    /// SETUP ///
    ALTROSolver solver(horizon);
    solver.SetOptions(opts);
    solver.SetDimension(GYROSTAT_EULER_MPC_STATE_DIM, GYROSTAT_MPC_INPUT_DIM);
    solver.SetExplicitDynamics(dt_dyn, dt_jac);
    solver.SetTimeStep(h);

    for (int i = 0; i <= horizon; i++) {
        solver.SetLQRCost(GYROSTAT_EULER_MPC_STATE_DIM, GYROSTAT_MPC_INPUT_DIM,
                          robot_state.params.euler_mpc_q_weights.data(), robot_state.params.mpc_r_weights.data(),
                          x_traj_ref.at(i).data(), u_traj_ref.at(i).data(), i);
    }
    
    x_init.setZero();
    x_init << robot_state.fbk.torso_euler[0],
              robot_state.fbk.torso_euler[1],
              robot_state.fbk.torso_euler[2],
              robot_state.fbk.torso_ang_vel_body[0],
              robot_state.fbk.torso_ang_vel_body[1],
              robot_state.fbk.torso_ang_vel_body[2],
              robot_state.fbk.wheel_vel[0] * robot_state.params.wheel_inertia[0],
              robot_state.fbk.wheel_vel[1] * robot_state.params.wheel_inertia[1];

    // std::cout << "horizon: " << horizon << std::endl;
    // std::cout << "q_weights: " << robot_state.params.quat_mpc_q_weights.transpose() << std::endl;
    // std::cout << "r_weights: " << robot_state.params.mpc_r_weights.transpose() << std::endl;
    // std::cout << "quat_weight: " << robot_state.params.mpc_quat_weight << std::endl;
    // std::cout << "torso_quat: " << robot_state.fbk.torso_quat << std::endl;
    // std::cout << "torso_ang_vel: " << robot_state.fbk.torso_ang_vel_body.transpose() << std::endl;
    // std::cout << "wheel_vel: " << robot_state.fbk.wheel_vel.transpose() << std::endl;

    solver.SetInitialState(x_init.data(), GYROSTAT_EULER_MPC_STATE_DIM);
    solver.Initialize();

    // Initial guesses
    for (int i = 0; i <= horizon; i++) {
        solver.SetState(x_traj_ref.at(i).data(), GYROSTAT_EULER_MPC_STATE_DIM, i);
    }
    solver.SetInput(u_traj_ref.at(0).data(), GYROSTAT_MPC_INPUT_DIM);

    /// SOLVE ///
    solver.Solve();

    /// SAVE ///
    Eigen::VectorXd u(GYROSTAT_MPC_INPUT_DIM);
    solver.GetInput(u.data(), 0);
    // std::cout << "u: " << u.transpose() << std::endl;
    robot_state.ctrl.wheel_tau_d = u;
}

}  // namespace mpc
