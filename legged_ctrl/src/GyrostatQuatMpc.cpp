#include "GyrostatQuatMpc.h"

namespace mpc {

GyrostatQuatMpc::GyrostatQuatMpc(RobotState &robot_state) {
    h = robot_state.params.mpc_dt;
    horizon = robot_state.params.mpc_horizon;

    opts.verbose = Verbosity::Silent;
    opts.iterations_max = 15;
    opts.use_backtracking_linesearch = true;
    opts.use_quaternion = true;
    opts.quat_start_index = 0;
    opts.penalty_scaling = 10.0;

    model_ptr = make_shared<GyrostatModel>();

    u_traj_ref.clear();
    Eigen::Matrix<double, GYROSTAT_MPC_INPUT_DIM, 1> u_ref;
    u_ref << 0.0, 0.0;
    for (int i = 0; i <= horizon; i++) {
        u_traj_ref.push_back(u_ref);
    }
}

void GyrostatQuatMpc::update(RobotState &robot_state) {
    goal_update(robot_state);
    ctrl_update(robot_state);    
}

void GyrostatQuatMpc::goal_update(RobotState &robot_state) {        
    // Update angular velocity goal
    robot_state.ctrl.torso_ang_vel_d_body[0] = robot_state.joy_cmd.joy_roll_vel;
    robot_state.ctrl.torso_ang_vel_d_body[1] = robot_state.joy_cmd.joy_pitch_vel;
    robot_state.ctrl.torso_ang_vel_d_body[2] = robot_state.joy_cmd.joy_yaw_vel;

    // Falling cat experiment
    Eigen::Vector4d torso_quat_d_vec = robot_state.ctrl.best_falling_pose;
    std::cout << "torso_quat_d_vec: " << torso_quat_d_vec.transpose() << std::endl;
    
    // Update reference state trajectory
    x_traj_ref.clear();
    Eigen::Matrix<double, GYROSTAT_QUAT_MPC_STATE_DIM, 1> x_ref;
    for (int i = 0; i <= horizon; i++) {
        x_ref.setZero();

        x_ref.head(4) = torso_quat_d_vec;
        x_ref.block<3, 1>(4, 0) = robot_state.ctrl.torso_ang_vel_d_body;
        x_ref.tail(2) << 0, 0;

        x_traj_ref.push_back(x_ref);
    }
}

void GyrostatQuatMpc::ctrl_update(RobotState &robot_state) {
    /// DYNAMICS ///
    ct_dyn = [&](double *x_dot, const double *x, const double *u) {
        model_ptr->ct_quat_dyn(x_dot, x, u,
                               robot_state.params.robot_inertia, robot_state.params.robot_mass, robot_state.fbk.torso_rot_mat);
    };
    ct_jac = [&](double *jac, const double *x, const double *u) {
        model_ptr->ct_quat_jac(jac, x, u,
                               robot_state.params.robot_inertia, robot_state.params.robot_mass);
    };
    dt_dyn = Utils::midpoint_dyn(GYROSTAT_QUAT_MPC_STATE_DIM, GYROSTAT_MPC_INPUT_DIM, ct_dyn);
    dt_jac = Utils::midpoint_jac(GYROSTAT_QUAT_MPC_STATE_DIM, GYROSTAT_MPC_INPUT_DIM, ct_dyn, ct_jac);

    /// SETUP ///
    ALTROSolver solver(horizon);
    solver.SetOptions(opts);
    solver.SetDimension(GYROSTAT_QUAT_MPC_STATE_DIM, GYROSTAT_MPC_INPUT_DIM);
    solver.SetExplicitDynamics(dt_dyn, dt_jac);
    solver.SetTimeStep(h);
    for (int i = 0; i <= horizon; i++) {
        solver.SetQuaternionCost(GYROSTAT_QUAT_MPC_STATE_DIM, GYROSTAT_MPC_INPUT_DIM,
                                 robot_state.params.quat_mpc_q_weights.data(), robot_state.params.mpc_r_weights.data(), robot_state.params.mpc_quat_weight,
                                 x_traj_ref.at(i).data(), u_traj_ref.at(i).data(), i, 0);
    }
    
    x_init.setZero();
    x_init << robot_state.fbk.torso_quat.w(),
              robot_state.fbk.torso_quat.x(),
              robot_state.fbk.torso_quat.y(),
              robot_state.fbk.torso_quat.z(),
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

    solver.SetInitialState(x_init.data(), GYROSTAT_QUAT_MPC_STATE_DIM);
    solver.Initialize();

    // Initial guesses
    for (int i = 0; i <= horizon; i++) {
        solver.SetState(x_traj_ref.at(i).data(), GYROSTAT_QUAT_MPC_STATE_DIM, i);
    }
    solver.SetInput(u_traj_ref.at(0).data(), GYROSTAT_MPC_INPUT_DIM);

    /// SOLVE ///
    solver.Solve();

    /// SAVE ///
    Eigen::VectorXd u(GYROSTAT_MPC_INPUT_DIM);
    solver.GetInput(u.data(), 0);
    std::cout << "u: " << u.transpose() << std::endl;
    robot_state.ctrl.wheel_tau_d = u;
}

}  // namespace mpc
