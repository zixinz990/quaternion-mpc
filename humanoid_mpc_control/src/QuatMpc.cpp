#include "QuatMpc.h"

namespace mpc {

QuatMpc::QuatMpc(RobotState &robot_state) {
    first_iter = true;
    h = robot_state.params.mpc_dt;
    horizon = robot_state.params.mpc_horizon;

    opts.verbose = Verbosity::Silent;
    opts.iterations_max = 15;
    opts.use_backtracking_linesearch = true;
    opts.use_quaternion = true;
    opts.quat_start_index = 3;
    opts.penalty_scaling = 10.0;

    model_ptr = make_shared<HumanoidModel>();

    C_mat << 1, 0, -robot_state.params.mu,   //  fx - mu * fz <= 0
             -1, 0, -robot_state.params.mu,  // -fx - mu * fz <= 0
             0, 1, -robot_state.params.mu,   //  fy - mu * fz <= 0
             0, -1, -robot_state.params.mu,  // -fy - mu * fz <= 0
             0, 0, 1,                        //  fz - fz_max  <= 0
             0, 0, -1;                       // -fz           <= 0

    u_traj_ref.clear();
    Eigen::Matrix<double, MPC_INPUT_DIM, 1> u_ref;
    u_ref << 0.0, 0.0, robot_state.params.robot_mass * 9.81 / 4,
             0.0, 0.0, robot_state.params.robot_mass * 9.81 / 4,
             0.0, 0.0, robot_state.params.robot_mass * 9.81 / 4,
             0.0, 0.0, robot_state.params.robot_mass * 9.81 / 4;
    for (int i = 0; i <= horizon; i++) {
        u_traj_ref.push_back(u_ref);
    }

    robot_state.ctrl.torso_pos_d_world << 0.0, 0.0, 0.78;

    attitude_traj_count = 0;
}

void QuatMpc::update(RobotState &robot_state) {
    goal_update(robot_state);
    ctrl_update(robot_state);    
}

void QuatMpc::goal_update(RobotState &robot_state) {
    // cout << "MPC GOAL UPDATE" << endl;
    // Update linear velocity goal
    robot_state.ctrl.torso_lin_vel_d_rel[0] = robot_state.joy_cmd.joy_vel_x;
    robot_state.ctrl.torso_lin_vel_d_rel[1] = robot_state.joy_cmd.joy_vel_y;
    robot_state.ctrl.torso_lin_vel_d_rel[2] = robot_state.joy_cmd.joy_vel_z;
    robot_state.ctrl.torso_lin_vel_d_world = robot_state.fbk.torso_rot_mat_z * robot_state.ctrl.torso_lin_vel_d_rel;
    // robot_state.ctrl.torso_lin_vel_d_body = robot_state.fbk.torso_rot_mat.transpose() * robot_state.ctrl.torso_lin_vel_d_world;
    robot_state.ctrl.torso_lin_vel_d_body.setZero();
    
    // Update angular velocity goal
    robot_state.ctrl.torso_ang_vel_d_body[0] = robot_state.joy_cmd.joy_roll_vel;
    robot_state.ctrl.torso_ang_vel_d_body[1] = robot_state.joy_cmd.joy_pitch_vel;
    robot_state.ctrl.torso_ang_vel_d_body[2] = robot_state.joy_cmd.joy_yaw_vel;

    // Update torso position goal
    robot_state.ctrl.torso_pos_d_world[0] += robot_state.ctrl.torso_lin_vel_d_world[0] * h;
    robot_state.ctrl.torso_pos_d_world[1] += robot_state.ctrl.torso_lin_vel_d_world[1] * h;
    robot_state.ctrl.torso_pos_d_world[2] += robot_state.ctrl.torso_lin_vel_d_world[2] * h;
    robot_state.ctrl.torso_pos_d_body = robot_state.fbk.torso_rot_mat.transpose() * (robot_state.ctrl.torso_pos_d_world - robot_state.fbk.torso_pos_world);

    // Update torso attitude goal
    if (robot_state.joy_cmd.sin_ang_vel) {
        robot_state.ctrl.torso_ang_vel_d_body[0] = 3.14 / 12 * sin(2 * 3.14 / 900 * attitude_traj_count);
        robot_state.ctrl.torso_ang_vel_d_body[1] = 3.14 / 12 * sin(2 * 3.14 / 900 * attitude_traj_count);
        robot_state.ctrl.torso_ang_vel_d_body[2] = 3.14 / 12 * sin(2 * 3.14 / 900 * attitude_traj_count);
        attitude_traj_count += 1;
    }

    Eigen::Vector4d torso_quat_d_vec;
    torso_quat_d_vec << robot_state.ctrl.torso_quat_d.w(),
                        robot_state.ctrl.torso_quat_d.x(),
                        robot_state.ctrl.torso_quat_d.y(),
                        robot_state.ctrl.torso_quat_d.z();
    // torso_quat_d_vec = torso_quat_d_vec + 0.5 * Utils::G(torso_quat_d_vec) * robot_state.ctrl.torso_ang_vel_d_body * 0.01;
    torso_quat_d_vec = Utils::quat_rk4(torso_quat_d_vec, robot_state.ctrl.torso_ang_vel_d_body, 0.01);
    torso_quat_d_vec = torso_quat_d_vec / torso_quat_d_vec.norm();
    robot_state.ctrl.torso_quat_d.w() = torso_quat_d_vec[0];
    robot_state.ctrl.torso_quat_d.x() = torso_quat_d_vec[1];
    robot_state.ctrl.torso_quat_d.y() = torso_quat_d_vec[2];
    robot_state.ctrl.torso_quat_d.z() = torso_quat_d_vec[3];
    
    // robot_state.ctrl.torso_ang_vel_d_body.setZero();

    // Update reference state trajectory
    x_traj_ref.clear();
    Eigen::Matrix<double, MPC_STATE_DIM, 1> x_ref;
    for (int i = 0; i <= horizon; i++) {
        x_ref.setZero();

        x_ref.block<3, 1>(0, 0) = robot_state.ctrl.torso_pos_d_body;
        x_ref.block<4, 1>(3, 0) = torso_quat_d_vec;
        x_ref.block<3, 1>(7, 0) = robot_state.ctrl.torso_lin_vel_d_body;
        x_ref.block<3, 1>(10, 0) = robot_state.ctrl.torso_ang_vel_d_body;

        x_traj_ref.push_back(x_ref);
    }
}

void QuatMpc::ctrl_update(RobotState &robot_state) {
    // cout << "MPC CTRL UPDATE" << endl;
    /// DYNAMICS ///
    ct_dyn = [&](double *x_dot, const double *x, const double *u) {
        model_ptr->ct_srb_quat_dyn(x_dot, x, u, robot_state.fbk.foot_pos_body,
                                   robot_state.params.robot_inertia, robot_state.params.robot_mass, robot_state.fbk.torso_rot_mat);
    };
    ct_jac = [&](double *jac, const double *x, const double *u) {
        model_ptr->ct_srb_quat_jac(jac, x, u, robot_state.fbk.foot_pos_body,
                                   robot_state.params.robot_inertia, robot_state.params.robot_mass);
    };
    dt_dyn = Utils::midpoint_dyn(MPC_STATE_DIM, MPC_INPUT_DIM, ct_dyn);
    dt_jac = Utils::midpoint_jac(MPC_STATE_DIM, MPC_INPUT_DIM, ct_dyn, ct_jac);

    /// CONSTRAINTS ///
    friction_cone_con = [&](a_float *c, const a_float *x, const a_float *u) {
        (void)x;
        Eigen::Map<Eigen::Matrix<a_float, 6 * 4, 1>> C(c);
        Eigen::Map<const Eigen::VectorXd> u_vec(u, 6);
        C.setZero();
        for (int i = 0; i < 4; i++) {
            Eigen::Matrix<a_float, 6, 1> b_vec;
            b_vec << 0, 0, 0, 0, -2000.0, 0;
            C.segment<6>(i * 6) = C_mat * robot_state.fbk.torso_rot_mat * u_vec.segment<3>(i * 3) + b_vec;
        }
    };
    friction_cone_jac = [&](a_float *jac, const a_float *x, const a_float *u) {
        (void)x;
        (void)u;
        Eigen::Map<Eigen::Matrix<a_float, 6 * 4, MPC_STATE_DIM - 1 + MPC_INPUT_DIM>> J(jac);
        J.setZero();
        for (int i = 0; i < 4; i++) {
            J.block<6, 3>(i * 6, 12 + i * 3) = C_mat * robot_state.fbk.torso_rot_mat;
        }
    };

    /// SETUP ///
    ALTROSolver solver(horizon);
    solver.SetOptions(opts);
    solver.SetDimension(MPC_STATE_DIM, MPC_INPUT_DIM);
    solver.SetExplicitDynamics(dt_dyn, dt_jac);
    solver.SetTimeStep(h);
    for (int i = 0; i <= horizon; i++) {
        solver.SetQuaternionCost(MPC_STATE_DIM, MPC_INPUT_DIM,
                                 robot_state.params.mpc_q_weights.data(), robot_state.params.mpc_r_weights.data(), robot_state.params.mpc_quat_weight,
                                 x_traj_ref.at(i).data(), u_traj_ref.at(i).data(), i, 0);
    }
    // solver.SetConstraint(friction_cone_con, friction_cone_jac, 6 * 4, ConstraintType::INEQUALITY, "friction cone", 0, horizon);
    
    x_init.setZero();
    x_init << 0.0, 0.0, 0.0,
              robot_state.fbk.torso_quat.w(),
              robot_state.fbk.torso_quat.x(),
              robot_state.fbk.torso_quat.y(),
              robot_state.fbk.torso_quat.z(),
              robot_state.fbk.torso_lin_vel_body[0],
              robot_state.fbk.torso_lin_vel_body[1],
              robot_state.fbk.torso_lin_vel_body[2];
              robot_state.fbk.torso_ang_vel_body[0],
              robot_state.fbk.torso_ang_vel_body[1],
              robot_state.fbk.torso_ang_vel_body[2];
    solver.SetInitialState(x_init.data(), MPC_STATE_DIM);
    solver.Initialize();

    // Initial guesses
    for (int i = 0; i <= horizon; i++) {
        solver.SetState(x_traj_ref.at(i).data(), MPC_STATE_DIM, i);
    }
    solver.SetInput(u_traj_ref.at(0).data(), MPC_INPUT_DIM);

    /// SOLVE ///
    solver.Solve();

    /// SAVE ///
    Eigen::VectorXd u(MPC_INPUT_DIM);
    solver.GetInput(u.data(), 0);
    for (int i = 0; i < 4; i++) {
        if (first_iter) {
            u.segment<3>(i * 3) << 0.0, 0.0, 46.0;
            robot_state.ctrl.grf_d.block<3, 1>(i * 3, 0) = u.segment<3>(i * 3);
        } else {
            robot_state.ctrl.grf_d.block<3, 1>(i * 3, 0) = u.segment<3>(i * 3);
        }
    }
    // cout << "grf_d: " << robot_state.ctrl.grf_d.transpose() << endl;
    first_iter = false;
}

}  // namespace mpc
