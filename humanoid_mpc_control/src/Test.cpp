#include <iostream>
#include <vector>
#include <Eigen/Dense>

#include "HumanoidModel.h"
#include "RobotState.h"
#include "Utils.h"
#include "altro/altro_solver.hpp"
#include "altro/solver/solver.hpp"

using namespace altro;
using namespace mpc;

int main() {
    // // Reference trajectory
    // double h = 0.01;
    // int horizon = 20;
    // double robot_mass = 24.267546;
    // Eigen::Matrix3d robot_inertia;
    // robot_inertia << 0.168459, 0.000124, 0.006493,
    //                  0.000124, 0.101358, 0.000278,
    //                  0.006493, 0.000278, 0.091754;
    // Eigen::Matrix3d torso_rot_mat = Eigen::Matrix3d::Identity();
    // Eigen::Matrix<double, 3, 4> foot_pos_body; // left toe, left_heel, right_toe, right_heel
    // foot_pos_body << 0.0425, -0.0425, 0.0425, -0.0425,
    //                  0.08, 0.08, -0.08, -0.08,
    //                  -0.85, -0.85, -0.85, -0.85;
    // double mu = 0.7;

    // std::vector<Eigen::Matrix<double, MPC_STATE_DIM, 1>> x_traj_ref;
    // std::vector<Eigen::Matrix<double, MPC_INPUT_DIM, 1>> u_traj_ref;

    // Eigen::Matrix<double, MPC_STATE_DIM, 1> x_ref;
    // Eigen::Matrix<double, MPC_INPUT_DIM, 1> u_ref;

    // u_ref << 0, 0, robot_mass * 9.81 / 4,
    //          0, 0, robot_mass * 9.81 / 4,
    //          0, 0, robot_mass * 9.81 / 4,
    //          0, 0, robot_mass * 9.81 / 4;
    
    // // std::cout << "u_ref: " << u_ref.transpose() << std::endl;

    // for (int i = 0; i <= horizon; i++) {
    //     x_ref.setZero();

    //     x_ref.block<3, 1>(0, 0) = Eigen::Vector3d(0, 0, 0);
    //     x_ref.block<4, 1>(3, 0) = Eigen::Vector4d(1, 0, 0, 0);
    //     x_ref.block<3, 1>(7, 0) = Eigen::Vector3d(0, 0, 0);
    //     x_ref.block<3, 1>(10, 0) = Eigen::Vector3d(0, 0, 0);

    //     x_traj_ref.emplace_back(x_ref);
    //     u_traj_ref.emplace_back(u_ref);
    // }

    // // Dynamics
    // auto model_ptr = std::make_shared<HumanoidModel>();
    // ContinuousDynamicsFunction ct_dyn;  // continuous time dynamics function
    // ContinuousDynamicsJacobian ct_jac;  // continuous time dynamics jacobian
    // ExplicitDynamicsFunction dt_dyn;    // discrete time dynamics function
    // ExplicitDynamicsJacobian dt_jac;    // discrete time dynamics jacobian

    // ct_dyn = [&](double *x_dot, const double *x, const double *u) {
    //     model_ptr->ct_srb_quat_dyn(x_dot, x, u, foot_pos_body,
    //                                robot_inertia, robot_mass, torso_rot_mat);
    // };
    // ct_jac = [&](double *jac, const double *x, const double *u) {
    //     model_ptr->ct_srb_quat_jac(jac, x, u, foot_pos_body,
    //                                robot_inertia, robot_mass);
    // };
    // dt_dyn = Utils::midpoint_dyn(MPC_STATE_DIM, MPC_INPUT_DIM, ct_dyn);
    // dt_jac = Utils::midpoint_jac(MPC_STATE_DIM, MPC_INPUT_DIM, ct_dyn, ct_jac);

    // // Constraints
    // Eigen::Matrix<double, 6, 3> C_mat;     // C matrix about friction cone
    // ConstraintFunction friction_cone_con;  // friction cone constraint function
    // ConstraintJacobian friction_cone_jac;  // friction cone constraint jacobian

    // C_mat << 1, 0, -mu,   //  fx - mu * fz <= 0
    //          -1, 0, -mu,  // -fx - mu * fz <= 0
    //          0, 1, -mu,   //  fy - mu * fz <= 0
    //          0, -1, -mu,  // -fy - mu * fz <= 0
    //          0, 0, 1,     //  fz - fz_max  <= 0
    //          0, 0, -1;    // -fz           <= 0

    // friction_cone_con = [&](a_float *c, const a_float *x, const a_float *u) {
    //     (void)x;
    //     Eigen::Map<Eigen::Matrix<a_float, 6 * 4, 1>> C(c);
    //     Eigen::Map<const Eigen::VectorXd> u_vec(u, MPC_INPUT_DIM);
    //     C.setZero();
    //     for (int i = 0; i < 4; i++) {
    //         Eigen::Matrix<a_float, 6, 1> b_vec;
    //         b_vec << 0, 0, 0, 0, -500.0, 0;
    //         C.segment<6>(i * 6) = C_mat * torso_rot_mat * u_vec.segment<3>(i * 3) + b_vec;
    //     }
    // };
    // friction_cone_jac = [&](a_float *jac, const a_float *x, const a_float *u) {
    //     (void)x;
    //     (void)u;
    //     Eigen::Map<Eigen::Matrix<a_float, 6 * 4, MPC_STATE_DIM - 1 + MPC_INPUT_DIM>> J(jac);
    //     J.setZero();
    //     for (int i = 0; i < 4; i++) {
    //         J.block<6, 3>(i * 6, 12 + i * 3) = C_mat * torso_rot_mat;
    //     }
    // };

    // // Setup
    // ALTROSolver solver(horizon);

    // AltroOptions opts;
    // opts.verbose = Verbosity::Inner;
    // opts.iterations_max = 20;
    // opts.use_backtracking_linesearch = true;
    // opts.use_quaternion = true;
    // opts.quat_start_index = 3;
    // opts.penalty_scaling = 10.0;

    // solver.SetOptions(opts);
    // solver.SetDimension(MPC_STATE_DIM, MPC_INPUT_DIM);
    // solver.SetExplicitDynamics(dt_dyn, dt_jac);
    // solver.SetTimeStep(h);

    // Eigen::Matrix<double, MPC_STATE_DIM, 1> mpc_q_weights;
    // Eigen::Matrix<double, MPC_INPUT_DIM, 1> mpc_r_weights;
    // double mpc_quat_weight;

    // mpc_q_weights << 1.0, 1.0, 1.0,
    //                  0.0, 0.0, 0.0, 0.0,
    //                  1.0, 1.0, 1.0,
    //                  1.0, 1.0, 1.0;
    // mpc_r_weights << 0.000001, 0.000001, 0.000001, 0.000001, 0.000001, 0.000001, 0.000001, 0.000001, 0.000001, 0.000001, 0.000001, 0.000001;
    // mpc_quat_weight = 1.0;

    // for (int i = 0; i <= horizon; i++) {
    //     solver.SetQuaternionCost(MPC_STATE_DIM, MPC_INPUT_DIM,
    //                              mpc_q_weights.data(), mpc_r_weights.data(), mpc_quat_weight,
    //                              x_traj_ref.at(i).data(), u_traj_ref.at(i).data(), i, 0);
    // }
    // solver.SetConstraint(friction_cone_con, friction_cone_jac, 24, ConstraintType::INEQUALITY, "friction cone", 0, horizon);
    
    // Eigen::Matrix<double, MPC_STATE_DIM, 1> x_init;
    // x_init.setZero();
    // x_init << 0.0, 0.0, 0.0,
    //           1.0, 0.0, 0.0, 0.0,
    //           0.0, 0.0, 0.0,
    //           0.0, 0.0, 0.0;
    // solver.SetInitialState(x_init.data(), MPC_STATE_DIM);
    // solver.Initialize();

    // // Initial guesses
    // for (int i = 0; i <= horizon; i++) {
    //     solver.SetState(x_traj_ref.at(i).data(), MPC_STATE_DIM, i);
    // }
    // solver.SetInput(u_traj_ref.at(0).data(), MPC_INPUT_DIM);

    // /// SOLVE ///
    // solver.Solve();
    // // Eigen::VectorXd u(MPC_INPUT_DIM);
    // // solver.GetInput(u.data(), 0);
    // // std::cout << "u: " << u.transpose() << std::endl;

    // // return 0;

    Eigen::Matrix<double, 3, 5> mat;
    mat << 1, 2, 3, 4, 5,
           2, 4, 6, 8, 10,
           3, 6, 9, 12, 15;
    Eigen::FullPivLU<Eigen::Matrix<double, 3, 5>> lu_decomp(mat);
    auto rank = lu_decomp.rank();
    std::cout << "Rank: " << rank << std::endl;
}