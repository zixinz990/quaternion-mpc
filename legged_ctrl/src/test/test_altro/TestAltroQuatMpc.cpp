//
// Created by Zixin Zhang on 3/28/23.
// Copyright (c) 2023 Robotic Exploration Lab. All rights reserved.
//

#include <chrono>
#include <filesystem>
#include <iostream>

#include "Eigen/Dense"

#include "altro/altro_solver.hpp"
#include "altro/solver/solver.hpp"

#include "fmt/chrono.h"
#include "fmt/core.h"

#include "nlohmann/json.hpp"

#include "utils/AltroUtils.h"

using Eigen::MatrixXd;

namespace fs = std::filesystem;
using json = nlohmann::json;

using namespace altro;

int main() {
    // r_desired:  -4.7389e-05 -0.000245138     0.241745
    // q_desired: 1 0 0 0
    // v_desired: 0 0 0
    // w_desired: 0 0 0
    // foot_pos_body
    // 0.189665   0.201369  -0.159653  -0.151781
    // 0.164306  -0.162737   0.162824  -0.199366
    // -0.0420698 -0.0422378 -0.0420083 -0.0432552
    // x_init =            0            0            0     0.999048 -0.000500345  0.000120345   -0.0436295   0.00116501   0.00349211   -0.0268339   -0.0648412    0.0192918 -0.000137382

    auto t_start = std::chrono::high_resolution_clock::now();
    const int n = 13;
    const int en = 12;
    const int m = 12;
    const int em = 12;
    const int N = 30;
    const double h = 0.01;
    Eigen::Matrix<double, 3, 4> foot_pos_body;
    Eigen::Matrix<double, 3, 3> inertia_body;
    foot_pos_body << 0.189665, 0.201369, -0.159653, -0.151781,
                     0.164306, -0.162737, 0.162824, -0.199366,
                     -0.0420698, -0.0422378, -0.0420083, -0.0432552;
    inertia_body << 0.0158533, 0.0, 0.0, 0.0, 0.0377999, 0.0, 0.0, 0.0, 0.0456542;
    ALTROSolver solver(N);

    /// REFERENCES ///
    // Try some simple reference
    std::vector<Eigen::VectorXd> X_ref;
    std::vector<Eigen::VectorXd> U_ref;

    for (int i = 0; i <= N; i++) {
        Vector x_ref = Vector::Zero(n);
        Vector u_ref = Vector::Zero(m);

        x_ref << -4.7389e-05, -0.000245138, 0.241745,
                 1.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0;
        u_ref << 0.0, 0.0, 13 * 9.81 / 4,
                 0.0, 0.0, 13 * 9.81 / 4,
                 0.0, 0.0, 13 * 9.81 / 4,
                 0.0, 0.0, 13 * 9.81 / 4;

        X_ref.emplace_back(x_ref);
        U_ref.emplace_back(u_ref);
    }

    /// OBJECTIVE ///
    Eigen::Matrix<double, n, 1> Qd;
    Eigen::Matrix<double, m, 1> Rd;
    //  double w = 1.0;
    //
    //  Qd << 1.0, 1.0, 1.0,
    //        0, 0, 0, 0,
    //        10.0, 10.0, 10.0,
    //        10.0, 10.0, 10.0;

    double w = 100.0;
    Qd << 0.0, 0.0, 100.0,      // only track z position
            0.0, 0.0, 0.0, 0.0,  // ignore quaternion in Q
            1.0, 10.0, 1.0,       // track linear velocity
            1.0, 1.0, 1.0;       // track angular velocity
    Rd << 1e-6, 1e-6, 1e-6,
            1e-6, 1e-6, 1e-6,
            1e-6, 1e-6, 1e-6,
            1e-6, 1e-6, 1e-6;

    /// DYNAMICS ///
    auto model_ptr = std::make_shared<QuadrupedModel>();
    ContinuousDynamicsFunction ct_dyn = [model_ptr, foot_pos_body, inertia_body](double *x_dot, const double *x, const double *u) {
        model_ptr->ct_srb_quat_dynamics(x_dot, x, u, foot_pos_body, inertia_body);
    };
    ContinuousDynamicsJacobian ct_jac = [model_ptr, foot_pos_body, inertia_body](double *jac, const double *x, const double *u) {
        model_ptr->ct_srb_quat_jacobian(jac, x, u, foot_pos_body, inertia_body);
    };
    ExplicitDynamicsFunction dt_dyn = midpoint_dynamics(n, m, ct_dyn);
    ExplicitDynamicsJacobian dt_jac = midpoint_jacobian(n, m, ct_dyn, ct_jac);

    /// CONSTRAINTS ///
    float contacts[4] = {1.0, 1.0, 1.0, 1.0};  // FL, FR, RL, RR
    float mu = 0.7;
    float fz_max = 666;
    float fz_min = 5;
    auto friction_cone_con = [contacts, mu, fz_max, fz_min](a_float *c, const a_float *x, const a_float *u) {
        (void)x;
        for (int i = 0; i < 4; i++) {
        c[0 + i * 6] = u[0 + i * 3] - mu * u[2 + i * 3];      // fx - mu*fz <= 0
        c[1 + i * 6] = -u[0 + i * 3] - mu * u[2 + i * 3];     // -fx - mu*fz <= 0
        c[2 + i * 6] = u[1 + i * 3] - mu * u[2 + i * 3];      // fy - mu*fz <= 0
        c[3 + i * 6] = -u[1 + i * 3] - mu * u[2 + i * 3];     // -fy - mu*fz <= 0
        c[4 + i * 6] = u[2 + i * 3] - fz_max * contacts[i];   // fz <= fz_max
        c[5 + i * 6] = -u[2 + i * 3] + fz_min * contacts[i];  // -fz + 5 <= 0
        }
    };

    auto friction_cone_jac = [mu](a_float *jac, const a_float *x, const a_float *u) {
        (void)x;
        (void)u;
        Eigen::Map<Eigen::Matrix<a_float, 24, 24>> J(jac);
        J.setZero();

        for (int i = 0; i < 4; i++) {
        J(0 + i * 6, 12 + i * 3) = 1;    // dc0/dfx
        J(0 + i * 6, 14 + i * 3) = -mu;  // dc0/dfz
        J(1 + i * 6, 12 + i * 3) = -1;   // dc1/dfx
        J(1 + i * 6, 14 + i * 3) = -mu;  // dc1/dfz
        J(2 + i * 6, 13 + i * 3) = 1;    // dc2/dfy
        J(2 + i * 6, 14 + i * 3) = -mu;  // dc2/dfz
        J(3 + i * 6, 13 + i * 3) = -1;   // dc3/dfy
        J(3 + i * 6, 14 + i * 3) = -mu;  // dc3/dfz
        J(4 + i * 6, 14 + i * 3) = 1;    // dc4/dfz
        J(5 + i * 6, 14 + i * 3) = -1;   // dc5/dfz
        }
    };

    /// SETUP ///
    AltroOptions opts;
    opts.verbose = Verbosity::Inner;
    opts.iterations_max = 10;
    opts.use_backtracking_linesearch = true;
    opts.use_quaternion = true;
    opts.quat_start_index = 3;
    solver.SetOptions(opts);

    solver.SetDimension(n, m);
    solver.SetErrorDimension(en, em);
    solver.SetExplicitDynamics(dt_dyn, dt_jac);
    solver.SetTimeStep(h);

    // Cost function
    for (int i = 0; i <= N; i++) {
        solver.SetQuaternionCost(n, m, Qd.data(), Rd.data(), w, X_ref.at(i).data(), U_ref.at(i).data(),
                                i, 0);
    }

    solver.SetConstraint(friction_cone_con, friction_cone_jac, 24, ConstraintType::INEQUALITY, "friction cone", 0, N + 1);

    Vector x_init = Vector::Zero(n);
    x_init << 0, 0, 0,
              0.999048, -0.000500345, 0.000120345, -0.0436295,
              0.00116501, 0.00349211, -0.0268339,
              -0.0648412, 0.0192918, -0.000137382;
    solver.SetInitialState(x_init.data(), n);
    solver.Initialize();

    // Initial guesses
    for (int i = 0; i <= N; i++) {
        solver.SetState(X_ref.at(i).data(), n, i);
    }
    solver.SetInput(U_ref.at(0).data(), m);

    /// SOLVE ///
    fmt::print("#############################################\n");
    fmt::print("                 MPC Solve\n");
    fmt::print("#############################################\n");

    solver.Solve();
    auto t_end = std::chrono::high_resolution_clock::now();
    using SecondsDouble = std::chrono::duration<double, std::ratio<1>>;
    SecondsDouble t_total = std::chrono::duration_cast<SecondsDouble>(t_end - t_start);
    fmt::print("Total time = {} ms\n", t_total * 1000);

    /// SAVE ///
    std::vector<Vector> X_sim;
    std::vector<Vector> U_sim;

    for (int k = 0; k <= N; k++) {
        Eigen::VectorXd x(n);
        solver.GetState(x.data(), k);
        X_sim.emplace_back(x);
        if (k != N) {
        Eigen::VectorXd u(m);
        solver.GetInput(u.data(), k);
        U_sim.emplace_back(u);
        }
    }

    // Save trajectory to JSON file
    std::filesystem::path out_file = "/home/REXOperator/legged_mpc_ctrl_ws/src/legged_ctrl/src/test/test_altro/quadruped_quat_test.json";
    std::ofstream traj_out(out_file);
    json X_ref_data(X_ref);
    json U_ref_data(U_ref);
    json X_data(X_sim);
    json U_data(U_sim);
    json data;
    data["reference_state"] = X_ref_data;
    data["reference_input"] = U_ref_data;
    data["state_trajectory"] = X_data;
    data["input_trajectory"] = U_data;
    traj_out << std::setw(4) << data;

    // Calculate tracking error
    double tracking_err = 0;
    for (int i = 0; i < N; i++) {
        tracking_err =
            tracking_err + (X_sim.at(i) - X_ref.at(i)).transpose() * (X_sim.at(i) - X_ref.at(i));
    }
}