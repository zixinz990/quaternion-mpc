//
// Created by Zixin Zhang on 01/29/23.
// Copyright (c) 2022 Robotic Exploration Lab. All rights reserved.
//

#include <chrono>
#include <iostream>
#include <filesystem>

#include "Eigen/Dense"
#include "altro/altro_solver.hpp"
#include "altro/solver/solver.hpp"
#include "altro/utils/formatting.hpp"
#include "fmt/core.h"
#include "fmt/chrono.h"
#include "nlohmann/json.hpp"

#include "utils/AltroUtils.h"

using Eigen::MatrixXd;

namespace fs = std::filesystem;
using json = nlohmann::json;

using namespace altro;

int main() {
    auto t_start = std::chrono::high_resolution_clock::now();

    const int n = QuadrupedModel::num_states;
    const int m = QuadrupedModel::num_inputs;
    const int N = 10;
    float h = 0.01;
    float mass = 13;

    ALTROSolver solver(N);

    ///////////////////////////// References /////////////////////////////
    std::vector<Eigen::VectorXd> x_ref;
    std::vector<Eigen::VectorXd> u_ref;

    for (int i = 0; i <= N; i++) {
        Eigen::VectorXd x(n);
        Eigen::VectorXd u(m);

        x << 0.0, 0.0, 0.0, // roll, pitch, yaw
             0.5 * 0.5 * (h*i) * (h*i), 0.0, 0.3, // x, y, z
             0.0, 0.0, 0.0, // ang_vel_x, ang_vel_y, ang_vel_z
             0.5 * (h*i), 0.0, 0.0; // lin_vel_x, lin_vel_y, lin_vel_z
        u << 0.0, 0.0, mass * 9.81 / 4,
             0.0, 0.0, mass * 9.81 / 4,
             0.0, 0.0, mass * 9.81 / 4,
             0.0, 0.0, mass * 9.81 / 4;

        x_ref.emplace_back(x);
        u_ref.emplace_back(u);
    }

    ///////////////////////////// Objective /////////////////////////////
    Eigen::Matrix<a_float, n, 1> Qd;
    Eigen::Matrix<a_float, m, 1> Rd;

    Qd << 1.0, 1.0, 1.0,  // roll, pitch, yaw
          0.0, 0.0, 50.0, // x, y, z
          0.0, 0.0, 1.0,  // ang_vel_x, ang_vel_y, ang_vel_z
          1.0, 1.0, 1.0;  // lin_vel_x, lin_vel_y, lin_vel_z
    
    Rd << 1e-6, 1e-6, 1e-6,  // fx, fy, fz
          1e-6, 1e-6, 1e-6,  // fx, fy, fz
          1e-6, 1e-6, 1e-6,  // fx, fy, fz
          1e-6, 1e-6, 1e-6;  // fx, fy, fz

    ///////////////////////////// Dynamics /////////////////////////////
    Eigen::MatrixXd foot_pos_body(3, 4); // position of feet in body frame
    foot_pos_body << 0.17, 0.17, -0.17, -0.17,
                     0.13, -0.13, 0.13, -0.13,
                     -0.3, -0.3, -0.3, -0.3;
    
    ExplicitDynamicsFunction dyn;
    ExplicitDynamicsJacobian jac;

    auto model_ptr = std::make_shared<QuadrupedModel>();
    ContinuousDynamicsFunction dyn0 = [model_ptr, foot_pos_body](double *x_dot, const double *x, const double *u) {
        model_ptr->ct_srb_dynamics(x_dot, x, u, foot_pos_body);
    };
    ContinuousDynamicsJacobian jac0 = [model_ptr, foot_pos_body](double *jac, const double *x, const double *u) {
        model_ptr->ct_srb_jacobian(jac, x, u, foot_pos_body);
    };

    // dyn = midpoint_dynamics(n, m, dyn0);
    // jac = midpoint_jacobian(n, m, dyn0, jac0);
    dyn = forward_euler_dynamics(n, m, dyn0);
    jac = forward_euler_jacobian(n, m, dyn0, jac0);

    ///////////////////////////// Constraints /////////////////////////////
    auto friction_cone_con = [](a_float *c, const a_float *x, const a_float *u) {
        (void)x;
        float mu = 0.5;
        float fz_max = 1000;
        float contacts[4] = {1.0, 0.0, 0.0, 1.0};  // FL, FR, RL, RR

        for (int i = 0; i < 4; i++) {
            c[0 + i * 6] = u[0 + i * 3] - mu * u[2 + i * 3]; // fx - mu*fz <= 0
            c[1 + i * 6] = -u[0 + i * 3] - mu * u[2 + i * 3]; // -fx - mu*fz <= 0
            c[2 + i * 6] = u[1 + i * 3] - mu * u[2 + i * 3]; // fy - mu*fz <= 0
            c[3 + i * 6] = -u[1 + i * 3] - mu * u[2 + i * 3]; // -fy - mu*fz <= 0
            c[4 + i * 6] = u[2 + i * 3] - fz_max * contacts[i]; // fz <= fz_max
            c[5 + i * 6] = -u[2 + i * 3]; // -fz <= 0
        }
    };

    auto friction_cone_jac = [](a_float *jac, const a_float *x, const a_float *u) {
        (void)x;
        (void)u;
        Eigen::Map<Eigen::Matrix<a_float, 24, 24>> J(jac);
        J.setZero();

        float mu = 0.5;
        float fz_max = 1000;

        for (int i = 0; i < 4; i++) {
            J(0 + i * 6, 12 + i * 3) = 1; // dc0/dfx
            J(0 + i * 6, 14 + i * 3) = -mu; // dc0/dfz
            J(1 + i * 6, 12 + i * 3) = -1; // dc1/dfx
            J(1 + i * 6, 14 + i * 3) = -mu; // dc1/dfz
            J(2 + i * 6, 13 + i * 3) = 1; // dc2/dfy
            J(2 + i * 6, 14 + i * 3) = -mu; // dc2/dfz
            J(3 + i * 6, 13 + i * 3) = -1; // dc3/dfy
            J(3 + i * 6, 14 + i * 3) = -mu; // dc3/dfz
            J(4 + i * 6, 14 + i * 3) = 1; // dc4/dfz
            J(5 + i * 6, 14 + i * 3) = -1; // dc5/dfz
        }
    };

    ///////////////////////////// Setup /////////////////////////////
    solver.SetDimension(n, m);             // Set Dimension
    solver.SetExplicitDynamics(dyn, jac);  // Set Dynamics
    solver.SetTimeStep(h);                 // Set Time Step
    // Set cost
    for (int k = 0; k <= N; k++) {
        solver.SetLQRCost(n, m, Qd.data(), Rd.data(), x_ref.at(k).data(), u_ref.at(k).data(), k);
    }
    // Set constraints
    solver.SetConstraint(friction_cone_con, friction_cone_jac, 24, ConstraintType::INEQUALITY, "friction cone", 0, N + 1);
    // Set initial state
    solver.SetInitialState(x_ref[0].data(), n);
    // Initialize solver
    solver.Initialize();
    // Set initial state guess
    for (int k = 0; k <= N; k++) {
        solver.SetState(x_ref.at(k).data(), n, k);
    }
    // Set initial input guess
    solver.SetInput(u_ref.at(0).data(), m);

    ///////////////////////////// Solve /////////////////////////////
    fmt::print("#############################################\n");
    fmt::print("                 MPC Solve\n");
    fmt::print("#############################################\n");

    AltroOptions opts;
    opts.verbose = Verbosity::Silent;
    opts.iterations_max = 1;
    opts.use_backtracking_linesearch = true;
    solver.SetOptions(opts);

    SolveStatus status;
    status = solver.Solve();

    auto t_end = std::chrono::high_resolution_clock::now();
    using SecondsDouble = std::chrono::duration<double, std::ratio<1>>;
    SecondsDouble t_total = std::chrono::duration_cast<SecondsDouble>(t_end - t_start);
    fmt::print("Total time = {} ms\n", t_total * 1000);

    ///////////////////////////// Save Results /////////////////////////////
    std::vector<Vector> x_sim;
    std::vector<Vector> u_sim;

    for (int k = 0; k < N; k++) {
        Eigen::VectorXd x(n);
        Eigen::VectorXd u(m);
        solver.GetState(x.data(), k);
        solver.GetInput(u.data(), k);
        x_sim.emplace_back(x);
        u_sim.emplace_back(u);
    }

    // Save trajectory to JSON file
    fs::path out_file = "/home/REXOperator/legged_mpc_ctrl_ws/src/legged_ctrl/src/test/test_altro/convex_mpc.json";
    std::ofstream traj_out(out_file);
    json x_ref_data(x_ref);
    json x_data(x_sim);
    json u_data(u_sim);
    json data;
    data["reference_trajectory"] = x_ref_data;
    data["state_trajectory"] = x_data;
    data["input_trajectory"] = u_data;
    traj_out << std::setw(4) << data;
}
