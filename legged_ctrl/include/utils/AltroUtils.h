//
// Created by Zixin Zhang on 01/25/23.
// Copyright (c) 2022 Robotic Exploration Lab. All rights reserved.
//

#pragma once

#include <iostream>
#include <vector>

#include "LeggedParams.h"
#include "LeggedState.h"
#include "Eigen/Dense"
#include "altro/solver/typedefs.hpp"
#include "altro/utils/formatting.hpp"

#include "utils/Utils.h"
#include "utils/QuaternionUtils.h"

using ContinuousDynamicsFunction = std::function<void(double *, const double *, const double *)>;
using ContinuousDynamicsJacobian = std::function<void(double *, const double *, const double *)>;

altro::ExplicitDynamicsFunction midpoint_dynamics(int n, int m, ContinuousDynamicsFunction f);
altro::ExplicitDynamicsJacobian midpoint_jacobian(int n, int m, ContinuousDynamicsFunction f, ContinuousDynamicsJacobian jac);

altro::ExplicitDynamicsFunction forward_euler_dynamics(int n, int m, ContinuousDynamicsFunction f);
altro::ExplicitDynamicsJacobian forward_euler_jacobian(int n, int m, ContinuousDynamicsFunction f, ContinuousDynamicsJacobian jac);

altro::ExplicitDynamicsFunction discrete_error_dynamics(int n, int m,
                                                      Eigen::Matrix<double, 13, 1> x_ref_, Eigen::Matrix<double, 12, 1> u_ref_,
                                                      Eigen::Matrix<double, 13, 1> x_ref_next_,
                                                      ContinuousDynamicsFunction f,
                                                      ContinuousDynamicsJacobian df);
altro::ExplicitDynamicsJacobian discrete_error_jacobian(int n, int m,
                                                      Eigen::Matrix<double, 13, 1> x_ref_, Eigen::Matrix<double, 12, 1> u_ref_,
                                                      Eigen::Matrix<double, 13, 1> x_ref_next_,
                                                      ContinuousDynamicsFunction f,
                                                      ContinuousDynamicsJacobian df);

class QuadrupedModel {
public:
    QuadrupedModel() {};

    void ct_srb_dynamics(double *x_dot, const double *x, const double *u, Eigen::Matrix<double, 3, NUM_LEG> foot_pos) const;

    void ct_srb_jacobian(double *jac, const double *x, const double *u, Eigen::Matrix<double, 3, NUM_LEG> foot_pos) const;

    void ct_srb_quat_dynamics(double *x_dot, const double *x, const double *u, Eigen::Matrix<double, 3, NUM_LEG> foot_pos_body, Eigen::Matrix3d robot_inertia, double robot_mass, Eigen::Matrix3d torso_rot_mat) const;

    void ct_srb_quat_jacobian(double *jac, const double *x, const double *u, Eigen::Matrix<double, 3, NUM_LEG> foot_pos_body, Eigen::Matrix3d robot_inertia, double robot_mass) const;

    void ct_srb_trot_quat_dynamics(double *x_dot, const double *x, const double *u, Eigen::Matrix<double, 3, 2> foot_pos_body, Eigen::Matrix3d robot_inertia, double robot_mass) const;

    void ct_srb_trot_quat_jacobian(double *jac, const double *x, const double *u, Eigen::Matrix<double, 3, 2> foot_pos_body, Eigen::Matrix3d robot_inertia, double robot_mass) const;

    static constexpr int num_states = 13;
    static constexpr int num_inputs = 12;
};
