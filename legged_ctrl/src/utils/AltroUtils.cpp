//
// Created by Zixin Zhang on 01/25/23.
// Copyright (c) 2022 Robotic Exploration Lab. All rights reserved.
//

#include "utils/AltroUtils.h"

// THIS FUNCTION SHOULD BE GOOD
altro::ExplicitDynamicsFunction midpoint_dynamics(int n, int m, ContinuousDynamicsFunction f) {
    auto fd = [n, m, f](double *xn, const double *x, const double *u, float h) {
        static Eigen::VectorXd xm(n);
        Eigen::Map<Eigen::VectorXd> xn_vec(xn, n);
        Eigen::Map<const Eigen::VectorXd> x_vec(x, n);
        Eigen::Map<const Eigen::VectorXd> u_vec(u, n);
        f(xm.data(), x, u);
        xm *= h / 2;
        xm.noalias() += x_vec;
        f(xn, xm.data(), u);
        xn_vec = x_vec + h * xn_vec;
    };
    return fd;
}

altro::ExplicitDynamicsFunction forward_euler_dynamics(int n, int m, ContinuousDynamicsFunction f) {
    auto fd = [n, m, f](double *xn, const double *x, const double *u, float h) {
        Eigen::Map <Eigen::VectorXd> xn_vec(xn, n);
        Eigen::Map<const Eigen::VectorXd> x_vec(x, n);
        Eigen::Map<const Eigen::VectorXd> u_vec(u, n);
        f(xn, x, u); // xn is actually x_dot here
        xn_vec = x_vec + h * xn_vec;
    };
    return fd;
}

// THIS FUNCTION SHOULD BE GOOD
// altro::ExplicitDynamicsJacobian midpoint_jacobian(int n, int m, ContinuousDynamicsFunction f, ContinuousDynamicsJacobian df) {
//     auto fd = [n, m, f, df](double *jac, const double *x, const double *u, float h) {
//         Eigen::Map <Eigen::MatrixXd> J(jac, 13, 25);
//         Eigen::Map<const Eigen::VectorXd> x_vec(x, 13);
//         Eigen::Map<const Eigen::VectorXd> u_vec(u, 12);

//         Eigen::MatrixXd x_dot(13, 1);
//         Eigen::MatrixXd x_mid(13, 1);
//         Eigen::MatrixXd ct_jac(13, 25);
//         Eigen::MatrixXd ct_jac_mid(13, 25);
//         Eigen::MatrixXd Am(13, 13);
//         Eigen::MatrixXd Bm(13, 12);
//         Eigen::MatrixXd A(13, 13);
//         Eigen::MatrixXd B(13, 12);

//         x_dot.setZero();
//         x_mid.setZero();
//         ct_jac.setZero();
//         ct_jac_mid.setZero();
//         Am.setZero();
//         Bm.setZero();
//         A.setZero();
//         B.setZero();

//         f(x_dot.data(), x, u); // calculate x_dot
//         x_mid = x_vec + h / 2 * x_dot; // calculate x_mid
//         df(ct_jac_mid.data(), x_mid.data(), u); // calculate ct_jac_mid
//         df(ct_jac.data(), x, u); // calculate ct_jac

//         Am = ct_jac_mid.leftCols(13);
//         Bm = ct_jac_mid.rightCols(12);

//         A = ct_jac.leftCols(13);
//         B = ct_jac.rightCols(12);

//         J.leftCols(13) = Eigen::MatrixXd::Identity(13, 13) + h * Am * (Eigen::MatrixXd::Identity(13, 13) + h/2 * A);
//         J.rightCols(12) = h * (Am * h/2 * B + Bm);
//     };
//     return fd;
// }

// Brian's code
altro::ExplicitDynamicsJacobian midpoint_jacobian(int n, int m, ContinuousDynamicsFunction f, ContinuousDynamicsJacobian df) {
  auto fd = [n, m, f, df](double *jac, const double *x, const double *u, float h) {
    static Eigen::MatrixXd A(n, n);
    static Eigen::MatrixXd B(n, m);
    static Eigen::MatrixXd Am(n, n);
    static Eigen::MatrixXd Bm(n, m);
    static Eigen::VectorXd xm(n);
    static Eigen::MatrixXd In = Eigen::MatrixXd::Identity(n, n);

    Eigen::Map<Eigen::MatrixXd> J(jac, n, n + m);
    Eigen::Map<const Eigen::VectorXd> x_vec(x, n);
    Eigen::Map<const Eigen::VectorXd> u_vec(u, n);

    // Evaluate the midpoint
    f(xm.data(), x, u);
    xm = x_vec + h / 2 * xm;

    // Evaluate the Jacobian
    df(J.data(), x, u);
    A = J.leftCols(n);
    B = J.rightCols(m);

    // Evaluate the Jacobian at the midpoint
    df(J.data(), xm.data(), u);
    Am = J.leftCols(n);
    Bm = J.rightCols(m);

    // Apply the chain rule
    J.leftCols(n) = In + h * Am * (In + h / 2 * A);
    J.rightCols(m) = h * (Am * h / 2 * B + Bm);
  };
  return fd;
}

altro::ExplicitDynamicsJacobian forward_euler_jacobian(int n, int m, ContinuousDynamicsFunction f, ContinuousDynamicsJacobian df) {
    auto fd = [n, m, f, df](double *jac, const double *x, const double *u, float h) {
        Eigen::Map <Eigen::MatrixXd> J(jac, n, n + m);
        Eigen::Map<const Eigen::VectorXd> x_vec(x, n);
        Eigen::Map<const Eigen::VectorXd> u_vec(u, n);

        static Eigen::MatrixXd In = Eigen::MatrixXd::Identity(n, n);

        df(J.data(), x, u);
        J.leftCols(n) = In + h * J.leftCols(n);
        J.rightCols(m) = h * J.rightCols(m);
    };
    return fd;
}

// THIS FUNCTION SHOULD BE GOOD
altro::ExplicitDynamicsFunction discrete_error_dynamics(int n, int m,
                                                        Eigen::Matrix<double, 13, 1> x_ref_, Eigen::Matrix<double, 12, 1> u_ref_,
                                                        Eigen::Matrix<double, 13, 1> x_ref_next_,
                                                        ContinuousDynamicsFunction f,
                                                        ContinuousDynamicsJacobian df) {
    // For discrete error dynamics, n = 12, m = 12
    auto fd = [n, m, f, df, x_ref_, u_ref_, x_ref_next_](double *ex_next, const double *ex, const double *eu, float h) {
        // Map the variables to Eigen vectors
        Eigen::Map<Eigen::VectorXd> ex_next_vec(ex_next, n);
        Eigen::Map<const Eigen::VectorXd> ex_vec(ex, n);
        Eigen::Map<const Eigen::VectorXd> eu_vec(eu, m);

        // Some initializations
        Eigen::MatrixXd A(12, 12);
        Eigen::MatrixXd B(12, 12);
        Eigen::Matrix<double, 13, 12> E_ref;
        Eigen::Matrix<double, 13, 12> E_ref_next;
        Eigen::MatrixXd J_ref(13, 25);

        A.setZero();
        B.setZero();
        E_ref.setZero();
        E_ref_next.setZero();
        J_ref.setZero();

        // Calculate error-state Jacobian E (13x12) at reference
        E_ref.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
        E_ref.block<4, 3>(3, 3) = QuaternionUtils::G(x_ref_.segment<4>(3));
        E_ref.block<3, 3>(7, 6) = Eigen::Matrix3d::Identity();
        E_ref.block<3, 3>(10, 9) = Eigen::Matrix3d::Identity();

        E_ref_next.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
        E_ref_next.block<4, 3>(3, 3) = QuaternionUtils::G(x_ref_next_.segment<4>(3));
        E_ref_next.block<3, 3>(7, 6) = Eigen::Matrix3d::Identity();
        E_ref_next.block<3, 3>(10, 9) = Eigen::Matrix3d::Identity();

        // Calculate Jacobian of discrete dynamics at (x_ref_, u_ref_)
        midpoint_jacobian(n, m, f, df)(J_ref.data(), x_ref_.data(), u_ref_.data(), h); // get midpoint discrete dynamics Jacobian
        
        A = E_ref_next.transpose() * J_ref.block<13, 13>(0, 0) * E_ref;
        B = E_ref_next.transpose() * J_ref.block<13, 12>(0, 13);
        
        ex_next_vec = A * ex_vec + B * eu_vec;
    };
    return fd;
}

altro::ExplicitDynamicsJacobian discrete_error_jacobian(int n, int m,
                                                      Eigen::Matrix<double, 13, 1> x_ref_, Eigen::Matrix<double, 12, 1> u_ref_,
                                                      Eigen::Matrix<double, 13, 1> x_ref_next_,
                                                      ContinuousDynamicsFunction f,
                                                      ContinuousDynamicsJacobian df) {
    // For discrete error dynamics, n = 12, m = 12
    auto fd = [n, m, x_ref_, u_ref_, x_ref_next_, f, df](double *jac, const double *ex, const double *eu, float h) {
        // Map the variables to Eigen vectors
        Eigen::Map <Eigen::MatrixXd> J(jac, n, n + m);
        Eigen::Map<const Eigen::VectorXd> ex_vec(ex, n);
        Eigen::Map<const Eigen::VectorXd> eu_vec(eu, m);

        // Some initializations
        Eigen::MatrixXd A(12, 12);
        Eigen::MatrixXd B(12, 12);
        Eigen::Matrix<double, 13, 12> E_ref;
        Eigen::Matrix<double, 13, 12> E_ref_next;
        Eigen::MatrixXd J_ref(13, 25);

        A.setZero();
        B.setZero();
        E_ref.setZero();
        E_ref_next.setZero();
        J_ref.setZero();

        // Calculate error-state Jacobian E (13x12) at reference
        E_ref.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
        E_ref.block<4, 3>(3, 3) = QuaternionUtils::G(x_ref_.segment<4>(3));
        E_ref.block<3, 3>(7, 6) = Eigen::Matrix3d::Identity();
        E_ref.block<3, 3>(10, 9) = Eigen::Matrix3d::Identity();

        E_ref_next.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
        E_ref_next.block<4, 3>(3, 3) = QuaternionUtils::G(x_ref_next_.segment<4>(3));
        E_ref_next.block<3, 3>(7, 6) = Eigen::Matrix3d::Identity();
        E_ref_next.block<3, 3>(10, 9) = Eigen::Matrix3d::Identity();

        // Calculate Jacobian of discrete dynamics at (x_ref_, u_ref_)
        midpoint_jacobian(n, m, f, df)(J_ref.data(), x_ref_.data(), u_ref_.data(), h); // get midpoint discrete dynamics Jacobian
        
        A = E_ref_next.transpose() * J_ref.block<13, 13>(0, 0) * E_ref;
        B = E_ref_next.transpose() * J_ref.block<13, 12>(0, 13);

        J.block<12, 12>(0, 0) = A;
        J.block<12, 12>(0, 12) = B;
    };
    return fd;
}

// Single rigid body dynamics of a quadruped robot
void QuadrupedModel::ct_srb_dynamics(double *x_dot, const double *x, const double *u, Eigen::Matrix<double, 3, NUM_LEG> foot_pos) const {
    // Map the variables to Eigen vectors
    Eigen::Map <Eigen::VectorXd> x_dot_vec(x_dot, num_states);
    Eigen::Map<const Eigen::VectorXd> x_vec(x, num_states);
    Eigen::Map<const Eigen::VectorXd> u_vec(u, num_inputs);

    // Initialize some variables
    Eigen::VectorXd g_vec;
    g_vec.resize(num_states);
    g_vec << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -9.81;

    Eigen::Matrix<double, 12, 12> Ac;
    Eigen::Matrix<double, 12, 12> Bc;
    Ac.setZero();
    Bc.setZero();
    double robot_mass = 12.84;
    Eigen::Matrix3d trunk_inertia;
    Eigen::Matrix3d trunk_inertia_world;
    Eigen::Matrix3d torso_rot_mat;
    // Eigen::Matrix<double, 3, 4> foot_pos;

    // Calculate the A matrix in continuous-time dynamics
    double sin_roll = sin(x[0]);
    double cos_roll = cos(x[0]);

    double sin_pitch = sin(x[1]);
    double cos_pitch = cos(x[1]);
    double tan_pitch = tan(x[1]);

    double sin_yaw = sin(x[2]);
    double cos_yaw = cos(x[2]);

    Eigen::Matrix3d ang_vel_to_rpy_rate;
    ang_vel_to_rpy_rate << cos_yaw, sin_yaw, 0,
            -sin_yaw, cos_yaw, 0,
            0, 0, 1;
    // ang_vel_to_rpy_rate << cos_yaw/cos_pitch, sin_yaw/cos_pitch, 0,
    //                        -sin_yaw, cos_yaw, 0,
    //                        cos_yaw*tan_pitch, sin_yaw*tan_pitch, 1;

    Ac.block<3, 3>(0, 6) = ang_vel_to_rpy_rate;
    Ac.block<3, 3>(3, 9) = Eigen::Matrix3d::Identity();

    // Calculate the B matrix in continuou-time dynamics
    trunk_inertia << 0.0168128557, 0.0, 0.0,
            0.0, 0.063009565, 0.0,
            0.0, 0.0, 0.0716547275;

    // torsorot_mat.setIdentity();
    torso_rot_mat << cos_yaw, -sin_yaw, 0,
            sin_yaw, cos_yaw, 0,
            0, 0, 1;

    // torsorot_mat << cos_yaw * cos_pitch, cos_yaw * sin_pitch * sin_roll - sin_yaw * cos_roll, cos_yaw * sin_pitch * cos_roll + sin_yaw * sin_roll,
    //                 sin_yaw * cos_pitch, sin_yaw * sin_pitch * sin_roll + cos_yaw * cos_roll, sin_yaw * sin_pitch * cos_roll - cos_yaw * sin_roll,
    //                 -sin_pitch, cos_pitch * sin_roll, cos_pitch * cos_roll;

    // foot_pos << 0.17, 0.17, -0.17, -0.17,
    //             0.13, -0.13, 0.13, -0.13,
    //             -0.3, -0.3, -0.3, -0.3;
    trunk_inertia_world = torso_rot_mat * trunk_inertia * torso_rot_mat.transpose();

    for (int i = 0; i < NUM_LEG; ++i) {
        Bc.block<3, 3>(6, 3 * i) = trunk_inertia_world.inverse() * Utils::skew(foot_pos.block<3, 1>(0, i));
        Bc.block<3, 3>(9, 3 * i) = (1 / robot_mass) * Eigen::Matrix3d::Identity();
    }

    // Continuous-time dynamics
    x_dot_vec = Ac * x_vec + Bc * u_vec + g_vec;
}

void QuadrupedModel::ct_srb_jacobian(double *jac, const double *x, const double *u, Eigen::Matrix<double, 3, NUM_LEG> foot_pos) const {
    Eigen::Matrix<double, 12, 12> Ac;
    Eigen::Matrix<double, 12, 12> Bc;
    Ac.setZero();
    Bc.setZero();
    double robot_mass = 12.84;
    Eigen::Matrix3d trunk_inertia;
    Eigen::Matrix3d trunk_inertia_world;
    Eigen::Matrix3d torso_rot_mat;
    // Eigen::Matrix<double, 3, 4> foot_pos;

    // Calculate the A matrix in continuous-time dynamics
    double cos_roll = cos(x[0]);
    double sin_roll = sin(x[0]);

    double sin_pitch = sin(x[1]);
    double cos_pitch = cos(x[1]);
    double tan_pitch = tan(x[1]);

    double cos_yaw = cos(x[2]);
    double sin_yaw = sin(x[2]);

    Eigen::Matrix3d ang_vel_to_rpy_rate;
    ang_vel_to_rpy_rate << cos_yaw, sin_yaw, 0,
            -sin_yaw, cos_yaw, 0,
            0, 0, 1;
    // ang_vel_to_rpy_rate << cos_yaw/cos_pitch, sin_yaw/cos_pitch, 0,
    //                        -sin_yaw, cos_yaw, 0,
    //                        cos_yaw*tan_pitch, sin_yaw*tan_pitch, 1;

    Ac.block<3, 3>(0, 6) = ang_vel_to_rpy_rate;
    Ac.block<3, 3>(3, 9) = Eigen::Matrix3d::Identity();

    // Calculate the B matrix in continuou-time dynamics
    trunk_inertia << 0.0168128557, 0.0, 0.0,
            0.0, 0.063009565, 0.0,
            0.0, 0.0, 0.0716547275;

    // torsorot_mat.setIdentity();
    torso_rot_mat << cos_yaw, -sin_yaw, 0,
            sin_yaw, cos_yaw, 0,
            0, 0, 1;

    // torsorot_mat << cos_yaw * cos_pitch, cos_yaw * sin_pitch * sin_roll - sin_yaw * cos_roll, cos_yaw * sin_pitch * cos_roll + sin_yaw * sin_roll,
    //                 sin_yaw * cos_pitch, sin_yaw * sin_pitch * sin_roll + cos_yaw * cos_roll, sin_yaw * sin_pitch * cos_roll - cos_yaw * sin_roll,
    //                 -sin_pitch, cos_pitch * sin_roll, cos_pitch * cos_roll;

    // foot_pos << 0.17, 0.17, -0.17, -0.17,
    //             0.12, -0.12, 0.12, -0.12,
    //             -0.3, -0.3, -0.3, -0.3;
    trunk_inertia_world = torso_rot_mat * trunk_inertia * torso_rot_mat.transpose();

    for (int i = 0; i < NUM_LEG; ++i) {
        Bc.block<3, 3>(6, 3 * i) = trunk_inertia_world.inverse() * Utils::skew(foot_pos.block<3, 1>(0, i));
        Bc.block<3, 3>(9, 3 * i) = (1 / robot_mass) * Eigen::Matrix3d::Identity();
    }

    // Jacobian
    Eigen::Map <Eigen::Matrix<double, 12, 24>> J(jac);
    J.setZero();
    J(0, 2) = x[7] * cos(x[2]) - x[6] * sin(x[2]);
    J(1, 2) = -x[6] * cos(x[2]) - x[7] * sin(x[2]);
    J.block<6, 6>(0, 6) = Ac.block<6, 6>(0, 6);
    J.block<6, 12>(6, 12) = Bc.block<6, 12>(6, 0);
}

// Single rigid body dynamics of a quadruped robot with quaternions
// THIS FUNCTION SHOULD BE GOOD
void QuadrupedModel::ct_srb_quat_dynamics(double *x_dot, const double *x, const double *u, Eigen::Matrix<double, 3, 4> foot_pos_body, Eigen::Matrix3d robot_inertia, double robot_mass, Eigen::Matrix3d torso_rot_mat) const {
    Eigen::Map<Eigen::VectorXd> x_dot_vec(x_dot, 13);
    Eigen::Map<const Eigen::VectorXd> x_vec(x, 13);
    Eigen::Map<const Eigen::VectorXd> u_vec(u, 12);

    Eigen::Vector3d g_vec_world;
    Eigen::Vector3d g_vec_body;
    g_vec_world << 0, 0, -9.81;
    g_vec_body = torso_rot_mat.transpose() * g_vec_world;

    Eigen::Vector3d body_com(0.0223, 0.002, -0.0005);
    Eigen::Vector3d moment_gravity = body_com.cross(5.204 * g_vec_body);

    Eigen::Vector3d moment_body;
    moment_body = Utils::skew(foot_pos_body.block<3, 1>(0, 0)) * u_vec.segment<3>(0) +
                  Utils::skew(foot_pos_body.block<3, 1>(0, 1)) * u_vec.segment<3>(3) +
                  Utils::skew(foot_pos_body.block<3, 1>(0, 2)) * u_vec.segment<3>(6) +
                  Utils::skew(foot_pos_body.block<3, 1>(0, 3)) * u_vec.segment<3>(9) +
                  moment_gravity;

    // change rate of position
    x_dot_vec.segment<3>(0) = x_vec.segment<3>(7);
    // change rate of quaternion
    x_dot_vec.segment<4>(3) = 0.5 * QuaternionUtils::G(x_vec.segment<4>(3)) * x_vec.segment<3>(10);
    // change rate of linear velocity
    x_dot_vec.segment<3>(7) = (u_vec.segment<3>(0) + u_vec.segment<3>(3) + u_vec.segment<3>(6) + u_vec.segment<3>(9)) / robot_mass + g_vec_body;
    // change rate of angular velocity
    // x_dot_vec.segment<3>(10) = robot_inertia.inverse() * (moment_body - x_vec.segment<3>(10).cross(robot_inertia * x_vec.segment<3>(10)));
    x_dot_vec.segment<3>(10) = robot_inertia.inverse() * moment_body;
}

// THIS FUNCTION SHOULD BE GOOD
void QuadrupedModel::ct_srb_quat_jacobian(double *jac, const double *x, const double *u, Eigen::Matrix<double, 3, 4> foot_pos_body, Eigen::Matrix3d robot_inertia, double robot_mass) const {
    (void)u;
    Eigen::Map<Eigen::Matrix<double, 13, 25>> J(jac);  // jac = [dfc_dx, dfc_du]
    J.setZero();

    Eigen::Map<const Eigen::VectorXd> x_vec(x, 13);

    // Calculate dfc_dx
    Eigen::MatrixXd dfc_dx(13, 13);
    dfc_dx.setZero();
    // dv/dv
    dfc_dx.block<3, 3>(0, 7) = Eigen::Matrix3d::Identity();
    // dqdot/dq
    dfc_dx.block<1, 3>(3, 4) = -0.5 * x_vec.segment<3>(10).transpose();
    dfc_dx.block<3, 1>(4, 3) = 0.5 * x_vec.segment<3>(10);
    dfc_dx.block<3, 3>(4, 4) = -0.5 * Utils::skew(x_vec.segment<3>(10));
    // dqdot/domega
    dfc_dx(3, 10) = -0.5 * x_vec(4);  // -0.5qa
    dfc_dx(3, 11) = -0.5 * x_vec(5);  // -0.5qb
    dfc_dx(3, 12) = -0.5 * x_vec(6);  // -0.5qc
    dfc_dx(4, 10) = 0.5 * x_vec(3);   // 0.5qs
    dfc_dx(4, 11) = -0.5 * x_vec(6);  // -0.5qc
    dfc_dx(4, 12) = 0.5 * x_vec(5);   // 0.5qb
    dfc_dx(5, 10) = 0.5 * x_vec(6);   // 0.5qc
    dfc_dx(5, 11) = 0.5 * x_vec(3);   // 0.5qs
    dfc_dx(5, 12) = -0.5 * x_vec(4);  // -0.5qa
    dfc_dx(6, 10) = -0.5 * x_vec(5);  // -0.5qb
    dfc_dx(6, 11) = 0.5 * x_vec(4);   // 0.5qa
    dfc_dx(6, 12) = 0.5 * x_vec(3);   // 0.5qs
    // domegadot/domega
    // dfc_dx.block<3, 3>(10, 10) = robot_inertia.inverse() * (Utils::skew(x_vec.segment<3>(10)) * robot_inertia - Utils::skew(robot_inertia * x_vec.segment<3>(10)));

    // Calculate dfc_du
    Eigen::MatrixXd dfc_du(13, 12);
    dfc_du.setZero();

    for (int i = 0; i < 4; ++i) {
        dfc_du.block<3, 3>(7, 3 * i) = (1 / robot_mass) * Eigen::Matrix3d::Identity();
        dfc_du.block<3, 3>(10, 3 * i) = robot_inertia.inverse() * Utils::skew(foot_pos_body.block<3, 1>(0, i));
    }

    // Get Jacobian
    J.block<13, 13>(0, 0) = dfc_dx;
    J.block<13, 12>(0, 13) = dfc_du;
}

void QuadrupedModel::ct_srb_trot_quat_dynamics(double *x_dot, const double *x, const double *u, Eigen::Matrix<double, 3, 2> foot_pos_body, Eigen::Matrix3d robot_inertia, double robot_mass) const {
    Eigen::Map<Eigen::VectorXd> x_dot_vec(x_dot, 13);
    Eigen::Map<const Eigen::VectorXd> x_vec(x, 13);
    Eigen::Map<const Eigen::VectorXd> u_vec(u, 6);

    Eigen::Vector3d g_vec;
    g_vec << 0, 0, -9.81;

    Eigen::Vector3d body_com(0.0223, 0.002, -0.0005);
    Eigen::Vector3d moment_gravity = body_com.cross(5.204 * g_vec);

    Eigen::Vector3d moment_body;
    moment_body = Utils::skew(foot_pos_body.block<3, 1>(0, 0)) * u_vec.segment<3>(0) +
                  Utils::skew(foot_pos_body.block<3, 1>(0, 1)) * u_vec.segment<3>(3) +
                  moment_gravity;

    // change rate of position
    x_dot_vec.segment<3>(0) = x_vec.segment<3>(7);
    // change rate of quaternion
    x_dot_vec.segment<4>(3) = 0.5 * QuaternionUtils::G(x_vec.segment<4>(3)) * x_vec.segment<3>(10);
    // change rate of linear velocity
    x_dot_vec.segment<3>(7) = (u_vec.segment<3>(0) + u_vec.segment<3>(3)) / robot_mass + g_vec;
    // change rate of angular velocity
    // x_dot_vec.segment<3>(10) = robot_inertia.inverse() * (moment_body - x_vec.segment<3>(10).cross(robot_inertia * x_vec.segment<3>(10)));
    x_dot_vec.segment<3>(10) = robot_inertia.inverse() * moment_body;
}

// THIS FUNCTION SHOULD BE GOOD
void QuadrupedModel::ct_srb_trot_quat_jacobian(double *jac, const double *x, const double *u, Eigen::Matrix<double, 3, 2> foot_pos_body, Eigen::Matrix3d robot_inertia, double robot_mass) const {
    (void)u;
    Eigen::Map<Eigen::Matrix<double, 13, 19>> J(jac);  // jac = [dfc_dx, dfc_du]
    J.setZero();

    Eigen::Map<const Eigen::VectorXd> x_vec(x, 13);

    // Calculate dfc_dx
    Eigen::MatrixXd dfc_dx(13, 13);
    dfc_dx.setZero();
    // dv/dv
    dfc_dx.block<3, 3>(0, 7) = Eigen::Matrix3d::Identity();
    // dqdot/dq
    dfc_dx.block<1, 3>(3, 4) = -0.5 * x_vec.segment<3>(10).transpose();
    dfc_dx.block<3, 1>(4, 3) = 0.5 * x_vec.segment<3>(10);
    dfc_dx.block<3, 3>(4, 4) = -0.5 * Utils::skew(x_vec.segment<3>(10));
    // dqdot/domega
    dfc_dx(3, 10) = -0.5 * x_vec(4);  // -0.5qa
    dfc_dx(3, 11) = -0.5 * x_vec(5);  // -0.5qb
    dfc_dx(3, 12) = -0.5 * x_vec(6);  // -0.5qc
    dfc_dx(4, 10) = 0.5 * x_vec(3);   // 0.5qs
    dfc_dx(4, 11) = -0.5 * x_vec(6);  // -0.5qc
    dfc_dx(4, 12) = 0.5 * x_vec(5);   // 0.5qb
    dfc_dx(5, 10) = 0.5 * x_vec(6);   // 0.5qc
    dfc_dx(5, 11) = 0.5 * x_vec(3);   // 0.5qs
    dfc_dx(5, 12) = -0.5 * x_vec(4);  // -0.5qa
    dfc_dx(6, 10) = -0.5 * x_vec(5);  // -0.5qb
    dfc_dx(6, 11) = 0.5 * x_vec(4);   // 0.5qa
    dfc_dx(6, 12) = 0.5 * x_vec(3);   // 0.5qs
    // domegadot/domega
    // dfc_dx.block<3, 3>(10, 10) = robot_inertia.inverse() * (Utils::skew(x_vec.segment<3>(10)) * robot_inertia - Utils::skew(robot_inertia * x_vec.segment<3>(10)));

    // Calculate dfc_du
    Eigen::MatrixXd dfc_du(13, 6);
    dfc_du.setZero();

    for (int i = 0; i < 2; ++i) {
        dfc_du.block<3, 3>(7, 3 * i) = (1 / robot_mass) * Eigen::Matrix3d::Identity();
        dfc_du.block<3, 3>(10, 3 * i) = robot_inertia.inverse() * Utils::skew(foot_pos_body.block<3, 1>(0, i));
    }

    // Get Jacobian
    J.block<13, 13>(0, 0) = dfc_dx;
    J.block<13, 6>(0, 13) = dfc_du;
}
