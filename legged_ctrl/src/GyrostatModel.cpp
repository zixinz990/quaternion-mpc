#include "GyrostatModel.h"

namespace mpc {
void GyrostatModel::ct_quat_dyn(double *x_dot, const double *x, const double *u,
                                Eigen::Matrix3d robot_inertia, double robot_mass, Eigen::Matrix3d torso_rot_mat) const {
    Eigen::Map<Eigen::VectorXd> x_dot_vec(x_dot, 9); // [qs, qx, qy, qz, wx, wy, wz, rho_x, rho_y]
    Eigen::Map<const Eigen::VectorXd> x_vec(x, 9);
    Eigen::Map<const Eigen::VectorXd> u_vec(u, 2); // [tau_x, tau_y]

    Eigen::Vector3d g_vec_world = torso_rot_mat.transpose() * g_vec_body;

    // Change rate of quaternion
    x_dot_vec.head(4) = 0.5 * Utils::G(x_vec.head(4)) * x_vec.segment<3>(4);

    // Angular acceleration
    x_dot_vec.segment<3>(4) = robot_inertia.inverse() * (- rw_axis_map.transpose() * u_vec
                                                         - x_vec.segment<3>(4).cross(robot_inertia * x_vec.segment<3>(4)
                                                                                     + rw_axis_map.transpose() * x_vec.tail(2)));
    
    // Change rate of wheels' angular momentum
    x_dot_vec.tail(2) = u_vec;
}

void GyrostatModel::ct_quat_jac(double *jac, const double *x, const double *u,
                                Eigen::Matrix3d robot_inertia, double robot_mass) const {
    (void)u;
    Eigen::Map<Eigen::Matrix<double, 9, 11>> J(jac); // jac = [dfc_dx, dfc_du]
    Eigen::Matrix<double, 9, 9> dfc_dx;
    Eigen::Matrix<double, 9, 2> dfc_du;
    Eigen::Map<const Eigen::VectorXd> x_vec(x, 9);

    J.setZero();
    dfc_dx.setZero();
    dfc_du.setZero();

    // dqdot/dq
    dfc_dx.block<1, 3>(0, 1) = -0.5 * x_vec.segment<3>(4).transpose();
    dfc_dx.block<3, 1>(1, 0) = 0.5 * x_vec.segment<3>(4);
    dfc_dx.block<3, 3>(1, 1) = -Utils::skew(0.5 * x_vec.segment<3>(4));

    // dqdot/domega
    dfc_dx.block<4, 3>(0, 4) = 0.5 * Utils::G(x_vec.head(4));

    // dwdot/dw
    dfc_dx.block<3, 3>(4, 4) = -robot_inertia.inverse() * (- Utils::skew(robot_inertia * x_vec.segment<3>(4))
                                                           - Utils::skew(rw_axis_map.transpose() * x_vec.tail(2))
                                                           + Utils::skew(x_vec.segment<3>(4)) * robot_inertia);

    // dwdot/drho
    dfc_dx.block<3, 2>(4, 7) = -robot_inertia.inverse() * Utils::skew(x_vec.segment<3>(4)) * rw_axis_map.transpose();

    // dwdot/du
    dfc_du.block<3, 2>(4, 0) = -robot_inertia.inverse() * rw_axis_map.transpose();

    // drhodot/du
    dfc_du.block<2, 2>(7, 0) = Eigen::Matrix2d::Identity();

    // Get Jacobian
    J.block<9, 9>(0, 0) = dfc_dx;
    J.block<9, 2>(0, 9) = dfc_du;
}

void GyrostatModel::ct_euler_dyn(double *x_dot, const double *x, const double *u,
                                 Eigen::Matrix3d robot_inertia) const {
    Eigen::Map<Eigen::VectorXd> x_dot_vec(x_dot, 8); // [roll, pitch, yaw, wx, wy, wz, rho_x, rho_y]
    Eigen::Map<const Eigen::VectorXd> x_vec(x, 8);
    Eigen::Map<const Eigen::VectorXd> u_vec(u, 2); // [tau_x, tau_y]

    // Change rate of Euler angles (ZYX)
    double sin_roll = sin(x[0]);
    double cos_roll = cos(x[0]);

    double sin_pitch = sin(x[1]);
    double cos_pitch = cos(x[1]);
    double tan_pitch = tan(x[1]);

    double sin_yaw = sin(x[2]);
    double cos_yaw = cos(x[2]);

    Eigen::Matrix3d ang_vel_to_rpy_rate;
    ang_vel_to_rpy_rate << cos_yaw/cos_pitch, sin_yaw/cos_pitch, 0,
                           -sin_yaw, cos_yaw, 0,
                           cos_yaw*tan_pitch, sin_yaw*tan_pitch, 1;
    x_dot_vec.head(3) = ang_vel_to_rpy_rate * x_vec.segment<3>(3);

    // Angular acceleration
    x_dot_vec.segment<3>(3) = robot_inertia.inverse() * (- rw_axis_map.transpose() * u_vec
                                                         - x_vec.segment<3>(3).cross(robot_inertia * x_vec.segment<3>(3)
                                                                                     + rw_axis_map.transpose() * x_vec.tail(2)));
    
    // Change rate of wheels' angular momentum
    x_dot_vec.tail(2) = u_vec;
}

void GyrostatModel::ct_euler_jac(double *jac, const double *x, const double *u,
                                 Eigen::Matrix3d robot_inertia) const {
    (void)u;
    Eigen::Map<Eigen::Matrix<double, 8, 10>> J(jac); // jac = [dfc_dx, dfc_du]
    Eigen::Matrix<double, 8, 8> dfc_dx;
    Eigen::Matrix<double, 8, 2> dfc_du;
    Eigen::Map<const Eigen::VectorXd> x_vec(x, 8);

    J.setZero();
    dfc_dx.setZero();
    dfc_du.setZero();

    // dthetadot/domega
    double sin_roll = sin(x[0]);
    double cos_roll = cos(x[0]);

    double sin_pitch = sin(x[1]);
    double cos_pitch = cos(x[1]);
    double tan_pitch = tan(x[1]);

    double sin_yaw = sin(x[2]);
    double cos_yaw = cos(x[2]);

    Eigen::Matrix3d ang_vel_to_rpy_rate;
    ang_vel_to_rpy_rate << cos_yaw/cos_pitch, sin_yaw/cos_pitch, 0,
                           -sin_yaw, cos_yaw, 0,
                           cos_yaw*tan_pitch, sin_yaw*tan_pitch, 1;
    dfc_dx.block<3, 3>(0, 3) = ang_vel_to_rpy_rate;

    // dwdot/dw
    dfc_dx.block<3, 3>(3, 3) = -robot_inertia.inverse() * (- Utils::skew(robot_inertia * x_vec.segment<3>(3))
                                                           - Utils::skew(rw_axis_map.transpose() * x_vec.tail(2))
                                                           + Utils::skew(x_vec.segment<3>(3)) * robot_inertia);
    
    // dwdot/drho
    dfc_dx.block<3, 2>(3, 6) = -robot_inertia.inverse() * Utils::skew(x_vec.segment<3>(3)) * rw_axis_map.transpose();

    // dwdot/du
    dfc_du.block<3, 2>(3, 0) = -robot_inertia.inverse() * rw_axis_map.transpose();

    // drhodot/du
    dfc_du.block<2, 2>(6, 0) = Eigen::Matrix2d::Identity();

    // Get Jacobian
    J.block<8, 8>(0, 0) = dfc_dx;
    J.block<8, 2>(0, 8) = dfc_du;
}
} // namespace mpc
