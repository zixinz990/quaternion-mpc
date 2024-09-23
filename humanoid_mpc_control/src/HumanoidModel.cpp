#include "HumanoidModel.h"

namespace mpc {
void HumanoidModel::ct_srb_quat_dyn(double *x_dot, const double *x, const double *u, Eigen::Matrix<double, 3, 4> foot_pos_body, Eigen::Matrix3d robot_inertia, double robot_mass, Eigen::Matrix3d torso_rot_mat) const {
    Eigen::Map<Eigen::VectorXd> x_dot_vec(x_dot, MPC_STATE_DIM);
    Eigen::Map<const Eigen::VectorXd> x_vec(x, MPC_STATE_DIM);
    Eigen::Map<const Eigen::VectorXd> u_vec(u, MPC_INPUT_DIM);

    Eigen::Vector3d g_vec_world;
    Eigen::Vector3d g_vec_body;
    g_vec_world << 0, 0, -9.81;
    g_vec_body = torso_rot_mat.transpose() * g_vec_world;

    // Eigen::Vector3d body_com(0.0223, 0.002, -0.0005);
    // Eigen::Vector3d moment_gravity = body_com.cross(5.204 * g_vec_body);

    Eigen::Vector3d moment_body;
    moment_body = Utils::skew(foot_pos_body.block<3, 1>(0, 0)) * u_vec.segment<3>(0) +
                  Utils::skew(foot_pos_body.block<3, 1>(0, 1)) * u_vec.segment<3>(3) +
                  Utils::skew(foot_pos_body.block<3, 1>(0, 2)) * u_vec.segment<3>(6) +
                  Utils::skew(foot_pos_body.block<3, 1>(0, 3)) * u_vec.segment<3>(9);

    // change rate of position
    x_dot_vec.segment<3>(0) = x_vec.segment<3>(7);
    // change rate of quaternion
    x_dot_vec.segment<4>(3) = 0.5 * Utils::G(x_vec.segment<4>(3)) * x_vec.segment<3>(10);
    // change rate of linear velocity
    x_dot_vec.segment<3>(7) = (u_vec.segment<3>(0) + u_vec.segment<3>(3) + u_vec.segment<3>(6) + u_vec.segment<3>(9)) / robot_mass + g_vec_body;
    // change rate of angular velocity
    // x_dot_vec.segment<3>(10) = robot_inertia.inverse() * (moment_body - x_vec.segment<3>(10).cross(robot_inertia * x_vec.segment<3>(10)));
    x_dot_vec.segment<3>(10) = robot_inertia.inverse() * moment_body;
}

void HumanoidModel::ct_srb_quat_jac(double *jac, const double *x, const double *u, Eigen::Matrix<double, 3, 4> foot_pos_body, Eigen::Matrix3d robot_inertia, double robot_mass) const {
    (void)u;
    Eigen::Map<Eigen::Matrix<double, MPC_STATE_DIM, MPC_STATE_DIM + MPC_INPUT_DIM>> J(jac);  // jac = [dfc_dx, dfc_du]
    J.setZero();

    Eigen::Map<const Eigen::VectorXd> x_vec(x, MPC_STATE_DIM);

    // Calculate dfc_dx
    Eigen::MatrixXd dfc_dx(MPC_STATE_DIM, MPC_STATE_DIM);
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
    Eigen::MatrixXd dfc_du(MPC_STATE_DIM, MPC_INPUT_DIM);
    dfc_du.setZero();

    for (int i = 0; i < 4; ++i) {
        dfc_du.block<3, 3>(7, 3 * i) = (1 / robot_mass) * Eigen::Matrix3d::Identity();
        dfc_du.block<3, 3>(10, 3 * i) = robot_inertia.inverse() * Utils::skew(foot_pos_body.block<3, 1>(0, i));
    }

    // Get Jacobian
    J.block<MPC_STATE_DIM, MPC_STATE_DIM>(0, 0) = dfc_dx;
    J.block<MPC_STATE_DIM, MPC_INPUT_DIM>(0, MPC_STATE_DIM) = dfc_du;
}
}  // namespace mpc
