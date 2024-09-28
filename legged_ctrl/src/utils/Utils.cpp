//
// Created by shuoy on 10/19/21.
//

#include "utils/Utils.h"

Eigen::Vector3d Utils::quat_to_euler(Eigen::Quaterniond quat) {
    Eigen::Vector3d rst;

    // order https://github.com/libigl/eigen/blob/master/Eigen/src/Geometry/Quaternion.h
    Eigen::Matrix<double, 4, 1> coeff = quat.coeffs();
    double x = coeff(0);
    double y = coeff(1);
    double z = coeff(2);
    double w = coeff(3);

    double y_sqr = y * y;

    double t0 = +2.0 * (w * x + y * z);
    double t1 = +1.0 - 2.0 * (x * x + y_sqr);

    rst[0] = atan2(t0, t1);

    double t2 = +2.0 * (w * y - z * x);
    t2 = t2 > +1.0 ? +1.0 : t2;
    t2 = t2 < -1.0 ? -1.0 : t2;
    rst[1] = asin(t2);

    double t3 = +2.0 * (w * z + x * y);
    double t4 = +1.0 - 2.0 * (y_sqr + z * z);
    rst[2] = atan2(t3, t4);
    return rst;
}

Eigen::Vector3d Utils::quatToZyx(const Eigen::Quaterniond q) {
    Eigen::Vector3d zyx; // roll pitch yaw, but ZYX order

    // Convert a quaternion to ZYX order Euler angle
    double ysqr = q.y() * q.y();

    // Roll (x-axis rotation)
    double t0 = +2.0 * (q.w() * q.x() + q.y() * q.z());
    double t1 = +1.0 - 2.0 * (q.x() * q.x() + ysqr);
    zyx[0] = std::atan2(t0, t1);

    // Pitch (y-axis rotation)
    double t2 = +2.0 * (q.w() * q.y() - q.z() * q.x());
    t2 = t2 > 1.0 ? 1.0 : t2;
    t2 = t2 < -1.0 ? -1.0 : t2;
    zyx[1] = std::asin(t2);

    // Yaw (z-axis rotation)
    double t3 = +2.0 * (q.w() * q.z() + q.x() * q.y());
    double t4 = +1.0 - 2.0 * (ysqr + q.z() * q.z());
    zyx[2] = std::atan2(t3, t4);

    return zyx;
}

Eigen::Vector3d Utils::quat_to_rotVec(Eigen::Quaterniond quat) {
    Eigen::Vector3d axis;
    axis.setZero();
    double angle = 0.0;

    Eigen::Vector3d vec{quat.x(), quat.y(), quat.z()};
    if (vec.norm() < 1e-12) {
        axis.setZero();
    } else {
        axis = vec / vec.norm();
    }
    angle = 2 * atan2(vec.norm(), quat.w());

    return axis * angle;
}

Eigen::Quaterniond Utils::euler_to_quat(Eigen::Vector3d euler) {
    Eigen::Quaterniond rst;

    double roll = euler(0);
    double pitch = euler(1);
    double yaw = euler(2);

    roll /= 2;
    pitch /= 2;
    yaw /= 2;

    double cos_half_yaw = cos(yaw);
    double sin_half_yaw = sin(yaw);
    double cos_half_pitch = cos(pitch);
    double sin_half_pitch = sin(pitch);
    double cos_half_roll = cos(roll);
    double sin_half_roll = sin(roll);

    rst.w() = cos_half_yaw * cos_half_pitch * cos_half_roll + sin_half_yaw * sin_half_pitch * sin_half_roll;
    rst.x() = cos_half_yaw * cos_half_pitch * sin_half_roll - sin_half_yaw * sin_half_pitch * cos_half_roll;
    rst.y() = cos_half_yaw * sin_half_pitch * cos_half_roll + sin_half_yaw * cos_half_pitch * sin_half_roll;
    rst.z() = sin_half_yaw * cos_half_pitch * cos_half_roll - cos_half_yaw * sin_half_pitch * sin_half_roll;
    return rst;
}

Eigen::Matrix3d Utils::skew(Eigen::Vector3d vec) {
    Eigen::Matrix3d skew;
    skew << 0, -vec[2], vec[1], vec[2], 0, -vec[0], -vec[1], vec[0], 0;
    return skew;
}

// https://gist.github.com/pshriwise/67c2ae78e5db3831da38390a8b2a209f
Eigen::Matrix3d Utils::pseudo_inverse(const Eigen::Matrix3d &mat) {
    Eigen::JacobiSVD <Eigen::Matrix3d> svd(mat, Eigen::ComputeFullU | Eigen::ComputeFullV);
    double epsilon = std::numeric_limits<double>::epsilon();
    // For a non-square matrix
    // Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeThinU | Eigen::ComputeThinV);
    double tolerance = epsilon * std::max(mat.cols(), mat.rows()) * svd.singularValues().array().abs()(0);
    return svd.matrixV() * (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() *
           svd.matrixU().adjoint();
}

double Utils::cal_dihedral_angle(Eigen::Vector3d surf_coef_1, Eigen::Vector3d surf_coef_2) {
    // surface 1: a1 * x + b1 * y + c1 * z + d1 = 0, coef: [a1, b1, c1]
    // surface 2: a2 * x + b2 * y + c2 * z + d2 = 0, coef: [a1, b2, c2]
    double angle_cos =
            abs(surf_coef_1[0] * surf_coef_2[0] + surf_coef_1[1] * surf_coef_2[1] + surf_coef_1[2] * surf_coef_2[2])
            / (sqrt(surf_coef_1[0] * surf_coef_1[0] + surf_coef_1[1] * surf_coef_1[1] + surf_coef_1[2] * surf_coef_1[2]) *
               sqrt(surf_coef_2[0] * surf_coef_2[0] + surf_coef_2[1] * surf_coef_2[1] + surf_coef_2[2] * surf_coef_2[2]));
    return acos(angle_cos);
}

Eigen::Vector3d Utils::get_walk_surf_coef(Eigen::Matrix<double, 3, NUM_LEG> foot_pos_abs_prev) {
    Eigen::Matrix<double, NUM_LEG, 3> W;
    Eigen::VectorXd foot_pos_z;
    Eigen::Vector3d a;
    Eigen::Vector3d surf_coef;

    W.block<NUM_LEG, 1>(0, 0).setOnes();
    W.block<NUM_LEG, 2>(0, 1) = foot_pos_abs_prev.block<2, NUM_LEG>(0, 0).transpose();

    foot_pos_z.resize(NUM_LEG);
    foot_pos_z = foot_pos_abs_prev.block<1, NUM_LEG>(2, 0).transpose();

    a = Utils::pseudo_inverse(W.transpose() * W) * W.transpose() * foot_pos_z;

    // surface: a1 * x + a2 * y - z + a0 = 0, coefficient vector: [a1, a2, -1]
    surf_coef << a[1], a[2], -1;
    return surf_coef;
}

Eigen::Matrix<double, NUM_DOF, 1> Utils::joint_vec_unitree_to_pinnochio(Eigen::Matrix<double, NUM_DOF, 1> vec) {
    Eigen::Matrix<double, NUM_DOF, 1> return_vec;
    return_vec.segment<3>(0) = vec.segment<3>(0);
    return_vec.segment<3>(3) = vec.segment<3>(6);
    return_vec.segment<3>(6) = vec.segment<3>(3);
    return_vec.segment<3>(9) = vec.segment<3>(9);
    return return_vec;
}

Eigen::Matrix<double, NUM_DOF, 1> Utils::joint_vec_pinnochio_to_unitree(Eigen::Matrix<double, NUM_DOF, 1> vec) {
    Eigen::Matrix<double, NUM_DOF, 1> return_vec;
    return_vec.segment<3>(0) = vec.segment<3>(0);
    return_vec.segment<3>(3) = vec.segment<3>(6);
    return_vec.segment<3>(6) = vec.segment<3>(3);
    return_vec.segment<3>(9) = vec.segment<3>(9);
    return return_vec;
}

Eigen::Matrix<double, 6, 1> BezierUtils::get_foot_pos_curve(float t,
                                                            Eigen::Vector3d foot_pos_start,
                                                            Eigen::Vector3d foot_pos_final,
                                                            double terrain_pitch_angle = 0) {
    Eigen::Matrix<double, 6, 1> foot_pos_vel_acc_target;
    Eigen::Vector2d rst;
    // X-axis
    std::vector<double> bezierX{foot_pos_start(0),
                                foot_pos_start(0),
                                foot_pos_final(0),
                                foot_pos_final(0),
                                foot_pos_final(0)};
    rst = bezier_curve(t, bezierX);
    foot_pos_vel_acc_target[0] = rst[0];
    foot_pos_vel_acc_target[3] = rst[1];

    // Y-axis
    std::vector<double> bezierY{foot_pos_start(1),
                                foot_pos_start(1),
                                foot_pos_final(1),
                                foot_pos_final(1),
                                foot_pos_final(1)};
    rst = bezier_curve(t, bezierY);
    foot_pos_vel_acc_target[1] = rst[0];
    foot_pos_vel_acc_target[4] = rst[1];

    // Z-axis
    std::vector<double> bezierZ{foot_pos_start(2),
                                foot_pos_start(2),
                                foot_pos_final(2),
                                foot_pos_final(2),
                                foot_pos_final(2)};
    bezierZ[1] += FOOT_SWING_CLEARANCE1;
    bezierZ[2] += FOOT_SWING_CLEARANCE2 + 0.5 * sin(terrain_pitch_angle);
    rst = bezier_curve(t, bezierZ);
    foot_pos_vel_acc_target[2] = rst[0];
    foot_pos_vel_acc_target[5] = rst[1];

    return foot_pos_vel_acc_target;
}


Eigen::Vector2d BezierUtils::bezier_curve(double t, const std::vector<double> &P) {
    double y = 0, dy = 0;
    // for (int i = 0; i <= bezier_degree; i++) {
    //     y += bezier_coefficient(t, bezier_degree, i) * P[i];
    //     if (i < bezier_degree) {
    //         dy += bezier_coefficient(t, bezier_degree - 1, i) * (P[i + 1] - P[i]) * bezier_degree;
    //     }
    // }
    std::vector<double> coefficients{1, 4, 6, 4, 1};
    for (int i = 0; i <= bezier_degree; i++) {
        y += coefficients[i] * std::pow(t, i) * std::pow(1 - t, bezier_degree - i) * P[i];
    }
    return Eigen::Vector2d(y, dy);
}

double BezierUtils::binomial(int n, int k) {
    if (k > n) {
        return 0;
    }
    if (k == 0 || k == n) {
        return 1;
    }
    return binomial(n - 1, k - 1) + binomial(n - 1, k);
}

double BezierUtils::bezier_coefficient(double t, int n, int k) {
    return binomial(n, k) * std::pow(t, k) * std::pow(1 - t, n - k);
}

Eigen::Matrix<double, 9, 1> QuinticCurve::get_foot_swing_target(float t, float T, Eigen::Vector3d foot_pos_start, Eigen::Vector3d foot_pos_final) {
    Eigen::Matrix<double, 9, 1> foot_pos_vel_acc_target;
    Eigen::MatrixXd C(6, 6);
    C << 1, 0, 0, 0, 0, 0,
         1, T, T*T, T*T*T, T*T*T*T, T*T*T*T*T,
         0, 1, 0, 0, 0, 0,
         0, 1, 2*T, 3*T*T, 4*T*T*T, 5*T*T*T*T,
         1, T/2, T*T/4, T*T*T/8, T*T*T*T/16, T*T*T*T*T/32,
         0, 1, T, 3*T*T/4, 4*T*T*T/8, 5*T*T*T*T/16;
    double dx = foot_pos_final(0) - foot_pos_start(0);
    double dy = foot_pos_final(1) - foot_pos_start(1);
    double k = 1.26 / T;
    double v_xy_mid = k * sqrt(dx * dx + dy * dy);
    double theta = atan2(abs(dy), abs(dx));
    double v_x_mid = (dx >= 0 ? 1 : -1) * v_xy_mid * cos(theta);
    double v_y_mid = (dy >= 0 ? 1 : -1) * v_xy_mid * sin(theta);

    // Z
    double z_0 = foot_pos_start(2);
    double z_T = foot_pos_final(2);
    double z_dot_0 = 0.1;
    double z_dot_T = -0.1;
    double z_apex = 0.1;
    Eigen::VectorXd z_con(6);
    z_con << z_0, z_T, z_dot_0, z_dot_T, z_apex, 0.0;
    a_z = C.inverse() * z_con;
    foot_pos_vel_acc_target(2) = a_z(0) + a_z(1) * t + a_z(2) * t * t + a_z(3) * t * t * t + a_z(4) * t * t * t * t + a_z(5) * t * t * t * t * t;
    foot_pos_vel_acc_target(5) = a_z(1) + 2 * a_z(2) * t + 3 * a_z(3) * t * t + 4 * a_z(4) * t * t * t + 5 * a_z(5) * t * t * t * t;
    foot_pos_vel_acc_target(8) = 2 * a_z(2) + 6 * a_z(3) * t + 12 * a_z(4) * t * t + 20 * a_z(5) * t * t * t;

    // X
    double x_0 = foot_pos_start(0);
    double x_T = foot_pos_final(0);
    double x_dot_0 = 0.0;
    double x_dot_T = 0.0;
    double x_mid = (x_0 + x_T) / 2;
    Eigen::VectorXd x_con(6);
    x_con << x_0, x_T, x_dot_0, x_dot_T, x_mid, v_x_mid;
    a_x = C.inverse() * x_con;
    foot_pos_vel_acc_target(0) = a_x(0) + a_x(1) * t + a_x(2) * t * t + a_x(3) * t * t * t + a_x(4) * t * t * t * t + a_x(5) * t * t * t * t * t;
    foot_pos_vel_acc_target(3) = a_x(1) + 2 * a_x(2) * t + 3 * a_x(3) * t * t + 4 * a_x(4) * t * t * t + 5 * a_x(5) * t * t * t * t;
    foot_pos_vel_acc_target(6) = 2 * a_x(2) + 6 * a_x(3) * t + 12 * a_x(4) * t * t + 20 * a_x(5) * t * t * t;

    // Y
    double y_0 = foot_pos_start(1);
    double y_T = foot_pos_final(1);
    double y_dot_0 = 0.0;
    double y_dot_T = 0.0;
    double y_mid = (y_0 + y_T) / 2;
    Eigen::VectorXd y_con(6);
    y_con << y_0, y_T, y_dot_0, y_dot_T, y_mid, v_y_mid;
    a_y = C.inverse() * y_con;
    foot_pos_vel_acc_target(1) = a_y(0) + a_y(1) * t + a_y(2) * t * t + a_y(3) * t * t * t + a_y(4) * t * t * t * t + a_y(5) * t * t * t * t * t;
    foot_pos_vel_acc_target(4) = a_y(1) + 2 * a_y(2) * t + 3 * a_y(3) * t * t + 4 * a_y(4) * t * t * t + 5 * a_y(5) * t * t * t * t;
    foot_pos_vel_acc_target(7) = 2 * a_y(2) + 6 * a_y(3) * t + 12 * a_y(4) * t * t + 20 * a_y(5) * t * t * t;

    return foot_pos_vel_acc_target;
}
