#pragma once

#include <Eigen/Dense>

#include "RobotState.h"
#include "Utils.h"

namespace mpc {
class HumanoidModel {
   public:
    HumanoidModel(){};

    void ct_srb_quat_dyn(double *x_dot, const double *x, const double *u, Eigen::Matrix<double, 3, 4> foot_pos_body, Eigen::Matrix3d robot_inertia, double robot_mass, Eigen::Matrix3d torso_rot_mat) const;
    void ct_srb_quat_jac(double *jac, const double *x, const double *u, Eigen::Matrix<double, 3, 4> foot_pos_body, Eigen::Matrix3d robot_inertia, double robot_mass) const;
};
}  // namespace mpc
