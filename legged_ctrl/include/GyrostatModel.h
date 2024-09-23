#pragma once

#include <Eigen/Dense>

#include "Utils.h"

namespace mpc {
class GyrostatModel {
   public:
    GyrostatModel(){
        g_vec_body.setZero();
        rw_axis_map.setZero();
        
        g_vec_body << 0, 0, 0; // no gravity
        rw_axis_map << -1, 0, 0,
                        0, 1, 0;
    };

    void ct_quat_dyn(double *x_dot, const double *x, const double *u, Eigen::Matrix3d robot_inertia, double robot_mass, Eigen::Matrix3d torso_rot_mat) const;
    void ct_quat_jac(double *jac, const double *x, const double *u, Eigen::Matrix3d robot_inertia, double robot_mass) const;
    
    void ct_euler_dyn(double *x_dot, const double *x, const double *u, Eigen::Matrix3d robot_inertia) const;
    void ct_euler_jac(double *jac, const double *x, const double *u, Eigen::Matrix3d robot_inertia) const;

   private:
    Eigen::Vector3d g_vec_body;
    Eigen::Matrix<double, 2, 3> rw_axis_map;
};
}  // namespace mpc
