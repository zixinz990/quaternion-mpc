#pragma once

#include <Eigen/Dense>

#include "altro/solver/typedefs.hpp"
#include "altro/utils/formatting.hpp"

using namespace altro;

using ContinuousDynamicsFunction = std::function<void(double *, const double *, const double *)>;
using ContinuousDynamicsJacobian = std::function<void(double *, const double *, const double *)>;

class Utils {
   public:
    Utils(){};

    static Eigen::Matrix3d skew(Eigen::Vector3d vec);

    static Eigen::Vector3d quat_to_euler(Eigen::Quaterniond quat);

    static Eigen::Vector4d quat_rk4(Eigen::Vector4d q, Eigen::Vector3d w, double dt);

    // Cayley Map: phi to quaternion
    static Eigen::Vector4d cayley_map(Eigen::Vector3d phi);

    // Inverse Cayley Map: quaternion to phi
    static Eigen::Vector3d inv_cayley_map(Eigen::Vector4d q);

    // Quaternion multiplication
    static Eigen::Vector4d quat_mult(Eigen::Vector4d q1, Eigen::Vector4d q2);

    // Quternion conjugate
    static Eigen::Vector4d quat_conj(Eigen::Vector4d q);

    // L function
    static Eigen::Matrix4d L(Eigen::Vector4d q);

    // R function
    static Eigen::Matrix4d R(Eigen::Vector4d q);

    // G function
    static Eigen::MatrixXd G(Eigen::Vector4d q);

    static ExplicitDynamicsFunction midpoint_dyn(int n, int m, ContinuousDynamicsFunction f);

    static ExplicitDynamicsJacobian midpoint_jac(int n, int m, ContinuousDynamicsFunction f, ContinuousDynamicsJacobian df);
};
