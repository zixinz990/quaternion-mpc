#pragma once

#include <cmath>

#include <Eigen/Dense>

namespace legged {
    class Go1Dynamics {
    public:
        Go1Dynamics() {};
        ~Go1Dynamics() {};
        void go1_FL_inverse_dynamics_task_space(const Eigen::Vector3d &q, const Eigen::Vector3d &v, const Eigen::Vector3d &a, Eigen::Vector3d &tau);
        void go1_FR_inverse_dynamics_task_space(const Eigen::Vector3d &q, const Eigen::Vector3d &v, const Eigen::Vector3d &a, Eigen::Vector3d &tau);
        void go1_RL_inverse_dynamics_task_space(const Eigen::Vector3d &q, const Eigen::Vector3d &v, const Eigen::Vector3d &a, Eigen::Vector3d &tau);
        void go1_RR_inverse_dynamics_task_space(const Eigen::Vector3d &q, const Eigen::Vector3d &v, const Eigen::Vector3d &a, Eigen::Vector3d &tau);
    };
}