#pragma once

#include "mpc/LeggedMpc.h"
#include "utils/MovingWindowFilter.hpp"

namespace legged {
    using namespace altro;
    class QuatMpc : public LeggedMpc {
    public:
        QuatMpc() {};
        QuatMpc(LeggedState &state);
        bool update(LeggedState &state) override;
        bool goal_update(LeggedState &state) override;
        bool grf_update(LeggedState &state) override;
        bool foot_update(LeggedState &state) override;
        bool terrain_update(LeggedState &state) override;
    private:
        bool torso_pos_d_world_init = false;

        MovingWindowFilter torso_lin_vel_d_body_filter[3];
        MovingWindowFilter torso_pos_d_body_filter[3];

        Eigen::Vector3d torso_lin_vel_d_body_filtered;
        Eigen::Vector3d torso_pos_d_body_filtered;

        
        int en; // error state dimension
        int em; // error input dimension

        Eigen::VectorXd x_ref;
        Eigen::VectorXd u_ref;

        double attitude_traj_count;
        MovingWindowFilter torso_lin_vel_d_rel_filter_x;
        MovingWindowFilter torso_lin_vel_d_rel_filter_y;
        MovingWindowFilter terrain_angle_filter;
        Eigen::Matrix<a_float, 6, 3> C_mat;
        double mu;
        double fz_min;
        double fz_max;
    };
}  // namespace legged
