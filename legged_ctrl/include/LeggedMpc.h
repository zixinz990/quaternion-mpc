#pragma once

#include <iostream>
#include <string>
#include <chrono>
#include <vector>

#include "altro/altro_solver.hpp"
#include "altro/solver/solver.hpp"
#include "altro/utils/formatting.hpp"
#include "fmt/core.h"
#include "fmt/chrono.h"

#include "RobotState.h"
#include "GyrostatModel.h"
#include "Utils.h"

using namespace std;
using namespace altro;
using namespace robot;

namespace mpc {
    class LeggedMpc {
    public:
        LeggedMpc();
        LeggedMpc(ros::NodeHandle &_nh);
        virtual void update(RobotState &state) {}
        virtual void goal_update(RobotState &state) {}
        virtual void ctrl_update(RobotState &state) {}
    protected:
        int n;
        int en;
        int m;
        int em;
        double h;
        int horizon;
        int num_contacts;
        AltroOptions opts;
        std::shared_ptr<GyrostatModel> model_ptr;
        ContinuousDynamicsFunction ct_dyn;
        ContinuousDynamicsJacobian ct_jac;
        ExplicitDynamicsFunction dt_dyn;
        ExplicitDynamicsJacobian dt_jac;
        // ConstraintFunction friction_cone_con;
        // ConstraintJacobian friction_cone_jac;
        // std::vector<Eigen::VectorXd> x_traj_ref;
        // std::vector<Eigen::VectorXd> u_traj_ref;
        // Eigen::VectorXd x_init;
    };
}  // namespace legged
