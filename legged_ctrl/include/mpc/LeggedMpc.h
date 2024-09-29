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

#include "LeggedState.h"
#include "utils/LeggedContactFSM.h"
#include "utils/Utils.h"
#include "utils/AltroUtils.h"

namespace legged {
    using namespace altro;
    class LeggedMpc {
    public:
        LeggedMpc();
        virtual bool update(LeggedState &state) { return true; }
        virtual bool goal_update(LeggedState &state) { return true; }
        virtual bool grf_update(LeggedState &state) { return true; }
        virtual bool foot_update(LeggedState &state) { return true; }
        virtual bool terrain_update(LeggedState &state) { return true; }
    protected:
        LeggedContactFSM leg_FSM[NUM_LEG];
        int n;
        int en;
        int m;
        int em;
        double h;
        int horizon;
        int num_contacts;        
        AltroOptions opts;
        std::shared_ptr<QuadrupedModel> model_ptr;
        ContinuousDynamicsFunction ct_dyn;
        ContinuousDynamicsJacobian ct_jac;
        ExplicitDynamicsFunction dt_dyn;
        ExplicitDynamicsJacobian dt_jac;
        ConstraintFunction friction_cone_con;
        ConstraintJacobian friction_cone_jac;
        std::vector<Eigen::VectorXd> x_traj_ref;
        std::vector<Eigen::VectorXd> u_traj_ref;
        Eigen::VectorXd x_init;        
    };
}  // namespace legged
