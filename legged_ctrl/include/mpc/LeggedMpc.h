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
        LeggedContactFSM leg_FSM[NUM_LEG]; // finite state machine for each leg

        int n; // state dimension
        int m; // input dimension
        double h; // time step
        int horizon; // horizon length
        int num_contacts; // number of contacts

        AltroOptions opts; // ALTRO solver options
        std::shared_ptr<QuadrupedModel> model_ptr; // pointer to model
        ContinuousDynamicsFunction ct_dyn; // continuous time dynamics function
        ContinuousDynamicsJacobian ct_jac; // continuous time dynamics jacobian
        ExplicitDynamicsFunction dt_dyn; // discrete time dynamics function
        ExplicitDynamicsJacobian dt_jac; // discrete time dynamics jacobian
        ConstraintFunction friction_cone_con; // friction cone constraint function
        ConstraintJacobian friction_cone_jac; // friction cone constraint jacobian
        std::vector<Eigen::VectorXd> x_traj_ref; // reference state trajectory
        std::vector<Eigen::VectorXd> u_traj_ref; // reference input trajectory
        Eigen::VectorXd x_init; // initial state
    };
}  // namespace legged
