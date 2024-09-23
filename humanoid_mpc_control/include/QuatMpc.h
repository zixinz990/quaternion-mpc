#pragma once

#include <Eigen/Dense>

#include "HumanoidModel.h"
#include "RobotState.h"
#include "Utils.h"
#include "altro/altro_solver.hpp"
#include "altro/solver/solver.hpp"

using namespace std;
using namespace altro;
using namespace robot;

namespace mpc {
class QuatMpc {
   public:
    QuatMpc(){};
    QuatMpc(RobotState &robot_state);

    void update(RobotState &robot_state);
    void goal_update(RobotState &robot_state);
    void ctrl_update(RobotState &robot_state);

   private:
    bool first_iter;
    double h;     // time step
    int horizon;  // horizon length

    AltroOptions opts;                         // ALTRO solver options
    std::shared_ptr<HumanoidModel> model_ptr;  // pointer to model

    ContinuousDynamicsFunction ct_dyn;  // continuous time dynamics function
    ContinuousDynamicsJacobian ct_jac;  // continuous time dynamics jacobian
    ExplicitDynamicsFunction dt_dyn;    // discrete time dynamics function
    ExplicitDynamicsJacobian dt_jac;    // discrete time dynamics jacobian

    Eigen::Matrix<double, 6, 3> C_mat;     // C matrix about friction cone
    ConstraintFunction friction_cone_con;  // friction cone constraint function
    ConstraintJacobian friction_cone_jac;  // friction cone constraint jacobian

    std::vector<Eigen::Matrix<double, MPC_STATE_DIM, 1>> x_traj_ref;  // reference state trajectory
    std::vector<Eigen::Matrix<double, MPC_INPUT_DIM, 1>> u_traj_ref;  // reference input trajectory
    Eigen::Matrix<double, MPC_STATE_DIM, 1> x_init;                   // initial state

    int attitude_traj_count;
};
}  // namespace mpc
