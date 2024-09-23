#pragma once

#include <Eigen/Dense>

#include "LeggedMpc.h"
#include "GyrostatModel.h"
#include "RobotState.h"
#include "Utils.h"
#include "altro/altro_solver.hpp"
#include "altro/solver/solver.hpp"

using namespace std;
using namespace altro;
using namespace robot;

namespace mpc {
class GyrostatEulerMpc : public LeggedMpc {
   public:
    GyrostatEulerMpc(){};
    GyrostatEulerMpc(RobotState &robot_state);

    void update(RobotState &robot_state) override;
    void goal_update(RobotState &robot_state) override;
    void ctrl_update(RobotState &robot_state) override;

   private:    
    std::vector<Eigen::Matrix<double, GYROSTAT_EULER_MPC_STATE_DIM, 1>> x_traj_ref;  // reference state trajectory
    std::vector<Eigen::Matrix<double, GYROSTAT_MPC_INPUT_DIM, 1>> u_traj_ref;  // reference input trajectory
    Eigen::Matrix<double, GYROSTAT_EULER_MPC_STATE_DIM, 1> x_init;  // initial state
};
}  // namespace mpc
