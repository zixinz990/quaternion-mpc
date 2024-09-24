#include "utils/LeggedContactFSM.h"

namespace legged {
    void LeggedContactFSM::reset_params(LeggedState &legged_state, int _leg_id) {
        leg_id = _leg_id;
        gait_freq = legged_state.param.gait_freq;
        s = STANCE;
        set_default_gait_pattern();
    }

    void LeggedContactFSM::reset() {
        gait_phase = 0;
        gait_freeze = false;
        gait_freeze_counter = 0;
        gait_pattern_index = 0;
        prev_gait_pattern_index = gait_pattern_size - 1;
        cur_state_start_time = 0;
        cur_state_end_time = gait_switch_time[gait_pattern_index];
        // to handle foot hold better 
        // at the moment of reset, stance foot keep holding its position
        // swing foot move to the saved target position
        if (s == SWING) {
            FSM_foot_pos_target_world = swing_end_foot_pos_world;
            FSM_foot_vel_target_world.setZero();
        }

        s = gait_state_pattern[gait_pattern_index];
        // terrain_height = 0;
        // so the outloop will keep reseting the FSM's target position
        not_first_call = false;
    }

    double LeggedContactFSM::update(double dt, double gait_freq, Eigen::Vector3d foot_pos_cur_world, Eigen::Vector3d foot_pos_target_world, bool foot_force_flag) {

        // the update is called first time in the main loop, record target position
        // in helper variables
        if (not_first_call == false) {
            swing_start_foot_pos_world = foot_pos_cur_world;
            swing_end_foot_pos_world = foot_pos_target_world;
            FSM_foot_pos_target_world = foot_pos_target_world;
            FSM_foot_vel_target_world.setZero();
            not_first_call = true;
        }

        gait_phase += gait_freq * dt;

        // std::cout << gait_phase << std::endl;
        // state transition
        if (s == STANCE) {
            if (gait_phase >= cur_state_end_time) {
                stance_exit(foot_pos_cur_world);
                swing_enter(foot_pos_cur_world);
                s = SWING;
            }
        } else if (s == SWING) {
            if (percent_in_state() > 0.9 && foot_force_flag) {
                // if we are in the second half of the swing phase and we have early contact
                // then we should switch to next phase immediately
                s = STANCE;
                swing_exit();
                stance_enter(foot_pos_cur_world);
            } else if (percent_in_state() >= 1.0) {
                s = STANCE;
                swing_exit();
                stance_enter(foot_pos_cur_world);
            }
        }

        // state update 
        if (s == STANCE) {
            // stance leg holds its position
            stance_update(dt, foot_force_flag);
        } else if (s == SWING) {
            // swing leg moves to the target position
            swing_update(dt, gait_freq, foot_pos_target_world);
        }
        return gait_phase;
    }

    void LeggedContactFSM::stance_exit(Eigen::Vector3d foot_pos_cur_world) {

        // record the height when we leave stance phase
        terrain_height = foot_pos_cur_world[2];
    }

    // TODO: add a function to load a gaite pattern from a file
    void LeggedContactFSM::set_default_gait_pattern() {
        // default gait pattern is trotting
        gait_state_pattern.clear();
        gait_pattern_index = 0;
        gait_switch_time.clear();
        if (leg_id == 0 || leg_id == 3) {
            gait_state_pattern.push_back(STANCE);
            gait_state_pattern.push_back(SWING);
        } else {
            gait_state_pattern.push_back(SWING);
            gait_state_pattern.push_back(STANCE);
        }
        gait_switch_time.push_back(0.5);
        gait_switch_time.push_back(1.0);
        gait_pattern_size = 2;
        gait_pattern_index = 0;
        prev_gait_pattern_index = gait_pattern_size - 1;
        gait_pattern_loaded = true;

        cur_state_start_time = 0.0;
        cur_state_end_time = gait_switch_time[gait_pattern_index];
    }

    void LeggedContactFSM::set_trot_with_stand_gait_pattern() {
        gait_state_pattern.clear();
        gait_pattern_index = 0;
        gait_switch_time.clear();
        if (leg_id == 0) {            //FL
            gait_state_pattern.push_back(STANCE);
            gait_state_pattern.push_back(SWING);
            gait_switch_time.push_back(0.6);
            gait_switch_time.push_back(1.0);
            gait_pattern_size = 2;
        } else if (leg_id == 1) {     //FR
            gait_state_pattern.push_back(STANCE);
            gait_state_pattern.push_back(SWING);
            gait_state_pattern.push_back(STANCE);
            gait_switch_time.push_back(0.1);
            gait_switch_time.push_back(0.5);
            gait_switch_time.push_back(1.0);
            gait_pattern_size = 3;
        } else if (leg_id == 2) {     //RL
            gait_state_pattern.push_back(STANCE);
            gait_state_pattern.push_back(SWING);
            gait_state_pattern.push_back(STANCE);
            gait_switch_time.push_back(0.1);
            gait_switch_time.push_back(0.5);
            gait_switch_time.push_back(1.0);
            gait_pattern_size = 3;
        } else if (leg_id == 3) {     //RR
            gait_state_pattern.push_back(STANCE);
            gait_state_pattern.push_back(SWING);
            gait_switch_time.push_back(0.6);
            gait_switch_time.push_back(1.0);
            gait_pattern_size = 2;
        }

        gait_pattern_index = 0;
        prev_gait_pattern_index = gait_pattern_size - 1;
        gait_pattern_loaded = true;

        cur_state_start_time = 0.0;
        cur_state_end_time = gait_switch_time[gait_pattern_index];
    }

    void LeggedContactFSM::set_crawl_gait_pattern() {
        gait_state_pattern.clear();
        gait_pattern_index = 0;
        gait_switch_time.clear();
        if (leg_id == 0) {            //FL
            gait_state_pattern.push_back(SWING);
            gait_state_pattern.push_back(STANCE);
            gait_switch_time.push_back(0.25);
            gait_switch_time.push_back(1.0);
            gait_pattern_size = 2;
        } else if (leg_id == 1) {     //FR
            gait_state_pattern.push_back(STANCE);
            gait_state_pattern.push_back(SWING);
            gait_state_pattern.push_back(STANCE);
            gait_switch_time.push_back(0.25);
            gait_switch_time.push_back(0.5);
            gait_switch_time.push_back(1.0);
            gait_pattern_size = 3;
        } else if (leg_id == 2) {     //RL
            gait_state_pattern.push_back(STANCE);
            gait_state_pattern.push_back(SWING);
            gait_state_pattern.push_back(STANCE);
            gait_switch_time.push_back(0.5);
            gait_switch_time.push_back(0.75);
            gait_switch_time.push_back(1.0);
            gait_pattern_size = 3;
        } else if (leg_id == 3) {     //RR
            gait_state_pattern.push_back(STANCE);
            gait_state_pattern.push_back(SWING);
            gait_switch_time.push_back(0.75);
            gait_switch_time.push_back(1.0);
            gait_pattern_size = 2;
        }


        gait_pattern_index = 0;
        prev_gait_pattern_index = gait_pattern_size - 1;
        gait_pattern_loaded = true;

        cur_state_start_time = 0.0;
        cur_state_end_time = gait_switch_time[gait_pattern_index];
    }

    void LeggedContactFSM::set_default_stand_pattern() {
        // default gait pattern is trotting
        gait_state_pattern.clear();
        gait_pattern_index = 0;
        gait_switch_time.clear();
        gait_state_pattern.push_back(STANCE);
        gait_switch_time.push_back(1.0);
        gait_pattern_size = 1;
        gait_pattern_index = 0;
        prev_gait_pattern_index = 0;
        gait_pattern_loaded = true;
    }

    void LeggedContactFSM::common_enter() {
        // first increase pattern index
        prev_gait_pattern_index = gait_pattern_index;
        gait_pattern_index = (gait_pattern_index + 1) % gait_pattern_size;
        if (gait_pattern_index < prev_gait_pattern_index) {
            // warp gait phase 
            gait_phase -= 1.0;
        }
        // reset cur_state_start_time
        cur_state_start_time = gait_phase;
        cur_state_end_time = gait_switch_time[gait_pattern_index];
        // std::cout << "cur_state_start_time: " << cur_state_start_time << std::endl;
        // std::cout << "cur_state_end_time: " << cur_state_end_time << std::endl;
        gait_freeze = false;
        gait_freeze_counter = 0;
    }

    void LeggedContactFSM::swing_enter(Eigen::Vector3d foot_pos_cur_world) {
        common_enter();
        swing_start_foot_pos_world << foot_pos_cur_world;
        swing_extend_foot_pos_world.setZero();
    }

    void LeggedContactFSM::stance_enter(Eigen::Vector3d foot_pos_cur_world) {
        common_enter();
        FSM_foot_pos_target_world << foot_pos_cur_world;
        FSM_foot_vel_target_world.setZero();
    }

    void LeggedContactFSM::swing_update(double dt, double gait_freq, Eigen::Vector3d foot_pos_target_world) {
        double t = percent_in_state();
        Eigen::Matrix<double, 9, 1> foot_pos_vel_acc_target =
            quintic_curve.get_foot_swing_target(0.5 * t / gait_freq, 0.5 / gait_freq,
                                                swing_start_foot_pos_world,
                                                foot_pos_target_world + swing_extend_foot_pos_world);
        FSM_foot_pos_target_world = foot_pos_vel_acc_target.segment(0, 3);
        FSM_foot_vel_target_world = foot_pos_vel_acc_target.segment(3, 3);
        FSM_foot_acc_target_world = foot_pos_vel_acc_target.segment(6, 3);
    }

    void LeggedContactFSM::stance_update(double dt, bool foot_force_flag) {
        // in stance phase, if do not have foot force flag, we should push down the foot to the ground
        // TODO: make these parameters configurable
//       if (!foot_force_flag) {
//           if (FSM_foot_pos_target_world[2] > -0.1) { // there is a threshold to prevent the foot from going too deep
//               FSM_foot_pos_target_world[2] -= 0.2 * dt;
//               FSM_foot_vel_target_world[2] = -0.2;
//           }
//       } else {
//           FSM_foot_vel_target_world.setZero();
//       }
    }

    double LeggedContactFSM::percent_in_state() {
        double percent = (gait_phase - cur_state_start_time) / (cur_state_end_time - cur_state_start_time);
        if (percent < 0.0) {
            percent = 0.0;
        } else if (percent > 1.0) {
            percent = 1.0;
        }

        return percent;
    }

    LeggedContactState LeggedContactFSM::predict_contact_state(double dt) {
        double predicted_gait_phase = gait_phase + gait_freq * dt;
        // warp predicted_gait_phase to 0 - 1 range
        while (predicted_gait_phase > 1.0) {
            predicted_gait_phase -= 1.0;
        }
        // determine the predicted state
        for (int i = 0; i < gait_pattern_size; i++) {
            if (predicted_gait_phase <= gait_switch_time[i]) {
                return gait_state_pattern[i];
            }
        }
        // should not reach here
        return STANCE;
    }

} // namespace legged
