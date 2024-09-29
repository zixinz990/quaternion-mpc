#pragma once

#include <vector>
#include <iostream>
#include <Eigen/Dense>
#include "LeggedParams.h"
#include "LeggedState.h"
#include "utils/Utils.h"

namespace legged {

    enum LeggedContactState {
        SWING,  // 0
        STANCE  // 1
    };

    class LeggedContactFSM {
    public:
        LeggedContactFSM() {};

        void reset_params(LeggedState &legged_state, int _leg_id);

        void set_trotting_gait();

        void set_static_gait();
        
        void set_climb_gait();

        void set_crawl_gait_pattern();

        void set_trot_with_stand_gait_pattern();

        void set_default_stand_pattern();

        LeggedContactState get_contact_state() { return s; }

        // predict contact after a given period of time
        LeggedContactState predict_contact_state(double dt);

        Eigen::Vector3d get_pos_target() { return FSM_foot_pos_target_world; }

        Eigen::Vector3d get_vel_target() { return FSM_foot_vel_target_world; }

        // foot_pos_cur_world is the feedback foot position, in stance phase we should hold this position
        // foot_pos_target_world is the raibert strategy target position, in swing phase we should move to this position
        double update(double dt, double gait_freq, Eigen::Vector3d foot_pos_cur_world, Eigen::Vector3d foot_pos_target_world, bool foot_force_flag);

        // reset everything to the start of the gait pattern
        void reset();

        void gait_update(double dt);

        void swing_enter(Eigen::Vector3d foot_pos_cur_world);

        void swing_update(double dt, double gait_freq, Eigen::Vector3d foot_pos_target_world);

        void swing_exit() {};

        // enter function that both swing and stance can use
        void common_enter();

        void stance_enter(Eigen::Vector3d foot_pos_cur_world);

        void stance_update(double dt, bool foot_force_flag);

        void stance_exit(Eigen::Vector3d foot_pos_cur_world);

        double percent_in_state();

        // either swing or stance, the getTarget function will return the target foot position
        Eigen::Vector3d FSM_foot_pos_target_world;
        Eigen::Vector3d FSM_foot_vel_target_world;
        Eigen::Vector3d FSM_foot_acc_target_world;

    private:
        int leg_id;

        // leg finite state machine
        LeggedContactState s;
        double gait_phase;   // 0-1
        double gait_freq;
        bool gait_freeze;  // this is used to deal with late contact
        int gait_freeze_counter;  // this is used to deal with late contact

        // gait pattern definition
        std::vector <LeggedContactState> gait_pattern;
        std::vector<double> gait_pattern_switch_time;
        int gait_pattern_size;
        bool gait_pattern_loaded = false;

        // gait execution control
        int gait_pattern_idx;
        int prev_gait_pattern_idx;

        // record time within one state
        double cur_state_start_time; // 0-1 phase time
        double cur_state_end_time;   // 0-1 phase time

        // swing phase helper variables
        bool not_first_call = false;
        Eigen::Vector3d swing_start_foot_pos_world;
        Eigen::Vector3d swing_end_foot_pos_world;
        Eigen::Vector3d swing_extend_foot_pos_world; // this variable is used to deal with late contact, if the foot is not in contact, we will extend the swing phase and add this position to the target position

        // record terrain height
        double terrain_height = 0;

        BezierUtils bezierUtils;
        QuinticCurve quintic_curve;
    };

}  // namespace legged