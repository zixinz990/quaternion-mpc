//
// Created by shuoy on 11/1/21.
//

#include "estimation/BasicKF.h"

namespace legged {

    BasicKF::BasicKF() {
        // constructor
        eye3.setIdentity();

        // C is fixed
        C.setZero();
        for (int i = 0; i < NUM_LEG; ++i) {
            C.block<3, 3>(i * 3, 0) = -eye3;  //-pos
            C.block<3, 3>(i * 3, 6 + i * 3) = eye3;  //foot pos
            C.block<3, 3>(NUM_LEG * 3 + i * 3, 3) = eye3;  // vel
            C(NUM_LEG * 6 + i, 6 + i * 3 + 2) = 1;  // height z of foot
        }

        // Q R are fixed
        Q.setIdentity();
        Q.block<3, 3>(0, 0) = PROCESS_NOISE_PIMU * eye3;               // position transition
        Q.block<3, 3>(3, 3) = PROCESS_NOISE_VIMU * eye3;               // velocity transition
        for (int i = 0; i < NUM_LEG; ++i) {
            Q.block<3, 3>(6 + i * 3, 6 + i * 3) = PROCESS_NOISE_PFOOT * eye3;  // foot position transition
        }

        R.setIdentity();
        for (int i = 0; i < NUM_LEG; ++i) {
            R.block<3, 3>(i * 3, i * 3) = SENSOR_NOISE_PIMU_REL_FOOT * eye3;                        // fk estimation
            R.block<3, 3>(NUM_LEG * 3 + i * 3, NUM_LEG * 3 + i * 3) = SENSOR_NOISE_VIMU_REL_FOOT * eye3;      // vel estimation
            R(NUM_LEG * 6 + i, NUM_LEG * 6 + i) = SENSOR_NOISE_ZFOOT;                               // height z estimation
        }

        // set A to identity
        A.setIdentity();

        // set B to zero
        B.setZero();

        assume_flat_ground = true;
    }

    BasicKF::BasicKF(bool assume_flat_ground_) : BasicKF() {
        // constructor
        assume_flat_ground = assume_flat_ground_;

        // change R according to this flag, if we do not assume the robot moves on flat ground,
        // then we cannot infer height z using this way
        if (assume_flat_ground == false) {
            for (int i = 0; i < NUM_LEG; ++i) {
                R(NUM_LEG * 6 + i, NUM_LEG * 6 + i) = 1e5;   // height z estimation not reliable
            }
        }
    }

    void BasicKF::init_state(LeggedState &state) {
        filter_initialized = true;
        P.setIdentity();
        P = P * 3;

        // set initial value of x
        x.setZero();
        x.segment<3>(0) = Eigen::Vector3d(0, 0, 0.09);
        for (int i = 0; i < NUM_LEG; ++i) {
            Eigen::Vector3d fk_pos = state.fbk.foot_pos_body.block<3, 1>(0, i);
            x.segment<3>(6 + i * 3) = state.fbk.torso_rot_mat * fk_pos + x.segment<3>(0);
        }
    }

    void BasicKF::update_estimation(LeggedState &state, double dt) {
        // update A B using latest dt
        A.block<3, 3>(0, 3) = dt * eye3;
        B.block<3, 3>(3, 0) = dt * eye3;

        // control input u is Ra + ag
        Eigen::Vector3d u = state.fbk.torso_rot_mat * state.fbk.torso_lin_acc_body + Eigen::Vector3d(0, 0, -9.81);

        // contact estimation, do something very simple first
        if (state.ctrl.movement_mode == 0) {  // stand
            for (int i = 0; i < NUM_LEG; ++i) estimated_contact[i] = 1.0;
        } else {  // walk
            for (int i = 0; i < NUM_LEG; ++i) {
                // estimated_contact[i] = std::min(std::max((state.fbk.foot_force(i)) / (70.0 - 0.0), 0.0), 1.0);
                // estimated_contact[i] = 1.0/(1.0+std::exp(-(state.fbk.foot_force(i)-20)));
                estimated_contact[i] = state.fbk.foot_contact_flag[i];
            }
        }

        // update Q
        Q.block<3, 3>(0, 0) = PROCESS_NOISE_PIMU * dt / 20.0 * eye3;
        Q.block<3, 3>(3, 3) = PROCESS_NOISE_VIMU * dt * 9.81 / 20.0 * eye3;

        // update Q R for legs not in contact
        for (int i = 0; i < NUM_LEG; ++i) {
            Q.block<3, 3>(6 + i * 3, 6 + i * 3) = (1 + (1 - estimated_contact[i]) * 1e3) * dt * PROCESS_NOISE_PFOOT * eye3; // foot position transition
            // for estimated_contact[i] == 1, Q = 0.002
            // for estimated_contact[i] == 0, Q = 1001*Q

            R.block<3, 3>(i * 3, i * 3) = (1 + (1 - estimated_contact[i]) * 1e3) * SENSOR_NOISE_PIMU_REL_FOOT * eye3; // fk estimation
            R.block<3, 3>(NUM_LEG * 3 + i * 3, NUM_LEG * 3 + i * 3) = (1 + (1 - estimated_contact[i]) * 1e3) * SENSOR_NOISE_VIMU_REL_FOOT * eye3; // vel estimation
            if (assume_flat_ground) {
                R(NUM_LEG * 6 + i, NUM_LEG * 6 + i) = (1 + (1 - estimated_contact[i]) * 1e3) * SENSOR_NOISE_ZFOOT; // height z estimation
            }
        }

        // process update
        xbar = A * x + B * u;
        Pbar = A * P * A.transpose() + Q;

        // measurement construction
        yhat = C * xbar;

        // actual measurement
        for (int i = 0; i < NUM_LEG; ++i) {
            Eigen::Vector3d fk_pos = state.fbk.foot_pos_body.block<3, 1>(0, i);
            y.block<3, 1>(i * 3, 0) = state.fbk.torso_rot_mat * fk_pos; // fk estimation
            Eigen::Vector3d leg_v = -state.fbk.foot_lin_vel_rel.block<3, 1>(0, i) - Utils::skew(state.fbk.torso_ang_vel_body) * fk_pos;
            y.block<3, 1>(NUM_LEG * 3 + i * 3, 0) = (1.0 - estimated_contact[i]) * x.segment<3>(3) + estimated_contact[i] * state.fbk.torso_rot_mat * leg_v; // vel estimation
            y(NUM_LEG * 6 + i) = (1.0 - estimated_contact[i]) * (x(2) + fk_pos(2)) + estimated_contact[i] * 0; // height z estimation
        }

        S = C * Pbar * C.transpose() + R;
        S = 0.5 * (S + S.transpose());

        error_y = y - yhat;
        Serror_y = S.fullPivHouseholderQr().solve(error_y);

        x = xbar + Pbar * C.transpose() * Serror_y;

        SC = S.fullPivHouseholderQr().solve(C);
        P = Pbar - Pbar * C.transpose() * SC * Pbar;
        P = 0.5 * (P + P.transpose());

        // reduce position drift
        if (P.block<2, 2>(0, 0).determinant() > 1e-6) {
            P.block<2, 16>(0, 2).setZero();
            P.block<16, 2>(2, 0).setZero();
            P.block<2, 2>(0, 0) /= 10.0;
        }

        // final step
        // put estimated values back to A1CtrlStates& state
        for (int i = 0; i < NUM_LEG; ++i) {
            if (estimated_contact[i] < 0.5) {
                state.fbk.estimated_contact[i] = false;
            } else {
                state.fbk.estimated_contact[i] = true;
            }
        }

        state.fbk.torso_pos_world = x.segment<3>(0);
        state.fbk.torso_lin_vel_world = x.segment<3>(3);
    }

}  // namespace legged
