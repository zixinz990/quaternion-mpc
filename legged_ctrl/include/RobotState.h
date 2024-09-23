#pragma once

#include <ros/ros.h>

#include <Eigen/Dense>

#define NUM_JOINTS 20
#define ACT_JOINTS 10
#define LEG_DOF 5
#define MPC_STATE_DIM 13
#define MPC_INPUT_DIM 12
#define GYROSTAT_QUAT_MPC_STATE_DIM 9
#define GYROSTAT_EULER_MPC_STATE_DIM 8
#define GYROSTAT_MPC_INPUT_DIM 2

namespace robot {
class RobotFeedback {
   public:
    RobotFeedback() { reset(); }

    void reset();

    // Torso position
    Eigen::Vector3d torso_pos_world;  // from Gazebo

    // Torso attitude
    // Definition of attitude: rotation from body frame to world frame, v_w = R * v_b
    Eigen::Quaterniond torso_quat;  // from Gazebo
    Eigen::Vector3d torso_euler;
    Eigen::Matrix3d torso_rot_mat;
    Eigen::Matrix3d torso_rot_mat_z;

    // Torso velocity
    Eigen::Vector3d torso_lin_vel_world;  // from Gazebo
    Eigen::Vector3d torso_lin_vel_body;
    Eigen::Vector3d torso_ang_vel_world;  // from Gazebo
    Eigen::Vector3d torso_ang_vel_body;   // from Gazebo

    // Joint states
    // Joint sequence:
    //     left_hip_yaw_joint, left_hip_abad_joint, left_hip_pitch_joint, left_knee_joint, left_ankle_joint,
    //     right_hip_yaw_joint, right_hip_abad_joint, right_hip_pitch_joint, right_knee_joint, right_ankle_joint,
    //     left_shoulder_pitch_joint, left_shoulder_abad_joint, left_shoulder_yaw_joint, left_elbow_joint, left_hand_joint,
    //     right_shoulder_pitch_joint, right_shoulder_abad_joint, right_shoulder_yaw_joint, right_elbow_joint, right_hand_joint
    Eigen::Matrix<double, NUM_JOINTS, 1> joint_pos;
    Eigen::Matrix<double, NUM_JOINTS, 1> joint_vel;

    // Wheels velocity
    Eigen::Matrix<double, 2, 1> wheel_vel;

    // Foot Jacobian matrix
    Eigen::Matrix<double, 3, LEG_DOF> left_ankle_jac;
    Eigen::Matrix<double, 3, LEG_DOF> right_ankle_jac;
    Eigen::Matrix<double, 3, LEG_DOF> left_toe_jac;
    Eigen::Matrix<double, 3, LEG_DOF> left_heel_jac;
    Eigen::Matrix<double, 3, LEG_DOF> right_toe_jac;
    Eigen::Matrix<double, 3, LEG_DOF> right_heel_jac;

    // Foot position
    Eigen::Matrix<double, 3, 4> foot_pos_body;
    Eigen::Matrix<double, 3, 4> foot_pos_world;
};

class RobotControl {
   public:
    RobotControl() { reset(); }

    void reset();

    // Desired torso position
    Eigen::Vector3d torso_pos_d_world;
    Eigen::Vector3d torso_pos_d_rel;
    Eigen::Vector3d torso_pos_d_body;

    // Desired torso attitude
    Eigen::Quaterniond torso_quat_d;
    Eigen::Vector3d torso_euler_d;

    // Desired torso velocity
    Eigen::Vector3d torso_lin_vel_d_world;
    Eigen::Vector3d torso_lin_vel_d_rel;
    Eigen::Vector3d torso_lin_vel_d_body;
    Eigen::Vector3d torso_ang_vel_d_body;

    // Desired ground reaction force
    Eigen::Matrix<double, MPC_INPUT_DIM, 1> grf_d;

    // Desired wheel torque
    Eigen::Matrix<double, 2, 1> wheel_tau_d;

    // Desired joint states
    Eigen::Matrix<double, NUM_JOINTS, 1> joint_pos_d;
    Eigen::Matrix<double, NUM_JOINTS, 1> joint_vel_d;
    Eigen::Matrix<double, NUM_JOINTS, 1> joint_tau_d;

    // Falling cat test
    Eigen::Matrix<double, 4, 1> best_falling_pose;
};

class RobotJoyCmd {
   public:
    RobotJoyCmd() { reset(); }

    void reset();

    // Joystick command
    double joy_vel_x, joy_vel_y, joy_vel_z;
    double joy_roll_vel, joy_pitch_vel, joy_yaw_vel;

    bool stop_control;

    bool sin_ang_vel;
};

class RobotParams {
   public:
    RobotParams() { reset(); }

    void reset();

    void load(ros::NodeHandle &nh);

    // Robot parameters
    double robot_mass;
    Eigen::Matrix3d robot_inertia;
    Eigen::Vector2d wheel_inertia;
    double mu;
    double grf_z_max;

    // MPC parameters
    double mpc_dt;
    int mpc_horizon;
    Eigen::Matrix<double, GYROSTAT_QUAT_MPC_STATE_DIM, 1> quat_mpc_q_weights;
    Eigen::Matrix<double, GYROSTAT_EULER_MPC_STATE_DIM, 1> euler_mpc_q_weights;
    Eigen::Matrix<double, GYROSTAT_MPC_INPUT_DIM, 1> mpc_r_weights;
    double mpc_quat_weight;

    // Joint controller parameters
    double joint_kp, joint_kd;

    // Joystick parameters
    double joy_vel_x_max, joy_vel_y_max, joy_vel_z_max;
    double joy_roll_vel_max, joy_pitch_vel_max, joy_yaw_vel_max;
    double torso_z_max;
};

class RobotState {
   public:
    RobotState(){};

    void reset() {
        fbk.reset();
        ctrl.reset();
        joy_cmd.reset();
        params.reset();
    }

    RobotFeedback fbk;
    RobotControl ctrl;
    RobotJoyCmd joy_cmd;
    RobotParams params;
};
}  // namespace robot
