#include "RobotState.h"

namespace robot {
void RobotFeedback::reset() {
    torso_pos_world.setZero();
    torso_quat.setIdentity();
    torso_euler.setZero();
    torso_rot_mat.setIdentity();
    torso_rot_mat_z.setIdentity();
    torso_lin_vel_world.setZero();
    torso_lin_vel_body.setZero();
    torso_ang_vel_world.setZero();
    torso_ang_vel_body.setZero();
    joint_pos.setZero();
    joint_vel.setZero();
    wheel_vel.setZero();
    left_ankle_jac.setZero();
    right_ankle_jac.setZero();
    left_toe_jac.setZero();
    left_heel_jac.setZero();
    right_toe_jac.setZero();
    right_heel_jac.setZero();
    foot_pos_body.setZero();
    foot_pos_world.setZero();
}

void RobotControl::reset() {
    torso_pos_d_world.setZero();
    torso_pos_d_rel.setZero();
    torso_pos_d_body.setZero();
    torso_quat_d.setIdentity();
    torso_lin_vel_d_world.setZero();
    torso_lin_vel_d_rel.setZero();
    torso_lin_vel_d_body.setZero();
    torso_ang_vel_d_body.setZero();
    grf_d.setZero();
    wheel_tau_d.setZero();
    joint_pos_d.setZero();
    joint_vel_d.setZero();
    joint_tau_d.setZero();
    best_falling_pose.setZero();
}

void RobotJoyCmd::reset() {
    joy_vel_x = 0.0;
    joy_vel_y = 0.0;
    joy_vel_z = 0.0;
    joy_roll_vel = 0.0;
    joy_pitch_vel = 0.0;
    joy_yaw_vel = 0.0;
    stop_control = false;
    sin_ang_vel = false;
}

void RobotParams::reset() {
    robot_mass = 10.71;
    robot_inertia << 0.08, 0, 0.01,
                     0, 0.11, 0,
                     0.01, 0, 0.07;
    wheel_inertia << 0.0052752, 0.0018904;
    mu = 0.7;
    grf_z_max = 500.0;
    mpc_dt = 0.01;
    mpc_horizon = 20;
    quat_mpc_q_weights << 0, 0, 0, 0,
                     1, 1, 1,
                     1, 1;
    euler_mpc_q_weights << 1, 1, 1,
                        1, 1, 1,
                        1, 1;
    mpc_r_weights << 0.000001, 0.000001;
    mpc_quat_weight = 1.0;
    joint_kp = 300.0;
    joint_kd = 10.0;
    joy_vel_x_max = 1.0;
    joy_vel_y_max = 1.0;
    joy_vel_z_max = 1.0;
    joy_roll_vel_max = 1.0;
    joy_pitch_vel_max = 1.0;
    joy_yaw_vel_max = 1.0;
    torso_z_max = 0.7;
}

void RobotParams::load(ros::NodeHandle &nh) {
    nh.param("robot_mass", robot_mass, 10.71);
    double robot_inertia_xx, robot_inertia_xy, robot_inertia_xz, robot_inertia_yy, robot_inertia_yz, robot_inertia_zz;
    nh.param("robot_inertia_xx", robot_inertia_xx, 0.08);
    nh.param("robot_inertia_xy", robot_inertia_xy, 0.0);
    nh.param("robot_inertia_xz", robot_inertia_xz, 0.01);
    nh.param("robot_inertia_yy", robot_inertia_yy, 0.11);
    nh.param("robot_inertia_yz", robot_inertia_yz, 0.0);
    nh.param("robot_inertia_zz", robot_inertia_zz, 0.07);
    robot_inertia << robot_inertia_xx, robot_inertia_xy, robot_inertia_xz,
                     robot_inertia_xy, robot_inertia_yy, robot_inertia_yz,
                     robot_inertia_xz, robot_inertia_yz, robot_inertia_zz;
    double wheel_x_inertia, wheel_y_inertia;
    nh.param("wheel_x_inertia", wheel_x_inertia, 0.0052752);
    nh.param("wheel_y_inertia", wheel_y_inertia, 0.0018904);
    wheel_inertia << wheel_x_inertia, wheel_y_inertia;
    nh.param("mu", mu, 0.7);
    nh.param("grf_z_max", grf_z_max, 500.0);
    nh.param("mpc_dt", mpc_dt, 0.01);
    nh.param("mpc_horizon", mpc_horizon, 20);
    double mpc_q_0, mpc_q_1, mpc_q_2, mpc_q_3, mpc_q_4, mpc_q_5, mpc_q_6, mpc_q_7, mpc_q_8;
    nh.param("mpc_q_0", mpc_q_0, 0.0);
    nh.param("mpc_q_1", mpc_q_1, 0.0);
    nh.param("mpc_q_2", mpc_q_2, 0.0);
    nh.param("mpc_q_3", mpc_q_3, 1.0);
    nh.param("mpc_q_4", mpc_q_4, 1.0);
    nh.param("mpc_q_5", mpc_q_5, 1.0);
    nh.param("mpc_q_6", mpc_q_6, 1.0);
    nh.param("mpc_q_7", mpc_q_7, 1.0);
    nh.param("mpc_q_8", mpc_q_8, 1.0);
    quat_mpc_q_weights << mpc_q_0, mpc_q_1, mpc_q_2, mpc_q_3,
                          mpc_q_4, mpc_q_5, mpc_q_6,
                          mpc_q_7, mpc_q_8;
    euler_mpc_q_weights << mpc_q_0, mpc_q_1, mpc_q_2,
                           mpc_q_3, mpc_q_4, mpc_q_5,
                           mpc_q_6, mpc_q_7;
    nh.param("mpc_quat_weight", mpc_quat_weight, 1.0);
    double mpc_r_0, mpc_r_1;
    nh.param("mpc_r_0", mpc_r_0, 0.000001);
    nh.param("mpc_r_1", mpc_r_1, 0.000001);    
    mpc_r_weights << mpc_r_0, mpc_r_1;
    nh.param("joint_kp", joint_kp, 300.0);
    nh.param("joint_kd", joint_kd, 10.0);
    nh.param("joy_vel_x_max", joy_vel_x_max, 1.0);
    nh.param("joy_vel_y_max", joy_vel_y_max, 1.0);
    nh.param("joy_vel_z_max", joy_vel_z_max, 1.0);
    nh.param("joy_roll_vel_max", joy_roll_vel_max, 1.0);
    nh.param("joy_pitch_vel_max", joy_pitch_vel_max, 1.0);
    nh.param("joy_yaw_vel_max", joy_yaw_vel_max, 1.0);
    nh.param("torso_z_max", torso_z_max, 0.85);
}
}  // namespace robot
