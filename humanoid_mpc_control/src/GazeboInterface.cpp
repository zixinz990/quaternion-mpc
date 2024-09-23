#include "GazeboInterface.h"
#include "Kinematics.h"

GazeboInterface::GazeboInterface(ros::NodeHandle& nh, string robot_name) {
    // ROS publishers
    pub_joint_cmd[0] = nh.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "/left_hip_yaw_controller/command", 1);
    pub_joint_cmd[1] = nh.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "/left_hip_abad_controller/command", 1);
    pub_joint_cmd[2] = nh.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "/left_hip_pitch_controller/command", 1);
    pub_joint_cmd[3] = nh.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "/left_knee_controller/command", 1);
    pub_joint_cmd[4] = nh.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "/left_ankle_controller/command", 1);

    pub_joint_cmd[5] = nh.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "/right_hip_yaw_controller/command", 1);
    pub_joint_cmd[6] = nh.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "/right_hip_abad_controller/command", 1);
    pub_joint_cmd[7] = nh.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "/right_hip_pitch_controller/command", 1);
    pub_joint_cmd[8] = nh.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "/right_knee_controller/command", 1);
    pub_joint_cmd[9] = nh.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "/right_ankle_controller/command", 1);

    pub_torso_cmd = nh.advertise<nav_msgs::Odometry>("/" + robot_name + "/torso_com_cmd", 1);

    foot_pos_world_pub[0] = nh.advertise<geometry_msgs::Point>("/" + robot_name + "/left_toe_pos_world", 1);
    foot_pos_world_pub[1] = nh.advertise<geometry_msgs::Point>("/" + robot_name + "/left_heel_pos_world", 1);
    foot_pos_world_pub[2] = nh.advertise<geometry_msgs::Point>("/" + robot_name + "/right_toe_pos_world", 1);
    foot_pos_world_pub[3] = nh.advertise<geometry_msgs::Point>("/" + robot_name + "/right_heel_pos_world", 1);

    // ROS subscribers
    sub_joint_states[0] = nh.subscribe("/" + robot_name + "/left_hip_yaw_controller/state", 1, &GazeboInterface::left_hip_yaw_state_callback, this);
    sub_joint_states[1] = nh.subscribe("/" + robot_name + "/left_hip_abad_controller/state", 1, &GazeboInterface::left_hip_abad_state_callback, this);
    sub_joint_states[2] = nh.subscribe("/" + robot_name + "/left_hip_pitch_controller/state", 1, &GazeboInterface::left_hip_pitch_state_callback, this);
    sub_joint_states[3] = nh.subscribe("/" + robot_name + "/left_knee_controller/state", 1, &GazeboInterface::left_knee_state_callback, this);
    sub_joint_states[4] = nh.subscribe("/" + robot_name + "/left_ankle_controller/state", 1, &GazeboInterface::left_ankle_state_callback, this);

    sub_joint_states[5] = nh.subscribe("/" + robot_name + "/right_hip_yaw_controller/state", 1, &GazeboInterface::right_hip_yaw_state_callback, this);
    sub_joint_states[6] = nh.subscribe("/" + robot_name + "/right_hip_abad_controller/state", 1, &GazeboInterface::right_hip_abad_state_callback, this);
    sub_joint_states[7] = nh.subscribe("/" + robot_name + "/right_hip_pitch_controller/state", 1, &GazeboInterface::right_hip_pitch_state_callback, this);
    sub_joint_states[8] = nh.subscribe("/" + robot_name + "/right_knee_controller/state", 1, &GazeboInterface::right_knee_state_callback, this);
    sub_joint_states[9] = nh.subscribe("/" + robot_name + "/right_ankle_controller/state", 1, &GazeboInterface::right_ankle_state_callback, this);

    sub_torso_com_odom = nh.subscribe("/" + robot_name + "/torso_com_odom", 1, &GazeboInterface::torso_com_odom_callback, this);
    sub_torso_imu = nh.subscribe("/" + robot_name + "/torso_imu", 1, &GazeboInterface::torso_imu_callback, this);
    sub_joy = nh.subscribe("/joy", 1, &GazeboInterface::joy_callback, this);

    robot_state.params.load(nh);
}

void GazeboInterface::ctrl_update() {
    // cout << "GAZEBO INTERFACE CTRL UPDATE" << endl;
    robot_state.ctrl.joint_pos_d = robot_state.fbk.joint_pos;
    robot_state.ctrl.joint_vel_d = robot_state.fbk.joint_vel;

    robot_state.ctrl.joint_tau_d.block<LEG_DOF, 1>(0, 0) = robot_state.fbk.left_toe_jac.transpose() * (-robot_state.ctrl.grf_d.block<3, 1>(0, 0)) +
                                                           robot_state.fbk.left_heel_jac.transpose() * (-robot_state.ctrl.grf_d.block<3, 1>(3, 0));

    robot_state.ctrl.joint_tau_d.block<LEG_DOF, 1>(LEG_DOF, 0) = robot_state.fbk.right_toe_jac.transpose() * (-robot_state.ctrl.grf_d.block<3, 1>(6, 0)) +
                                                                 robot_state.fbk.right_heel_jac.transpose() * (-robot_state.ctrl.grf_d.block<3, 1>(9, 0));

    send_cmd();
    debug_broadcast();
}

void GazeboInterface::fbk_update() {
    // cout << "GAZEBO INTERFACE FBK UPDATE" << endl;
    // Calculate rotation matrix
    robot_state.fbk.torso_rot_mat = robot_state.fbk.torso_quat.toRotationMatrix();
    robot_state.fbk.torso_euler = Utils::quat_to_euler(robot_state.fbk.torso_quat);
    robot_state.fbk.torso_rot_mat_z = Eigen::AngleAxisd(robot_state.fbk.torso_euler[2], Eigen::Vector3d::UnitZ());

    // Calculate foot position in body frame
    // robot_state.fbk.foot_pos_body.block<3, 1>(0, 0) = Kinematics::cal_left_ankle_pos_body(robot_state.fbk.joint_pos.block<LEG_DOF, 1>(0, 0));
    // robot_state.fbk.foot_pos_body.block<3, 1>(3, 0) = Kinematics::cal_right_ankle_pos_body(robot_state.fbk.joint_pos.block<LEG_DOF, 1>(LEG_DOF, 0));
    robot_state.fbk.foot_pos_body.block<3, 1>(0, 0) = Kinematics::cal_left_toe_pos_body(robot_state.fbk.joint_pos.block<LEG_DOF, 1>(0, 0));
    robot_state.fbk.foot_pos_body.block<3, 1>(0, 1) = Kinematics::cal_left_heel_pos_body(robot_state.fbk.joint_pos.block<LEG_DOF, 1>(0, 0));
    robot_state.fbk.foot_pos_body.block<3, 1>(0, 2) = Kinematics::cal_right_toe_pos_body(robot_state.fbk.joint_pos.block<LEG_DOF, 1>(LEG_DOF, 0));
    robot_state.fbk.foot_pos_body.block<3, 1>(0, 3) = Kinematics::cal_right_heel_pos_body(robot_state.fbk.joint_pos.block<LEG_DOF, 1>(LEG_DOF, 0));

    // Calculate foot position in world frame
    for (int i = 0; i < 4; i++) {
        robot_state.fbk.foot_pos_world.block<3, 1>(0, i) = robot_state.fbk.torso_pos_world + robot_state.fbk.torso_rot_mat * robot_state.fbk.foot_pos_body.block<3, 1>(0, i);
    }

    // Calculate foot Jacobian
    // robot_state.fbk.left_foot_jac = Kinematics::cal_left_ankle_jac(robot_state.fbk.joint_pos.block<LEG_DOF, 1>(0, 0));
    // robot_state.fbk.right_foot_jac = Kinematics::cal_right_ankle_jac(robot_state.fbk.joint_pos.block<LEG_DOF, 1>(LEG_DOF, 0));
    robot_state.fbk.left_toe_jac = Kinematics::cal_left_toe_jac(robot_state.fbk.joint_pos.block<LEG_DOF, 1>(0, 0));
    robot_state.fbk.left_heel_jac = Kinematics::cal_left_heel_jac(robot_state.fbk.joint_pos.block<LEG_DOF, 1>(0, 0));
    robot_state.fbk.right_toe_jac = Kinematics::cal_right_toe_jac(robot_state.fbk.joint_pos.block<LEG_DOF, 1>(LEG_DOF, 0));
    robot_state.fbk.right_heel_jac = Kinematics::cal_right_heel_jac(robot_state.fbk.joint_pos.block<LEG_DOF, 1>(LEG_DOF, 0));

    Eigen::FullPivLU<Eigen::Matrix<double, 3, 5>> lu_decomp_left_toe_jac(robot_state.fbk.left_toe_jac);
    Eigen::FullPivLU<Eigen::Matrix<double, 3, 5>> lu_decomp_left_heel_jac(robot_state.fbk.left_heel_jac);
    Eigen::FullPivLU<Eigen::Matrix<double, 3, 5>> lu_decomp_right_toe_jac(robot_state.fbk.right_toe_jac);
    Eigen::FullPivLU<Eigen::Matrix<double, 3, 5>> lu_decomp_right_heel_jac(robot_state.fbk.right_heel_jac);

    auto rank_left_toe_jac = lu_decomp_left_toe_jac.rank();
    auto rank_left_heel_jac = lu_decomp_left_heel_jac.rank();
    auto rank_right_toe_jac = lu_decomp_right_toe_jac.rank();
    auto rank_right_heel_jac = lu_decomp_right_heel_jac.rank();

    if (rank_left_toe_jac < 3 || rank_left_heel_jac < 3 || rank_right_toe_jac < 3 || rank_right_heel_jac < 3) {
        std::cout << "Foot Jacobian is singular!" << std::endl;
    }

    // Some coordinate transformation
    robot_state.fbk.torso_lin_vel_body = robot_state.fbk.torso_rot_mat.transpose() * robot_state.fbk.torso_lin_vel_world;
}

void GazeboInterface::send_cmd() {
    for (int i = 0; i < ACT_JOINTS; i++) {
        low_cmd.motorCmd[i].mode = 0x0A;
        low_cmd.motorCmd[i].q = 0;
        low_cmd.motorCmd[i].dq = 0;
        low_cmd.motorCmd[i].Kp = 0;
        low_cmd.motorCmd[i].Kd = 0;
        low_cmd.motorCmd[i].tau = robot_state.params.joint_kp * (robot_state.ctrl.joint_pos_d[i] - robot_state.fbk.joint_pos[i]) +
                                  robot_state.params.joint_kd * (robot_state.ctrl.joint_vel_d[i] - robot_state.fbk.joint_vel[i]) +
                                  robot_state.ctrl.joint_tau_d[i];
        pub_joint_cmd[i].publish(low_cmd.motorCmd[i]);
    }    
}

void GazeboInterface::debug_broadcast() {
    // Publish command
    torso_com_cmd.header.stamp = ros::Time::now();
    torso_com_cmd.pose.pose.position.x = robot_state.ctrl.torso_pos_d_world[0];
    torso_com_cmd.pose.pose.position.y = robot_state.ctrl.torso_pos_d_world[1];
    torso_com_cmd.pose.pose.position.z = robot_state.ctrl.torso_pos_d_world[2];
    torso_com_cmd.pose.pose.orientation.w = robot_state.ctrl.torso_quat_d.w();
    torso_com_cmd.pose.pose.orientation.x = robot_state.ctrl.torso_quat_d.x();
    torso_com_cmd.pose.pose.orientation.y = robot_state.ctrl.torso_quat_d.y();
    torso_com_cmd.pose.pose.orientation.z = robot_state.ctrl.torso_quat_d.z();
    torso_com_cmd.twist.twist.linear.x = robot_state.ctrl.torso_lin_vel_d_world[0];
    torso_com_cmd.twist.twist.linear.y = robot_state.ctrl.torso_lin_vel_d_world[1];
    torso_com_cmd.twist.twist.linear.z = robot_state.ctrl.torso_lin_vel_d_world[2];
    torso_com_cmd.twist.twist.angular.x = robot_state.ctrl.torso_ang_vel_d_body[0];
    torso_com_cmd.twist.twist.angular.y = robot_state.ctrl.torso_ang_vel_d_body[1];
    torso_com_cmd.twist.twist.angular.z = robot_state.ctrl.torso_ang_vel_d_body[2];
    pub_torso_cmd.publish(torso_com_cmd);

    // Publish foot position
    for (int i = 0; i < 4; i++) {
        foot_pos_world_msg[i].x = robot_state.fbk.foot_pos_world(0, i);
        foot_pos_world_msg[i].y = robot_state.fbk.foot_pos_world(1, i);
        foot_pos_world_msg[i].z = robot_state.fbk.foot_pos_world(2, i);
        foot_pos_world_pub[i].publish(foot_pos_world_msg[i]);
    }
}

void GazeboInterface::torso_com_odom_callback(const nav_msgs::Odometry::ConstPtr& odom_msg) {
    robot_state.fbk.torso_pos_world << odom_msg->pose.pose.position.x,
                                       odom_msg->pose.pose.position.y,
                                       odom_msg->pose.pose.position.z;
    robot_state.fbk.torso_quat = Eigen::Quaterniond(odom_msg->pose.pose.orientation.w,
                                                    odom_msg->pose.pose.orientation.x,
                                                    odom_msg->pose.pose.orientation.y,
                                                    odom_msg->pose.pose.orientation.z);
    robot_state.fbk.torso_lin_vel_world << odom_msg->twist.twist.linear.x,
                                           odom_msg->twist.twist.linear.y,
                                           odom_msg->twist.twist.linear.z;
    robot_state.fbk.torso_ang_vel_world << odom_msg->twist.twist.angular.x,
                                           odom_msg->twist.twist.angular.y,
                                           odom_msg->twist.twist.angular.z;
}

void GazeboInterface::torso_imu_callback(const sensor_msgs::Imu::ConstPtr& imu_msg) {
    robot_state.fbk.torso_ang_vel_body << imu_msg->angular_velocity.x,
                                          imu_msg->angular_velocity.y,
                                          imu_msg->angular_velocity.z;
}

void GazeboInterface::joy_callback(const sensor_msgs::Joy::ConstPtr& joy_msg) {
    robot_state.joy_cmd.joy_vel_x = joy_msg->axes[4] * robot_state.params.joy_vel_x_max;
    robot_state.joy_cmd.joy_vel_y = joy_msg->axes[3] * robot_state.params.joy_vel_y_max;
    robot_state.joy_cmd.joy_vel_z = joy_msg->axes[1] * robot_state.params.joy_vel_z_max;
    robot_state.joy_cmd.joy_roll_vel = joy_msg->axes[0] * robot_state.params.joy_roll_vel_max;
    // robot_state.joy_cmd.joy_roll_vel = -joy_msg->axes[6] * robot_state.params.joy_roll_vel_max;
    // robot_state.joy_cmd.joy_pitch_vel = -joy_msg->axes[7] * robot_state.params.joy_pitch_vel_max;
    // if (joy_msg->axes[2] < 1.0) {
    //     robot_state.joy_cmd.joy_pitch_vel = (1.0 - joy_msg->axes[2]) * robot_state.params.joy_pitch_vel_max;
    // } else if (joy_msg->axes[5] < 1.0) {
    //     robot_state.joy_cmd.joy_pitch_vel = (joy_msg->axes[5] - 1.0) * robot_state.params.joy_pitch_vel_max;
    // } else {
    //     robot_state.joy_cmd.joy_pitch_vel = 0.0;
    // }
    // robot_state.joy_cmd.joy_yaw_vel = joy_msg->axes[0] * robot_state.params.joy_yaw_vel_max;
    if (joy_msg->buttons[4] == 1) {
        robot_state.joy_cmd.stop_control = true;
    }
    if (joy_msg->buttons[3]) {
        robot_state.joy_cmd.sin_ang_vel = true;
    } else {
        robot_state.joy_cmd.sin_ang_vel = false;
    }
}

void GazeboInterface::left_hip_yaw_state_callback(const unitree_legged_msgs::MotorState& joint_state_msg) {
    robot_state.fbk.joint_pos[0] = joint_state_msg.q;
    robot_state.fbk.joint_vel[0] = joint_state_msg.dq;
}

void GazeboInterface::left_hip_abad_state_callback(const unitree_legged_msgs::MotorState& joint_state_msg) {
    robot_state.fbk.joint_pos[1] = joint_state_msg.q;
    robot_state.fbk.joint_vel[1] = joint_state_msg.dq;
}

void GazeboInterface::left_hip_pitch_state_callback(const unitree_legged_msgs::MotorState& joint_state_msg) {
    robot_state.fbk.joint_pos[2] = joint_state_msg.q;
    robot_state.fbk.joint_vel[2] = joint_state_msg.dq;
}

void GazeboInterface::left_knee_state_callback(const unitree_legged_msgs::MotorState& joint_state_msg) {
    robot_state.fbk.joint_pos[3] = joint_state_msg.q;
    robot_state.fbk.joint_vel[3] = joint_state_msg.dq;
}

void GazeboInterface::left_ankle_state_callback(const unitree_legged_msgs::MotorState& joint_state_msg) {
    robot_state.fbk.joint_pos[4] = joint_state_msg.q;
    robot_state.fbk.joint_vel[4] = joint_state_msg.dq;
}

void GazeboInterface::right_hip_yaw_state_callback(const unitree_legged_msgs::MotorState& joint_state_msg) {
    robot_state.fbk.joint_pos[5] = joint_state_msg.q;
    robot_state.fbk.joint_vel[5] = joint_state_msg.dq;
}

void GazeboInterface::right_hip_abad_state_callback(const unitree_legged_msgs::MotorState& joint_state_msg) {
    robot_state.fbk.joint_pos[6] = joint_state_msg.q;
    robot_state.fbk.joint_vel[6] = joint_state_msg.dq;
}

void GazeboInterface::right_hip_pitch_state_callback(const unitree_legged_msgs::MotorState& joint_state_msg) {
    robot_state.fbk.joint_pos[7] = joint_state_msg.q;
    robot_state.fbk.joint_vel[7] = joint_state_msg.dq;
}

void GazeboInterface::right_knee_state_callback(const unitree_legged_msgs::MotorState& joint_state_msg) {
    robot_state.fbk.joint_pos[8] = joint_state_msg.q;
    robot_state.fbk.joint_vel[8] = joint_state_msg.dq;
}

void GazeboInterface::right_ankle_state_callback(const unitree_legged_msgs::MotorState& joint_state_msg) {
    robot_state.fbk.joint_pos[9] = joint_state_msg.q;
    robot_state.fbk.joint_vel[9] = joint_state_msg.dq;
}
