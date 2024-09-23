#include "GazeboInterface.h"
#include "Kinematics.h"

GazeboInterface::GazeboInterface(ros::NodeHandle& nh, string robot_name) {
    // ROS publishers
    pub_wheel_tau_cmd[0] = nh.advertise<std_msgs::Float64>("/a1_rw_gazebo/RWX_controller/command", 1);
    pub_wheel_tau_cmd[1] = nh.advertise<std_msgs::Float64>("/a1_rw_gazebo/RWY_controller/command", 1);

    // ROS subscribers    
    sub_wheel_vel = nh.subscribe("/a1_rw_gazebo/joint_states", 1, &GazeboInterface::wheel_vel_callback, this);
    sub_torso_com_odom = nh.subscribe("/torso_odom", 1, &GazeboInterface::torso_com_odom_callback, this);
    sub_torso_imu = nh.subscribe("/trunk_imu", 1, &GazeboInterface::torso_imu_callback, this);
    sub_joy = nh.subscribe("/joy", 1, &GazeboInterface::joy_callback, this);
    sub_falling_pose = nh.subscribe("/a1_rw_gazebo/target_pose", 1, &GazeboInterface::falling_pose_callback, this);

    robot_state.params.load(nh);
}

void GazeboInterface::ctrl_update() {
    send_cmd();
}

void GazeboInterface::fbk_update() {
    // Calculate rotation matrix
    robot_state.fbk.torso_rot_mat = robot_state.fbk.torso_quat.toRotationMatrix();
    robot_state.fbk.torso_euler = Utils::quat_to_euler(robot_state.fbk.torso_quat);
    robot_state.fbk.torso_rot_mat_z = Eigen::AngleAxisd(robot_state.fbk.torso_euler[2], Eigen::Vector3d::UnitZ()); 

    // Some coordinate transformation
    robot_state.fbk.torso_lin_vel_body = robot_state.fbk.torso_rot_mat.transpose() * robot_state.fbk.torso_lin_vel_world;
}

void GazeboInterface::send_cmd() {
    wheel_tau_cmd_msg[0].data = robot_state.ctrl.wheel_tau_d[0];
    wheel_tau_cmd_msg[1].data = robot_state.ctrl.wheel_tau_d[1];
    pub_wheel_tau_cmd[0].publish(wheel_tau_cmd_msg[0]);
    pub_wheel_tau_cmd[1].publish(wheel_tau_cmd_msg[1]);
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

void GazeboInterface::wheel_vel_callback(const sensor_msgs::JointState::ConstPtr& joint_state_msg) {
    robot_state.fbk.wheel_vel[0] = joint_state_msg->velocity[12];
    robot_state.fbk.wheel_vel[1] = joint_state_msg->velocity[13];
}

void GazeboInterface::joy_callback(const sensor_msgs::Joy::ConstPtr& joy_msg) {
    robot_state.joy_cmd.joy_vel_x = joy_msg->axes[4] * robot_state.params.joy_vel_x_max;
    robot_state.joy_cmd.joy_vel_y = joy_msg->axes[3] * robot_state.params.joy_vel_y_max;
    robot_state.joy_cmd.joy_vel_z = joy_msg->axes[1] * robot_state.params.joy_vel_z_max;
    robot_state.joy_cmd.joy_roll_vel = joy_msg->axes[0] * robot_state.params.joy_roll_vel_max;
    // robot_state.joy_cmd.joy_roll_vel = -joy_msg->axes[6] * robot_state.params.joy_roll_vel_max;
    robot_state.joy_cmd.joy_pitch_vel = -joy_msg->axes[1] * robot_state.params.joy_pitch_vel_max;
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

void GazeboInterface::falling_pose_callback(const geometry_msgs::Pose::ConstPtr& pose_msg) {
    robot_state.ctrl.best_falling_pose << pose_msg->orientation.w,
                                          pose_msg->orientation.x,
                                          pose_msg->orientation.y,
                                          pose_msg->orientation.z;
}
