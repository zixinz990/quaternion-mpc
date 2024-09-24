#include "interfaces/GazeboInterface.h"

namespace legged {

    GazeboInterface::GazeboInterface(ros::NodeHandle &_nh, int robot_type) : BaseInterface(_nh) {
        std::string robot_name = robot_type == 0 ? "a1" : "go1";
        
        // ROS publisher
        pub_joint_cmd[0] = nh.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/FL_hip_controller/command", 1);
        pub_joint_cmd[1] = nh.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/FL_thigh_controller/command", 1);
        pub_joint_cmd[2] = nh.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/FL_calf_controller/command", 1);

        pub_joint_cmd[3] = nh.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/FR_hip_controller/command", 1);
        pub_joint_cmd[4] = nh.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/FR_thigh_controller/command", 1);
        pub_joint_cmd[5] = nh.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/FR_calf_controller/command", 1);

        pub_joint_cmd[6] = nh.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/RL_hip_controller/command", 1);
        pub_joint_cmd[7] = nh.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/RL_thigh_controller/command", 1);
        pub_joint_cmd[8] = nh.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/RL_calf_controller/command", 1);

        pub_joint_cmd[9] = nh.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/RR_hip_controller/command", 1);
        pub_joint_cmd[10] = nh.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/RR_thigh_controller/command", 1);
        pub_joint_cmd[11] = nh.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/RR_calf_controller/command", 1);

        // ROS register callback, call backs directly modify variables in A1CtrlStates
        sub_gt_pose_msg = nh.subscribe("/torso_odom", 100, &GazeboInterface::gt_pose_callback, this);
        sub_imu_msg = nh.subscribe("/trunk_imu", 100, &GazeboInterface::imu_callback, this);

        sub_joint_msg[0] = nh.subscribe("/" + robot_name + "_gazebo/FL_hip_controller/state", 2, &GazeboInterface::FL_hip_state_callback, this);
        sub_joint_msg[1] = nh.subscribe("/" + robot_name + "_gazebo/FL_thigh_controller/state", 2, &GazeboInterface::FL_thigh_state_callback, this);
        sub_joint_msg[2] = nh.subscribe("/" + robot_name + "_gazebo/FL_calf_controller/state", 2, &GazeboInterface::FL_calf_state_callback, this);

        sub_joint_msg[3] = nh.subscribe("/" + robot_name + "_gazebo/FR_hip_controller/state", 2, &GazeboInterface::FR_hip_state_callback, this);
        sub_joint_msg[4] = nh.subscribe("/" + robot_name + "_gazebo/FR_thigh_controller/state", 2, &GazeboInterface::FR_thigh_state_callback, this);
        sub_joint_msg[5] = nh.subscribe("/" + robot_name + "_gazebo/FR_calf_controller/state", 2, &GazeboInterface::FR_calf_state_callback, this);

        sub_joint_msg[6] = nh.subscribe("/" + robot_name + "_gazebo/RL_hip_controller/state", 2, &GazeboInterface::RL_hip_state_callback, this);
        sub_joint_msg[7] = nh.subscribe("/" + robot_name + "_gazebo/RL_thigh_controller/state", 2, &GazeboInterface::RL_thigh_state_callback, this);
        sub_joint_msg[8] = nh.subscribe("/" + robot_name + "_gazebo/RL_calf_controller/state", 2, &GazeboInterface::RL_calf_state_callback, this);

        sub_joint_msg[9] = nh.subscribe("/" + robot_name + "_gazebo/RR_hip_controller/state", 2, &GazeboInterface::RR_hip_state_callback, this);
        sub_joint_msg[10] = nh.subscribe("/" + robot_name + "_gazebo/RR_thigh_controller/state", 2, &GazeboInterface::RR_thigh_state_callback, this);
        sub_joint_msg[11] = nh.subscribe("/" + robot_name + "_gazebo/RR_calf_controller/state", 2, &GazeboInterface::RR_calf_state_callback, this);

        sub_foot_contact_msg[0] = nh.subscribe("/visual/FL_foot_contact/the_force", 2, &GazeboInterface::FL_foot_contact_callback, this);
        sub_foot_contact_msg[1] = nh.subscribe("/visual/FR_foot_contact/the_force", 2, &GazeboInterface::FR_foot_contact_callback, this);
        sub_foot_contact_msg[2] = nh.subscribe("/visual/RL_foot_contact/the_force", 2, &GazeboInterface::RL_foot_contact_callback, this);
        sub_foot_contact_msg[3] = nh.subscribe("/visual/RR_foot_contact/the_force", 2, &GazeboInterface::RR_foot_contact_callback, this);

        acc_x = MovingWindowFilter(5);
        acc_y = MovingWindowFilter(5);
        acc_z = MovingWindowFilter(5);
        gyro_x = MovingWindowFilter(5);
        gyro_y = MovingWindowFilter(5);
        gyro_z = MovingWindowFilter(5);
    }

    bool GazeboInterface::ctrl_update(double dt) {

        if (legged_state.estimator_init == false) {
            std::cout << "fbk_update not called! " << std::endl;
            return true;
        }

        bool joy_run = joy_update(dt);

        // run low level control
        bool basic_run = tau_ctrl_update(dt);

        bool safe_flag = safety_checker.is_safe(legged_state);

        if (safe_flag) {
            send_cmd();
        } else {
            std::cout << "safety check failed, terminate the controller! " << std::endl;
        }
        return joy_run && safe_flag;
    }

    bool GazeboInterface::fbk_update(double t, double dt) {
        return sensor_update(t, dt);
    }

    bool GazeboInterface::send_cmd() {
        if (legged_state.joy.set_default_pos) {
            // double default_pos[12] = {0.0, 0.67, -1.3, -0.0, 0.67, -1.3, 
            //                           0.0, 0.67, -1.3, -0.0, 0.67, -1.3};
            double default_pos[12] = {0.0, 3.927, -1.57,
                                      0.0, 0.785, -1.57,
                                      0.0, 0.785, -1.57,
                                      0.0, 3.927, -1.57};
            for(int i = 0; i < 4; i++){
                low_cmd.motorCmd[i*3 + 0].mode = 0x0A;
                low_cmd.motorCmd[i*3 + 0].Kp = 70;
                low_cmd.motorCmd[i*3 + 0].dq = 0;
                low_cmd.motorCmd[i*3 + 0].Kd = 3;
                low_cmd.motorCmd[i*3 + 0].tau = 0;
                
                low_cmd.motorCmd[i*3 + 1].mode = 0x0A;
                low_cmd.motorCmd[i*3 + 1].Kp = 180;
                low_cmd.motorCmd[i*3 + 1].dq = 0;
                low_cmd.motorCmd[i*3 + 1].Kd = 8;
                low_cmd.motorCmd[i*3 + 1].tau = 0;
                
                low_cmd.motorCmd[i*3 + 2].mode = 0x0A;
                low_cmd.motorCmd[i*3 + 2].Kp = 300;
                low_cmd.motorCmd[i*3 + 2].dq = 0;
                low_cmd.motorCmd[i*3 + 2].Kd = 15;
                low_cmd.motorCmd[i*3 + 2].tau = 0;
            }
            for(int i = 0; i < 12; i++){
                low_cmd.motorCmd[i].q = default_pos[i];
                pub_joint_cmd[i].publish(low_cmd.motorCmd[i]);
            }
        } else {
            // send control cmd to robot via ros topic
            // have to manually do PD control because gazebo only accepts tau command
            for (int i = 0; i < 12; i++) {
                low_cmd.motorCmd[i].mode = 0x0A;
                low_cmd.motorCmd[i].q = 0;
                low_cmd.motorCmd[i].dq = 0;
                low_cmd.motorCmd[i].Kp = 0;
                low_cmd.motorCmd[i].Kd = 0;
                low_cmd.motorCmd[i].tau = legged_state.param.kp_joint(i % LEG_DOF) * (legged_state.ctrl.joint_ang_tgt(i, 0) - legged_state.fbk.joint_pos(i, 0)) +
                                        legged_state.param.kd_joint(i % LEG_DOF) * (legged_state.ctrl.joint_vel_tgt(i, 0) - legged_state.fbk.joint_vel(i, 0)) +
                                        legged_state.ctrl.joint_tau_tgt(i, 0);
                // std::cout << "i: " << i << std::endl;
                // std::cout << "tau from PD " << legged_state.param.kp_joint(i % LEG_DOF) * (legged_state.ctrl.joint_ang_tgt(i, 0) - legged_state.fbk.joint_pos(i, 0)) +
                //                                legged_state.param.kd_joint(i % LEG_DOF) * (legged_state.ctrl.joint_vel_tgt(i, 0) - legged_state.fbk.joint_vel(i, 0)) << std::endl;
                // std::cout << "tau from ID" << legged_state.ctrl.joint_tau_tgt(i, 0) << std::endl;
                pub_joint_cmd[i].publish(low_cmd.motorCmd[i]);
            }
        }
        return true;
    }

    // callback functions
    void GazeboInterface::gt_pose_callback(const nav_msgs::Odometry::ConstPtr &odom) {
        // update
        if (legged_state.param.kf_type == 0) {
            legged_state.fbk.torso_quat = Eigen::Quaterniond(odom->pose.pose.orientation.w,
                                                             odom->pose.pose.orientation.x,
                                                             odom->pose.pose.orientation.y,
                                                             odom->pose.pose.orientation.z);
            legged_state.fbk.torso_pos_world << odom->pose.pose.position.x,
                                                odom->pose.pose.position.y,
                                                odom->pose.pose.position.z;
            legged_state.fbk.torso_lin_vel_world << odom->twist.twist.linear.x,
                                                    odom->twist.twist.linear.y,
                                                    odom->twist.twist.linear.z;
            legged_state.fbk.torso_ang_vel_world << odom->twist.twist.angular.x,
                                                    odom->twist.twist.angular.y,
                                                    odom->twist.twist.angular.z;
            legged_state.estimator_init = true;
        } else if (legged_state.param.kf_type == 1) {
            legged_state.fbk.torso_quat = Eigen::Quaterniond(odom->pose.pose.orientation.w,
                                                             odom->pose.pose.orientation.x,
                                                             odom->pose.pose.orientation.y,
                                                             odom->pose.pose.orientation.z);
        } else {
            // simulate opti track data
            current_count++;
            if (current_count < DROP_COUNT) {
                return;
            } else {
                double opti_t = odom->header.stamp.toSec();
                Eigen::Matrix<double, 3, 1> opti_pos;
                Eigen::Vector3d opti_euler;
                Eigen::Quaterniond opti_quat(odom->pose.pose.orientation.w,
                                             odom->pose.pose.orientation.x,
                                             odom->pose.pose.orientation.y,
                                             odom->pose.pose.orientation.z);
                opti_euler = Utils::quat_to_euler(opti_quat);
                opti_pos << odom->pose.pose.position.x,
                            odom->pose.pose.position.y,
                            odom->pose.pose.position.z;
                if (first_mocap_received == false) {
                    opti_t_prev = opti_t;
                    initial_opti_euler = opti_euler;
                    initial_opti_pos = opti_pos;
                    initial_opti_pos[2] = 0.0; // height is still absolute
                    first_mocap_received = true;
                } else {
                    double opti_dt = opti_t - opti_t_prev;

                    ekf_data.input_opti_dt(opti_dt);
                    ekf_data.input_opti_pos(opti_pos - initial_opti_pos);
                    ekf_data.input_opti_euler(opti_euler - initial_opti_euler);
                    ekf.update_filter_with_opti(ekf_data);
                }
            }
        }
        return;
    }

    void GazeboInterface::imu_callback(const sensor_msgs::Imu::ConstPtr &imu) {
        legged_state.fbk.torso_lin_acc_body = Eigen::Vector3d(
                acc_x.CalculateAverage(imu->linear_acceleration.x),
                acc_y.CalculateAverage(imu->linear_acceleration.y),
                acc_z.CalculateAverage(imu->linear_acceleration.z)
        );
        legged_state.fbk.torso_ang_vel_body = Eigen::Vector3d(
                gyro_x.CalculateAverage(imu->angular_velocity.x),
                gyro_y.CalculateAverage(imu->angular_velocity.y),
                gyro_z.CalculateAverage(imu->angular_velocity.z)
        );
    }

    // FL
    void GazeboInterface::FL_hip_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state) {
        legged_state.fbk.joint_pos[0] = a1_joint_state.q;
        legged_state.fbk.joint_vel[0] = a1_joint_state.dq;
        legged_state.fbk.joint_tau[0] = a1_joint_state.tauEst;
    }

    void GazeboInterface::FL_thigh_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state) {
        legged_state.fbk.joint_pos[1] = a1_joint_state.q;
        legged_state.fbk.joint_vel[1] = a1_joint_state.dq;
        legged_state.fbk.joint_tau[1] = a1_joint_state.tauEst;
    }

    void GazeboInterface::FL_calf_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state) {
        legged_state.fbk.joint_pos[2] = a1_joint_state.q;
        legged_state.fbk.joint_vel[2] = a1_joint_state.dq;
        legged_state.fbk.joint_tau[2] = a1_joint_state.tauEst;
    }

    // FR
    void GazeboInterface::FR_hip_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state) {
        legged_state.fbk.joint_pos[3] = a1_joint_state.q;
        legged_state.fbk.joint_vel[3] = a1_joint_state.dq;
        legged_state.fbk.joint_tau[3] = a1_joint_state.tauEst;
    }

    void GazeboInterface::FR_thigh_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state) {
        legged_state.fbk.joint_pos[4] = a1_joint_state.q;
        legged_state.fbk.joint_vel[4] = a1_joint_state.dq;
        legged_state.fbk.joint_tau[4] = a1_joint_state.tauEst;
    }

    void GazeboInterface::FR_calf_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state) {
        legged_state.fbk.joint_pos[5] = a1_joint_state.q;
        legged_state.fbk.joint_vel[5] = a1_joint_state.dq;
        legged_state.fbk.joint_tau[5] = a1_joint_state.tauEst;
    }

    // RL
    void GazeboInterface::RL_hip_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state) {
        legged_state.fbk.joint_pos[6] = a1_joint_state.q;
        legged_state.fbk.joint_vel[6] = a1_joint_state.dq;
        legged_state.fbk.joint_tau[6] = a1_joint_state.tauEst;
    }

    void GazeboInterface::RL_thigh_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state) {
        legged_state.fbk.joint_pos[7] = a1_joint_state.q;
        legged_state.fbk.joint_vel[7] = a1_joint_state.dq;
        legged_state.fbk.joint_tau[7] = a1_joint_state.tauEst;
    }

    void GazeboInterface::RL_calf_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state) {
        legged_state.fbk.joint_pos[8] = a1_joint_state.q;
        legged_state.fbk.joint_vel[8] = a1_joint_state.dq;
        legged_state.fbk.joint_tau[8] = a1_joint_state.tauEst;
    }

    // RR
    void GazeboInterface::RR_hip_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state) {
        legged_state.fbk.joint_pos[9] = a1_joint_state.q;
        legged_state.fbk.joint_vel[9] = a1_joint_state.dq;
        legged_state.fbk.joint_tau[9] = a1_joint_state.tauEst;
    }

    void GazeboInterface::RR_thigh_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state) {
        legged_state.fbk.joint_pos[10] = a1_joint_state.q;
        legged_state.fbk.joint_vel[10] = a1_joint_state.dq;
        legged_state.fbk.joint_tau[10] = a1_joint_state.tauEst;
    }

    void GazeboInterface::RR_calf_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state) {
        legged_state.fbk.joint_pos[11] = a1_joint_state.q;
        legged_state.fbk.joint_vel[11] = a1_joint_state.dq;
        legged_state.fbk.joint_tau[11] = a1_joint_state.tauEst;
    }

    // foot contact force, we just use norm
    void GazeboInterface::FL_foot_contact_callback(const geometry_msgs::WrenchStamped &force) {
        Eigen::Vector3d force_vec(force.wrench.force.x, force.wrench.force.y, force.wrench.force.z);
        legged_state.fbk.foot_force[0] = force_vec.norm();
    }

    void GazeboInterface::FR_foot_contact_callback(const geometry_msgs::WrenchStamped &force) {
        Eigen::Vector3d force_vec(force.wrench.force.x, force.wrench.force.y, force.wrench.force.z);
        legged_state.fbk.foot_force[1] = force_vec.norm();
    }

    void GazeboInterface::RL_foot_contact_callback(const geometry_msgs::WrenchStamped &force) {
        Eigen::Vector3d force_vec(force.wrench.force.x, force.wrench.force.y, force.wrench.force.z);
        legged_state.fbk.foot_force[2] = force_vec.norm();
    }

    void GazeboInterface::RR_foot_contact_callback(const geometry_msgs::WrenchStamped &force) {
        Eigen::Vector3d force_vec(force.wrench.force.x, force.wrench.force.y, force.wrench.force.z);
        legged_state.fbk.foot_force[3] = force_vec.norm();
    }

}  // namespace legged
