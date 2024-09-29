#include "interfaces/HardwareInterface.h"

namespace legged {

    HardwareInterface::HardwareInterface(ros::NodeHandle &_nh)
            : safe(UNITREE_LEGGED_SDK::LeggedType::Go1),
              udp(UNITREE_LEGGED_SDK::LOWLEVEL, 8090, "192.168.123.10", 8007),
              BaseInterface(_nh) {

        // for state estimation
        pub_joint_angle = nh.advertise<sensor_msgs::JointState>("/hardware_a1/joint_foot", 100);
        joint_foot_msg.name = {"FL0", "FL1", "FL2",
                               "FR0", "FR1", "FR2",
                               "RL0", "RL1", "RL2",
                               "RR0", "RR1", "RR2",
                               "FL_foot", "FR_foot", "RL_foot", "RR_foot"};
        joint_foot_msg.position.resize(NUM_DOF + NUM_LEG);
        joint_foot_msg.velocity.resize(NUM_DOF + NUM_LEG);
        joint_foot_msg.effort.resize(NUM_DOF + NUM_LEG);

        // imu data
        pub_imu = nh.advertise<sensor_msgs::Imu>("/hardware_a1/imu", 100);

        udp.InitCmdData(cmd);
        udp_init_send();

        // init swap order, very important because unitree use a different order then ours
        swap_joint_indices << 3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8;
        swap_foot_indices << 1, 0, 3, 2;

        for (int i = 0; i < NUM_LEG; i++) {
            foot_force_filters[i] = MovingWindowFilter(FOOT_FILTER_WINDOW_SIZE);
        }

        for (int i = 0; i < NUM_DOF; i++) {
            joint_vel_filters[i] = MovingWindowFilter(10);
        }

        // mocap callback 
        mocap_sub = _nh.subscribe("/natnet_ros/zixin_go1/pose", 120, &HardwareInterface::opti_callback, this);
    }

    bool HardwareInterface::ctrl_update(double dt) {

        if (legged_state.estimation_inited == false) {
            std::cout << "fbk_update not called! " << std::endl;
            return true;
        }
        bool joy_run = joy_update(dt);

        // run low level control 
        bool basic_run = tau_ctrl_update(dt);

        // send to hardware
        bool safe_flag = safety_checker.is_safe(legged_state);
        if (safe_flag) {
            send_cmd();
        } else {
            std::cout << "safety check failed, terminate the controller! " << std::endl;
        }

        return joy_run && basic_run && safe_flag;
    }

    bool HardwareInterface::fbk_update(double t, double dt) {
        // get sensor feedback & update state estimator
        receive_low_state(dt);

        bool sensor_run = sensor_update(t, dt);
        return sensor_run;
    }


    bool HardwareInterface::send_cmd() {
        // send control cmd to robot via unitree hardware interface
        // notice a1_ctrl_states.joint_torques uses order FL, FR, RL, RR
        // notice cmd uses order FR, FL, RR, RL
        // swap_joint_indices << 3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8;

        cmd.levelFlag = UNITREE_LEGGED_SDK::LOWLEVEL;

        if (legged_state.joy.set_default_pos) {
            double Kp = 20.0;
            double Kd = 2.0;
            // double default_pos[12] = {0.0, 0.67, -1.3,
            //                           0.0, 0.67, -1.3,
            //                           0.0, 0.67, -1.3,
            //                           0.0, 0.67, -1.3};
            double default_pos[12] = {0.0, 3.85, -1.0,
                                      0.0, 0.71, -1.25,
                                      0.0, 0.71, -1.25,
                                      0.0, 3.8, -1.25};
            double default_tau[12] = {0.0, 0.0, 0.0,
                                      0.0, 0.0, 0.0,
                                      0.0, 0.0, 0.0,
                                      0.0, 0.0, 0.0};
            double default_Kp[12] = {Kp, 15, 15,
                                     Kp, Kp, Kp,
                                     Kp, Kp, Kp,
                                     Kp, Kp, Kp};
            double default_Kd[12] = {Kd, Kd, Kd,
                                     Kd, Kd, Kd,
                                     Kd, Kd, Kd,
                                     Kd, Kd, Kd};
            for(int i = 0; i < 12; i++){
                cmd.motorCmd[i].mode = 0x0A;
                cmd.motorCmd[i].Kp = default_Kp[i];
                cmd.motorCmd[i].Kd = default_Kd[i];
                cmd.motorCmd[i].q = default_pos[i];
                cmd.motorCmd[i].dq = 0;
                cmd.motorCmd[i].tau = default_tau[i];
            }
        } else if (legged_state.joy.zero_torque_mode) {
            for(int i = 0; i < 12; i++){
                cmd.motorCmd[i].mode = 0x0A;
                cmd.motorCmd[i].Kp = 0;
                cmd.motorCmd[i].Kd = 0;
                cmd.motorCmd[i].q = 0;
                cmd.motorCmd[i].dq = 0;
                cmd.motorCmd[i].tau = 0;
            }
        } else {
            for (int i = 0; i < NUM_DOF; i++) {
                int swap_i = swap_joint_indices(i);
                int swap_leg = swap_foot_indices(i / LEG_DOF);
                cmd.motorCmd[i].mode = 0x0A;

                cmd.motorCmd[i].q = legged_state.ctrl.joint_ang_tgt(swap_i);
                cmd.motorCmd[i].Kp = legged_state.param.kp_joint(swap_i % LEG_DOF);
                cmd.motorCmd[i].dq = legged_state.ctrl.joint_vel_tgt(swap_i);
                cmd.motorCmd[i].Kd = legged_state.param.kd_joint(swap_i % LEG_DOF);

                cmd.motorCmd[i].tau = legged_state.ctrl.joint_tau_tgt(swap_i);
            }
        }
        // safe.PositionLimit(cmd);
        // safe.PowerProtect(cmd, unitree_state, 10);
        udp.SetSend(cmd);
        udp.Send();

        return true;
    }

    void HardwareInterface::udp_init_send() {
        cmd.levelFlag = UNITREE_LEGGED_SDK::LOWLEVEL;
        for (int i = 0; i < NUM_DOF; i++) {
            cmd.motorCmd[i].mode = 0x0A; // motor switch to servo (PMSM) mode
            cmd.motorCmd[i].q = UNITREE_LEGGED_SDK::PosStopF; // shut down position control
            cmd.motorCmd[i].Kp = 0;
            cmd.motorCmd[i].dq = UNITREE_LEGGED_SDK::VelStopF; // shut down velocity control
            cmd.motorCmd[i].Kd = 0;
            cmd.motorCmd[i].tau = 0;
        }
        // safe.PositionLimit(cmd);
        udp.SetSend(cmd);
        udp.Send();
    }

    void HardwareInterface::receive_low_state(double dt) {
        udp.Recv();
        udp.GetRecv(unitree_state);

        // load state from unitree_state to legged_state
        if (legged_state.param.kf_type == 1) {
            legged_state.fbk.torso_quat = Eigen::Quaterniond(unitree_state.imu.quaternion[0],
                                                             unitree_state.imu.quaternion[1],
                                                             unitree_state.imu.quaternion[2],
                                                             unitree_state.imu.quaternion[3]);
        }

        legged_state.fbk.torso_lin_acc_body = Eigen::Vector3d(unitree_state.imu.accelerometer[0], unitree_state.imu.accelerometer[1], unitree_state.imu.accelerometer[2]);
        legged_state.fbk.torso_ang_vel_body = Eigen::Vector3d(unitree_state.imu.gyroscope[0], unitree_state.imu.gyroscope[1], unitree_state.imu.gyroscope[2]);
        // std::cout << "torso_lin_acc_body: " << legged_state.fbk.torso_lin_acc_body.transpose() << std::endl;

        for (int i = 0; i < NUM_DOF; ++i) {
            int swap_i = swap_joint_indices(i);
            legged_state.fbk.joint_vel[i] = joint_vel_filters[i].CalculateAverage(unitree_state.motorState[swap_i].dq);
            // legged_state.fbk.joint_vel[i] = (unitree_state.motorState[swap_i].q - legged_state.fbk.joint_pos[i])/dt;
            legged_state.fbk.joint_pos[i] = unitree_state.motorState[swap_i].q;
            legged_state.fbk.joint_tau[i] = unitree_state.motorState[swap_i].tauEst;
        }

        if (legged_state.fbk.record_foot_force_bias == false) {
            for (int i = 0; i < NUM_LEG; ++i) {
                int swap_i = swap_foot_indices(i);
                legged_state.fbk.foot_sensor_bias[i] = unitree_state.footForce[swap_i];
            }
            legged_state.fbk.record_foot_force_bias = true;
        }

        // foot force, add a filter here
        for (int i = 0; i < NUM_LEG; ++i) {
            int swap_i = swap_foot_indices(i);
            double value = static_cast<double>(unitree_state.footForce[swap_i] - legged_state.fbk.foot_sensor_bias[i]);
            legged_state.fbk.foot_force[i] = foot_force_filters[i].CalculateAverage(value);
        }

        // publish state to ros for other nodes to use
        // publish joint angle and foot force
        for (int i = 0; i < NUM_DOF; ++i) {
            joint_foot_msg.position[i] = legged_state.fbk.joint_pos[i];
            joint_foot_msg.velocity[i] = legged_state.fbk.joint_vel[i];
            joint_foot_msg.effort[i] = legged_state.fbk.joint_tau[i];
        }

        for (int i = 0; i < NUM_LEG; ++i) {
            // publish plan contacts to help state estimation
            joint_foot_msg.velocity[NUM_DOF + i] = legged_state.ctrl.plan_contacts[i];
            joint_foot_msg.effort[NUM_DOF + i] = legged_state.fbk.foot_force[i];
        }

        imu_msg.angular_velocity.x = unitree_state.imu.gyroscope[0];
        imu_msg.angular_velocity.y = unitree_state.imu.gyroscope[1];
        imu_msg.angular_velocity.z = unitree_state.imu.gyroscope[2];

        imu_msg.linear_acceleration.x = unitree_state.imu.accelerometer[0];
        imu_msg.linear_acceleration.y = unitree_state.imu.accelerometer[1];
        imu_msg.linear_acceleration.z = unitree_state.imu.accelerometer[2];

        joint_foot_msg.header.stamp = ros::Time::now();
        imu_msg.header.stamp = joint_foot_msg.header.stamp;

        pub_joint_angle.publish(joint_foot_msg);
        pub_imu.publish(imu_msg);
    }

    void HardwareInterface::opti_callback(const geometry_msgs::PoseStamped::ConstPtr &opti_msg) {
        double opti_t = opti_msg->header.stamp.toSec();
        Eigen::Matrix<double, 3, 1> opti_pos;
        opti_pos << opti_msg->pose.position.x, opti_msg->pose.position.y, opti_msg->pose.position.z;

        Eigen::Vector3d opti_euler;
        Eigen::Quaterniond opti_quat(opti_msg->pose.orientation.w,
                                     opti_msg->pose.orientation.x,
                                     opti_msg->pose.orientation.y,
                                     opti_msg->pose.orientation.z);
        opti_euler = Utils::quat_to_euler(opti_quat);

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
        legged_state.fbk.torso_quat = opti_quat;
        legged_state.fbk.torso_pos_world[0] = opti_pos[0];
        legged_state.fbk.torso_pos_world[1] = opti_pos[1];
        legged_state.fbk.torso_pos_world[2] = opti_pos[2];
    }

} // namespace legged