//
//  Created by chiyenl on 05/5/22
//

#pragma once

#include <iostream>
#include <string>
#include <chrono>
#include <unordered_map>

#include <ros/console.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>

#include "LeggedParams.h"
#include "LeggedState.h"
#include "Utils.h"

using namespace std;

namespace legged {

    class LeggedLogger {

    public:
        LeggedLogger();

        LeggedLogger(ros::NodeHandle &_nh) {
            nh_ = _nh;

            // Initialize foot debug publisher
            string prefix = "/a1_debug";
            for (size_t i = 0; i < NUM_LEG; ++i) {
                string topic_name = prefix + foot_name_map_[i] + "/pose";
                string topic_name_target = prefix + foot_name_map_[i] + "/target";

                pub_foot_pose_[i] = nh_.advertise<visualization_msgs::Marker>(topic_name, 100);
                pub_foot_pose_target_[i] = nh_.advertise<visualization_msgs::Marker>(topic_name_target, 100);

                // initialize marker info
                foot_marker_[i].header.frame_id = "world";
                foot_marker_[i].ns = "basic_shapes";
                foot_marker_[i].id = 10 + i;
                foot_marker_[i].type = visualization_msgs::Marker::CYLINDER;
                foot_marker_[i].action = visualization_msgs::Marker::ADD;

                foot_marker_[i].scale.x = 0.08;
                foot_marker_[i].scale.y = 0.08;
                foot_marker_[i].scale.z = 0.02;
                foot_marker_[i].pose.orientation.x = 0.0;
                foot_marker_[i].pose.orientation.y = 0.0;
                foot_marker_[i].pose.orientation.z = 0.0;
                foot_marker_[i].pose.orientation.w = 1.0;

                foot_marker_[i].color.r = 1.0f;
                foot_marker_[i].color.g = 0.0f;
                foot_marker_[i].color.b = 0.0f;
                foot_marker_[i].color.a = 1.0;

                foot_marker_target_[i].header.frame_id = "world";
                foot_marker_target_[i].ns = "basic_shapes";
                foot_marker_target_[i].id = 10 + i;
                foot_marker_target_[i].type = visualization_msgs::Marker::CYLINDER;
                foot_marker_target_[i].action = visualization_msgs::Marker::ADD;

                foot_marker_target_[i].scale.x = 0.08;
                foot_marker_target_[i].scale.y = 0.08;
                foot_marker_target_[i].scale.z = 0.02;
                foot_marker_target_[i].pose.orientation.x = 0.0;
                foot_marker_target_[i].pose.orientation.y = 0.0;
                foot_marker_target_[i].pose.orientation.z = 0.0;
                foot_marker_target_[i].pose.orientation.w = 1.0;

                foot_marker_target_[i].color.r = 0.0f;
                foot_marker_target_[i].color.g = 1.0f;
                foot_marker_target_[i].color.b = 0.0f;
                foot_marker_target_[i].color.a = 1.0;
            }

            // Initialize robot state publisher
            pub_torso_pose_ = nh_.advertise<nav_msgs::Odometry>(prefix + "/odom", 100);
            pub_euler_ = nh_.advertise<geometry_msgs::Vector3Stamped>(prefix + "/euler", 100);
            pub_joint_data_ = nh_.advertise<sensor_msgs::JointState>(prefix + "/joint", 100);
            pub_joint_data_d_ = nh_.advertise<sensor_msgs::JointState>(prefix + "/joint_d", 100);
            joint_data_.name = {"FL_hip", "FL_thigh", "FL_calf", "FR_hip", "FR_thigh", "FR_calf", "RL_hip", "RL_thigh", "RL_calf", "RR_hip", "RR_thigh", "RR_calf"};
            joint_data_.effort.resize(12);
            joint_data_.position.resize(12);
            joint_data_.velocity.resize(12);
            joint_data_d_.name = {"FL_hip", "FL_thigh", "FL_calf", "FR_hip", "FR_thigh", "FR_calf", "RL_hip", "RL_thigh", "RL_calf", "RR_hip", "RR_thigh", "RR_calf"};
            joint_data_d_.effort.resize(12);
            joint_data_d_.position.resize(12);
            joint_data_d_.velocity.resize(12);

            // foot poses
            pub_FL_pose_ = nh_.advertise<nav_msgs::Odometry>(prefix + "/FL_pose", 100);
            pub_FR_pose_ = nh_.advertise<nav_msgs::Odometry>(prefix + "/FR_pose", 100);
            pub_RL_pose_ = nh_.advertise<nav_msgs::Odometry>(prefix + "/RL_pose", 100);
            pub_RR_pose_ = nh_.advertise<nav_msgs::Odometry>(prefix + "/RR_pose", 100);
            pub_FL_pose_d_ = nh_.advertise<nav_msgs::Odometry>(prefix + "/FL_pose_d", 100);
            pub_FR_pose_d_ = nh_.advertise<nav_msgs::Odometry>(prefix + "/FR_pose_d", 100);
            pub_RL_pose_d_ = nh_.advertise<nav_msgs::Odometry>(prefix + "/RL_pose_d", 100);
            pub_RR_pose_d_ = nh_.advertise<nav_msgs::Odometry>(prefix + "/RR_pose_d", 100);

            // Initialize desired state publisher
            pub_torso_pose_d_ = nh_.advertise<nav_msgs::Odometry>(prefix + "/odom_d", 100);
            pub_euler_d_ = nh_.advertise<geometry_msgs::Vector3Stamped>(prefix + "/euler_d", 100);

            // Initialize ground reaction forces
            pub_grf_ = nh_.advertise<sensor_msgs::JointState>(prefix + "/stance_foot_forces", 100);
            pub_swing_forces_ = nh_.advertise<sensor_msgs::JointState>(prefix + "/swing_foot_forces", 100);
            grf_msg_.name = {"fx_FL", "fy_FL", "fz_FL", "fx_FR", "fy_FR", "fz_FR", "fx_RL", "fy_RL", "fz_RL", "fx_RR", "fy_RR", "fz_RR"};
            swing_forces_msg_.name = {"fx_FL", "fy_FL", "fz_FL", "fx_FR", "fy_FR", "fz_FR", "fx_RL", "fy_RL", "fz_RL", "fx_RR", "fy_RR", "fz_RR"};
            grf_msg_.effort.resize(12);
            swing_forces_msg_.effort.resize(12);

            // Initialize contact forces
            pub_contact_forces_ = nh_.advertise<sensor_msgs::JointState>(prefix + "/contact_forces", 100);
            contact_forces_msg_.name = {"FL", "FR", "RL", "RR"};
            contact_forces_msg_.position.resize(4);
            contact_forces_msg_.velocity.resize(4);
            contact_forces_msg_.effort.resize(4);

            // Compare MPC contact force with feedback from the robot
            pub_mpc_contact_forces_ = nh_.advertise<sensor_msgs::JointState>(prefix + "/mpc_contact_forces", 100);
            mpc_contact_forces_msg_.name = {"FL", "FR", "RL", "RR"};
            mpc_contact_forces_msg_.position.resize(4);
            mpc_contact_forces_msg_.velocity.resize(4);
            mpc_contact_forces_msg_.effort.resize(4);

            pub_foot_force_tauEst_ = nh_.advertise<sensor_msgs::JointState>(prefix + "/tauEst_contact_forces", 100);
            foot_force_tauEst_msg_.name = {"FL", "FR", "RL", "RR"};
            foot_force_tauEst_msg_.position.resize(4);
            foot_force_tauEst_msg_.velocity.resize(4);
            foot_force_tauEst_msg_.effort.resize(4);

        }

        void publish_state(LeggedState &state) {
            // Fill out the robot pose
            auto stamp_now = ros::Time::now();
            odom_.header.stamp = stamp_now;
            odom_.pose.pose.position.x = state.fbk.torso_pos_world(0);
            odom_.pose.pose.position.y = state.fbk.torso_pos_world(1);
            odom_.pose.pose.position.z = state.fbk.torso_pos_world(2);

            odom_.pose.pose.orientation.w = state.fbk.torso_quat.w();
            odom_.pose.pose.orientation.x = state.fbk.torso_quat.x();
            odom_.pose.pose.orientation.y = state.fbk.torso_quat.y();
            odom_.pose.pose.orientation.z = state.fbk.torso_quat.z();

            odom_.twist.twist.linear.x = state.fbk.torso_lin_vel_world(0);
            odom_.twist.twist.linear.y = state.fbk.torso_lin_vel_world(1);
            odom_.twist.twist.linear.z = state.fbk.torso_lin_vel_world(2);

            odom_.twist.twist.angular.x = state.fbk.torso_ang_vel_body(0);
            odom_.twist.twist.angular.y = state.fbk.torso_ang_vel_body(1);
            odom_.twist.twist.angular.z = state.fbk.torso_ang_vel_body(2);

            euler_.vector.x = state.fbk.torso_euler(0);
            euler_.vector.y = state.fbk.torso_euler(1);
            euler_.vector.z = state.fbk.torso_euler(2);

            // foot poses
            FL_pose_msg_.header.stamp = stamp_now;
            FL_pose_msg_.pose.pose.position.x = state.fbk.foot_pos_world(0, 0);
            FL_pose_msg_.pose.pose.position.y = state.fbk.foot_pos_world(1, 0);
            FL_pose_msg_.pose.pose.position.z = state.fbk.foot_pos_world(2, 0);
            FL_pose_msg_.twist.twist.linear.x = state.fbk.foot_lin_vel_world(0, 0);
            FL_pose_msg_.twist.twist.linear.y = state.fbk.foot_lin_vel_world(1, 0);
            FL_pose_msg_.twist.twist.linear.z = state.fbk.foot_lin_vel_world(2, 0);

            FR_pose_msg_.header.stamp = stamp_now;
            FR_pose_msg_.pose.pose.position.x = state.fbk.foot_pos_world(0, 1);
            FR_pose_msg_.pose.pose.position.y = state.fbk.foot_pos_world(1, 1);
            FR_pose_msg_.pose.pose.position.z = state.fbk.foot_pos_world(2, 1);
            FR_pose_msg_.twist.twist.linear.x = state.fbk.foot_lin_vel_world(0, 1);
            FR_pose_msg_.twist.twist.linear.y = state.fbk.foot_lin_vel_world(1, 1);
            FR_pose_msg_.twist.twist.linear.z = state.fbk.foot_lin_vel_world(2, 1);

            RL_pose_msg_.header.stamp = stamp_now;
            RL_pose_msg_.pose.pose.position.x = state.fbk.foot_pos_world(0, 2);
            RL_pose_msg_.pose.pose.position.y = state.fbk.foot_pos_world(1, 2);
            RL_pose_msg_.pose.pose.position.z = state.fbk.foot_pos_world(2, 2);
            RL_pose_msg_.twist.twist.linear.x = state.fbk.foot_lin_vel_world(0, 2);
            RL_pose_msg_.twist.twist.linear.y = state.fbk.foot_lin_vel_world(1, 2);
            RL_pose_msg_.twist.twist.linear.z = state.fbk.foot_lin_vel_world(2, 2);

            RR_pose_msg_.header.stamp = stamp_now;
            RR_pose_msg_.pose.pose.position.x = state.fbk.foot_pos_world(0, 3);
            RR_pose_msg_.pose.pose.position.y = state.fbk.foot_pos_world(1, 3);
            RR_pose_msg_.pose.pose.position.z = state.fbk.foot_pos_world(2, 3);
            RR_pose_msg_.twist.twist.linear.x = state.fbk.foot_lin_vel_world(0, 3);
            RR_pose_msg_.twist.twist.linear.y = state.fbk.foot_lin_vel_world(1, 3);
            RR_pose_msg_.twist.twist.linear.z = state.fbk.foot_lin_vel_world(2, 3);

            // desired foot poses
            FL_pose_msg_d_.header.stamp = stamp_now;
            FL_pose_msg_d_.pose.pose.position.x = state.ctrl.optimized_state(6 + 0);
            FL_pose_msg_d_.pose.pose.position.y = state.ctrl.optimized_state(6 + 1);
            FL_pose_msg_d_.pose.pose.position.z = state.ctrl.optimized_state(6 + 2);
            FL_pose_msg_d_.twist.twist.linear.x = state.ctrl.optimized_input(12 + 0);
            FL_pose_msg_d_.twist.twist.linear.y = state.ctrl.optimized_input(12 + 1);
            FL_pose_msg_d_.twist.twist.linear.z = state.ctrl.optimized_input(12 + 2);

            FR_pose_msg_d_.header.stamp = stamp_now;
            FR_pose_msg_d_.pose.pose.position.x = state.ctrl.optimized_state(6 + 3);
            FR_pose_msg_d_.pose.pose.position.y = state.ctrl.optimized_state(6 + 4);
            FR_pose_msg_d_.pose.pose.position.z = state.ctrl.optimized_state(6 + 5);
            FR_pose_msg_d_.twist.twist.linear.x = state.ctrl.optimized_input(12 + 3);
            FR_pose_msg_d_.twist.twist.linear.y = state.ctrl.optimized_input(12 + 4);
            FR_pose_msg_d_.twist.twist.linear.z = state.ctrl.optimized_input(12 + 5);

            RL_pose_msg_d_.header.stamp = stamp_now;
            RL_pose_msg_d_.pose.pose.position.x = state.ctrl.optimized_state(6 + 6);
            RL_pose_msg_d_.pose.pose.position.y = state.ctrl.optimized_state(6 + 7);
            RL_pose_msg_d_.pose.pose.position.z = state.ctrl.optimized_state(6 + 8);
            RL_pose_msg_d_.twist.twist.linear.x = state.ctrl.optimized_input(12 + 6);
            RL_pose_msg_d_.twist.twist.linear.y = state.ctrl.optimized_input(12 + 7);
            RL_pose_msg_d_.twist.twist.linear.z = state.ctrl.optimized_input(12 + 8);

            RR_pose_msg_d_.header.stamp = stamp_now;
            RR_pose_msg_d_.pose.pose.position.x = state.ctrl.optimized_state(6 + 9);
            RR_pose_msg_d_.pose.pose.position.y = state.ctrl.optimized_state(6 + 10);
            RR_pose_msg_d_.pose.pose.position.z = state.ctrl.optimized_state(6 + 11);
            RR_pose_msg_d_.twist.twist.linear.x = state.ctrl.optimized_input(12 + 9);
            RR_pose_msg_d_.twist.twist.linear.y = state.ctrl.optimized_input(12 + 10);
            RR_pose_msg_d_.twist.twist.linear.z = state.ctrl.optimized_input(12 + 11);

            // fill out desired pose
            odom_d_.header.stamp = stamp_now;
            odom_d_.pose.pose.position.x = state.ctrl.torso_pos_d(0);
            odom_d_.pose.pose.position.y = state.ctrl.torso_pos_d(1);
            odom_d_.pose.pose.position.z = state.ctrl.torso_pos_d(2);

            odom_d_.pose.pose.orientation.w = state.ctrl.torso_quat_d.w();
            odom_d_.pose.pose.orientation.x = state.ctrl.torso_quat_d.x();
            odom_d_.pose.pose.orientation.y = state.ctrl.torso_quat_d.y();
            odom_d_.pose.pose.orientation.z = state.ctrl.torso_quat_d.z();

            odom_d_.twist.twist.linear.x = state.ctrl.torso_lin_vel_d_world(0);
            odom_d_.twist.twist.linear.y = state.ctrl.torso_lin_vel_d_world(1);
            odom_d_.twist.twist.linear.z = state.ctrl.torso_lin_vel_d_world(2);

            odom_d_.twist.twist.angular.x = state.ctrl.torso_ang_vel_d_body(0);
            odom_d_.twist.twist.angular.y = state.ctrl.torso_ang_vel_d_body(1);
            odom_d_.twist.twist.angular.z = state.ctrl.torso_ang_vel_d_body(2);

            state.ctrl.torso_euler_d = Utils::quatToZyx(state.ctrl.torso_quat_d);
            
            euler_d_.header.stamp = stamp_now;
            euler_d_.vector.x = state.ctrl.torso_euler_d(0);
            euler_d_.vector.y = state.ctrl.torso_euler_d(1);
            euler_d_.vector.z = state.ctrl.torso_euler_d(2);

            // GRFs and joints data
            joint_data_.header.stamp = stamp_now;
            grf_msg_.header.stamp = stamp_now;
            contact_forces_msg_.header.stamp = stamp_now;
            mpc_contact_forces_msg_.header.stamp = stamp_now;
            foot_force_tauEst_msg_.header.stamp = stamp_now;
            swing_forces_msg_.header.stamp = stamp_now;

            for (size_t i = 0; i < NUM_LEG; ++i) {
                foot_marker_[i].header.stamp = stamp_now;
                foot_marker_[i].pose.position.x = state.fbk.foot_pos_world(0, i);
                foot_marker_[i].pose.position.y = state.fbk.foot_pos_world(1, i);
                foot_marker_[i].pose.position.z = state.fbk.foot_pos_world(2, i);

                foot_marker_target_[i].pose.position.x = state.ctrl.foot_pos_target_world(0, i);
                foot_marker_target_[i].pose.position.y = state.ctrl.foot_pos_target_world(1, i);
                foot_marker_target_[i].pose.position.z = state.ctrl.foot_pos_target_world(2, i);

                joint_data_.position[i * 3] = state.fbk.joint_pos[i * 3];
                joint_data_.position[i * 3 + 1] = state.fbk.joint_pos[i * 3 + 1];
                joint_data_.position[i * 3 + 2] = state.fbk.joint_pos[i * 3 + 2];

                joint_data_.velocity[i * 3] = state.fbk.joint_vel[i * 3];
                joint_data_.velocity[i * 3 + 1] = state.fbk.joint_vel[i * 3 + 1];
                joint_data_.velocity[i * 3 + 2] = state.fbk.joint_vel[i * 3 + 2];

                joint_data_.effort[i * 3] = state.fbk.joint_tau[i * 3];
                joint_data_.effort[i * 3 + 1] = state.fbk.joint_tau[i * 3 + 1];
                joint_data_.effort[i * 3 + 2] = state.fbk.joint_tau[i * 3 + 2];

                joint_data_d_.position[i * 3] = state.ctrl.joint_ang_tgt[i * 3];
                joint_data_d_.position[i * 3 + 1] = state.ctrl.joint_ang_tgt[i * 3 + 1];
                joint_data_d_.position[i * 3 + 2] = state.ctrl.joint_ang_tgt[i * 3 + 2];

                joint_data_d_.velocity[i * 3] = state.ctrl.joint_vel_tgt[i * 3];
                joint_data_d_.velocity[i * 3 + 1] = state.ctrl.joint_vel_tgt[i * 3 + 1];
                joint_data_d_.velocity[i * 3 + 2] = state.ctrl.joint_vel_tgt[i * 3 + 2];

                // grf_msg_.effort[i*3] = state.fbk.foot_forces_grf(0, i);
                // grf_msg_.effort[i*3+1] = state.fbk.foot_forces_grf(1, i);
                // grf_msg_.effort[i*3+2] = state.fbk.foot_forces_grf(2, i);

                // swing_forces_msg_.effort[i*3] = state.fbk.foot_forces_kin(0, i);
                // swing_forces_msg_.effort[i*3+1] = state.fbk.foot_forces_kin(1, i);
                // swing_forces_msg_.effort[i*3+2] = state.fbk.foot_forces_kin(2, i);

                pub_foot_pose_[i].publish(foot_marker_[i]);
                pub_foot_pose_target_[i].publish(foot_marker_target_[i]);

                contact_forces_msg_.effort[i] = state.fbk.foot_force[i];
                contact_forces_msg_.position[i] = state.ctrl.plan_contacts[i];
                contact_forces_msg_.velocity[i] = state.ctrl.gait_counter[i];


                mpc_contact_forces_msg_.effort[i] = state.ctrl.optimized_input.segment<3>(i * 3).norm();
                foot_force_tauEst_msg_.effort[i] = state.fbk.foot_force_est.block<3, 1>(0, i).norm();
            }
            // contact_forces_msg_.quaternion.w = state.ctrl.plan_contacts[0];
            // contact_forces_msg_.quaternion.x = state.ctrl.plan_contacts[1];
            // contact_forces_msg_.quaternion.y = state.ctrl.plan_contacts[2];
            // contact_forces_msg_.quaternion.z = state.ctrl.plan_contacts[3];

            // publish
            pub_torso_pose_.publish(odom_);
            pub_torso_pose_d_.publish(odom_d_);

            pub_FL_pose_.publish(FL_pose_msg_);
            pub_FL_pose_d_.publish(FL_pose_msg_d_);
            pub_FR_pose_.publish(FR_pose_msg_);
            pub_FR_pose_d_.publish(FR_pose_msg_d_);
            pub_RL_pose_.publish(RL_pose_msg_);
            pub_RL_pose_d_.publish(RL_pose_msg_d_);
            pub_RR_pose_.publish(RR_pose_msg_);
            pub_RR_pose_d_.publish(RR_pose_msg_d_);

            pub_euler_.publish(euler_);
            pub_euler_d_.publish(euler_d_);
            pub_grf_.publish(grf_msg_);
            pub_joint_data_.publish(joint_data_);
            pub_joint_data_d_.publish(joint_data_d_);
            pub_contact_forces_.publish(contact_forces_msg_);
            pub_mpc_contact_forces_.publish(mpc_contact_forces_msg_);
            pub_foot_force_tauEst_.publish(foot_force_tauEst_msg_);
            pub_swing_forces_.publish(swing_forces_msg_);

            // publish tf
            tf::Transform transform;
            transform.setOrigin(tf::Vector3(state.fbk.torso_pos_world(0), state.fbk.torso_pos_world(1), state.fbk.torso_pos_world(2)));
            tf::Quaternion q;
            q.setW(state.fbk.torso_quat.w());
            q.setX(state.fbk.torso_quat.x());
            q.setY(state.fbk.torso_quat.y());
            q.setZ(state.fbk.torso_quat.z());
            transform.setRotation(q);
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "robot"));
        }

    private:
        ros::NodeHandle nh_;

        // publish robot state
        ros::Publisher pub_torso_pose_; // rigidbody state
        ros::Publisher pub_euler_; // extra publisher for euler angles
        ros::Publisher pub_joint_data_; // joint data
        nav_msgs::Odometry odom_;
        geometry_msgs::Vector3Stamped euler_;
        sensor_msgs::JointState joint_data_;

        // publish desired root state
        ros::Publisher pub_torso_pose_d_; // desired rigidbody state
        ros::Publisher pub_euler_d_; // desired angle pose
        ros::Publisher pub_joint_data_d_; // desired joint data
        nav_msgs::Odometry odom_d_;
        geometry_msgs::Vector3Stamped euler_d_;
        sensor_msgs::JointState joint_data_d_;

        // publish foot poses
        ros::Publisher pub_FL_pose_; // foot FL pose
        ros::Publisher pub_FR_pose_; // foot FR pose
        ros::Publisher pub_RL_pose_; // foot RL pose
        ros::Publisher pub_RR_pose_; // foot RL pose
        ros::Publisher pub_FL_pose_d_; // desired foot FL pose
        ros::Publisher pub_FR_pose_d_; // desired foot FR pose
        ros::Publisher pub_RL_pose_d_; // desired foot RL pose
        ros::Publisher pub_RR_pose_d_; // desired foot RL pose
        nav_msgs::Odometry FL_pose_msg_;
        nav_msgs::Odometry FR_pose_msg_;
        nav_msgs::Odometry RL_pose_msg_;
        nav_msgs::Odometry RR_pose_msg_;
        nav_msgs::Odometry FL_pose_msg_d_;
        nav_msgs::Odometry FR_pose_msg_d_;
        nav_msgs::Odometry RL_pose_msg_d_;
        nav_msgs::Odometry RR_pose_msg_d_;

        // publish ground reaction forces
        ros::Publisher pub_grf_; // ground reactions forces
        ros::Publisher pub_swing_forces_; // swing forces
        sensor_msgs::JointState grf_msg_;
        sensor_msgs::JointState swing_forces_msg_;

        // robot foot pose and target food pose
        ros::Publisher pub_foot_pose_[NUM_LEG];
        ros::Publisher pub_foot_pose_target_[NUM_LEG];
        visualization_msgs::Marker foot_marker_[NUM_LEG];
        visualization_msgs::Marker foot_marker_target_[NUM_LEG];

        // publish contact info
        ros::Publisher pub_contact_forces_;
        ros::Publisher pub_mpc_contact_forces_;
        ros::Publisher pub_foot_force_tauEst_;
        sensor_msgs::JointState contact_forces_msg_;
        sensor_msgs::JointState mpc_contact_forces_msg_;
        sensor_msgs::JointState foot_force_tauEst_msg_;

        // foot name mapping
        unordered_map<int, string> foot_name_map_{{0, "/FL"},
                                                  {1, "/FR"},
                                                  {2, "/RL"},
                                                  {3, "/RR"}};

        // tf
        tf::TransformBroadcaster br;
    };

} // namespace legged
