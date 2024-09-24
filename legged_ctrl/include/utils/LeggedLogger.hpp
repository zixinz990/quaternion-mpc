#pragma once

#include <iostream>
#include <string>
#include <chrono>
#include <unordered_map>

#include <ros/console.h>
#include <ros/ros.h>

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
            string prefix = "/debug";

            pub_torso_odom = nh_.advertise<nav_msgs::Odometry>(prefix + "/torso_odom", 100);
            pub_torso_odom_d = nh_.advertise<nav_msgs::Odometry>(prefix + "/torso_odom_d", 100);

            pub_mpc_grf = nh_.advertise<sensor_msgs::JointState>(prefix + "/mpc_grf", 100);
            pub_mpc_time = nh_.advertise<std_msgs::Float64>(prefix + "/mpc_time", 100);

            mpc_grf_msg.name = {"FL", "FR", "RL", "RR"};
            mpc_grf_msg.position.resize(4); // contact schedule, 0 or 1
            mpc_grf_msg.velocity.resize(4);
            mpc_grf_msg.effort.resize(4);   // GRF
        }

        void publish_state(LeggedState &state) {
            auto stamp_now = ros::Time::now();

            // Torso odometry
            torso_odom_msg.header.stamp = stamp_now;

            torso_odom_msg.pose.pose.position.x = state.fbk.torso_pos_world(0);
            torso_odom_msg.pose.pose.position.y = state.fbk.torso_pos_world(1);
            torso_odom_msg.pose.pose.position.z = state.fbk.torso_pos_world(2);

            torso_odom_msg.pose.pose.orientation.w = state.fbk.torso_quat.w();
            torso_odom_msg.pose.pose.orientation.x = state.fbk.torso_quat.x();
            torso_odom_msg.pose.pose.orientation.y = state.fbk.torso_quat.y();
            torso_odom_msg.pose.pose.orientation.z = state.fbk.torso_quat.z();

            torso_odom_msg.twist.twist.linear.x = state.fbk.torso_lin_vel_body(0);
            torso_odom_msg.twist.twist.linear.y = state.fbk.torso_lin_vel_body(1);
            torso_odom_msg.twist.twist.linear.z = state.fbk.torso_lin_vel_body(2);

            torso_odom_msg.twist.twist.angular.x = state.fbk.torso_ang_vel_body(0);
            torso_odom_msg.twist.twist.angular.y = state.fbk.torso_ang_vel_body(1);
            torso_odom_msg.twist.twist.angular.z = state.fbk.torso_ang_vel_body(2);

            // Desired torso odometry
            torso_odom_d_msg.header.stamp = stamp_now;

            torso_odom_d_msg.pose.pose.position.x = state.ctrl.torso_pos_d_body(0);
            torso_odom_d_msg.pose.pose.position.y = state.ctrl.torso_pos_d_body(1);
            torso_odom_d_msg.pose.pose.position.z = state.ctrl.torso_pos_d_body(2);

            torso_odom_d_msg.pose.pose.orientation.w = state.ctrl.torso_quat_d.w();
            torso_odom_d_msg.pose.pose.orientation.x = state.ctrl.torso_quat_d.x();
            torso_odom_d_msg.pose.pose.orientation.y = state.ctrl.torso_quat_d.y();
            torso_odom_d_msg.pose.pose.orientation.z = state.ctrl.torso_quat_d.z();

            torso_odom_d_msg.twist.twist.linear.x = state.ctrl.torso_lin_vel_d_body(0);
            torso_odom_d_msg.twist.twist.linear.y = state.ctrl.torso_lin_vel_d_body(1);
            torso_odom_d_msg.twist.twist.linear.z = state.ctrl.torso_lin_vel_d_body(2);

            torso_odom_d_msg.twist.twist.angular.x = state.ctrl.torso_ang_vel_d_body(0);
            torso_odom_d_msg.twist.twist.angular.y = state.ctrl.torso_ang_vel_d_body(1);
            torso_odom_d_msg.twist.twist.angular.z = state.ctrl.torso_ang_vel_d_body(2);

            // GRFs from MPC (in world frame)
            mpc_grf_msg.header.stamp = stamp_now;

            for (int i = 0; i < NUM_LEG; ++i) {
                mpc_grf_msg.position[i] = state.ctrl.plan_contacts[i];
                mpc_grf_msg.effort[i] = state.ctrl.mpc_grf_world.segment<3>(i * 3).norm();
            }

            // MPC solve time
            mpc_time_msg.data = state.fbk.mpc_time;

            // Publish
            pub_torso_odom.publish(torso_odom_msg);
            pub_torso_odom_d.publish(torso_odom_d_msg);
            pub_mpc_grf.publish(mpc_grf_msg);
            pub_mpc_time.publish(mpc_time_msg);

            // TF
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

        // Publish torso odometry
        // Position: world frame
        // Orientation: world frame
        // Linear velocity: relative frame
        // Angular velocity: body frame
        ros::Publisher pub_torso_odom;   // from ground truth or estimator
        ros::Publisher pub_torso_odom_d; // desired odometry

        nav_msgs::Odometry torso_odom_msg;
        nav_msgs::Odometry torso_odom_d_msg;

        // Publish MPC-related data
        ros::Publisher pub_mpc_grf;  // ground reaction forces in world frame
        ros::Publisher pub_mpc_time; // MPC solve time

        sensor_msgs::JointState mpc_grf_msg;
        std_msgs::Float64 mpc_time_msg;

        // Publish TF
        tf::TransformBroadcaster br;
    };

} // namespace legged
