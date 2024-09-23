#include <mutex>
#include <thread>
#include <iostream>

#include "GazeboInterface.h"
#include "LeggedMpc.h"
#include "GyrostatQuatMpc.h"
#include "GyrostatEulerMpc.h"
#include "RobotState.h"

using namespace std;
using namespace robot;
using namespace mpc;

std::mutex mtx;

int main(int argc, char **argv) {
    ros::init(argc, argv, "main");
    ros::NodeHandle nh;

    // Read ROS parameters
    string robot_name;
    string mpc_type;
    nh.param<string>("robot_name", robot_name, "a1_rw");
    nh.param<string>("mpc_type", mpc_type, "quat");

    auto intf_ptr = std::make_unique<GazeboInterface>(nh, robot_name);

    ros::Duration(0.5).sleep();

    // Thread 1: MPC
    // auto mpc_ptr = std::make_unique<GyrostatQuatMpc>(intf_ptr->robot_state);
    // auto mpc_ptr = std::make_unique<GyrostatEulerMpc>(intf_ptr->robot_state);
    std::unique_ptr<LeggedMpc> mpc_ptr = std::make_unique<GyrostatQuatMpc>(intf_ptr->robot_state);
    if (mpc_type == "euler") {
        mpc_ptr = std::make_unique<GyrostatEulerMpc>(intf_ptr->robot_state);
    }

    thread mpc_thread([&]() {
        ros::Rate loop_rate(100);
        while (ros::ok()) {
            // mtx.lock();
            mpc_ptr->update(intf_ptr->robot_state);
            loop_rate.sleep();
            // mtx.unlock();
        }
    });

    // Thread 2: send command
    thread send_cmd_thread([&]() {
        ros::Rate loop_rate(1000);
        while (ros::ok()) {
            // mtx.lock();
            intf_ptr->ctrl_update();
            loop_rate.sleep();
            // mtx.unlock();
        }
    });

    // Thread 3: feedback
    thread feedback_thread([&]() {
        ros::Rate loop_rate(1000);
        while (ros::ok()) {
            // mtx.lock();
            intf_ptr->fbk_update();
            loop_rate.sleep();
            // mtx.unlock();
        }
    });

    ros::spin();

    mpc_thread.join();
    send_cmd_thread.join();
    feedback_thread.join();

    return 0;
}
