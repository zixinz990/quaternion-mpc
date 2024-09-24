#include <chrono>
#include <iostream>
#include <iomanip>
#include <memory>
#include <mutex>
#include <thread>
#include <pthread.h>
#include <sched.h>

#include <ros/ros.h>
#include <ros/console.h>

#include "LeggedParams.h"
#include "LeggedState.h"
#include "interfaces/GazeboInterface.h"
#include "interfaces/HardwareInterface.h"
#include "mpc/LeggedMpc.h"
#include "mpc/ConvexMpc.h"
#include "mpc/QuatMpc.h"
#include "utils/LeggedLogger.hpp"

std::mutex mtx;

int main(int argc, char **argv) {
    ros::init(argc, argv, "main");
    ros::NodeHandle nh;

    // Set logging level to Debug
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
        ros::console::notifyLoggerLevelsChanged();
    }

    // Set ROS publishers (for debug)
    std::unique_ptr <legged::LeggedLogger> logger = std::unique_ptr<legged::LeggedLogger>(new legged::LeggedLogger(nh));

    // Read ROS parameters
    bool use_sim_time;
    int robot_type;
    int controller_type;
    std::string env_type;
    double mpc_update_period;

    if (ros::param::get("/use_sim_time", use_sim_time)) {
        if (use_sim_time) {
            std::cout << "Run in simulation!" << std::endl;
            env_type = string("simulation");
        } else {
            std::cerr << "Run on hardware!" << std::endl;
            env_type = string("hardware");
        }
    } else {
        std::cout << "Key parameter use_sim_time is not set" << std::endl;
        return -1;
    }

    ros::param::get("/robot_type", robot_type);
    ros::param::get("/controller_type", controller_type);

    std::cout << "Run type: \t" << env_type << std::endl;
    std::cout << "Control method: \t" << controller_type << std::endl;
    if (!use_sim_time) {
        std::cout << "Press ENTER to confirm and continue to start hardware run..." << std::endl;
        std::cin.get();
    }

    ros::param::get("/mpc_update_period", mpc_update_period);

    // Initialize interface 
    std::unique_ptr <legged::BaseInterface> intf;
    if (use_sim_time) {
        intf = std::unique_ptr<legged::GazeboInterface>(new legged::GazeboInterface(nh, robot_type));
    } else {
        intf = std::unique_ptr<legged::HardwareInterface>(new legged::HardwareInterface(nh));
    }

    if (intf->legged_state.param.kf_type == 0 && !use_sim_time) {
        std::cout << "Run on hardware but kf_type is set to 0" << std::endl;
        return -1;
    }

    // Wait for the ROS clock to be ready
    ros::Duration(0.5).sleep();

    sched_param sch_params;
    int policy;

    /// THREAD 1: SOLVE MODEL PREDICTIVE CONTROL ///
    std::thread mpc_thread([&]() {
        std::unique_ptr <legged::LeggedMpc> mpc_ptr;
        if (controller_type == 0) {
            // @todo: Add QP
        } else if (controller_type == 1) {
            mpc_ptr = std::make_unique<legged::ConvexMpc>(intf->get_legged_state());
        } else if (controller_type == 2) {
            mpc_ptr = std::make_unique<legged::QuatMpc>(intf->get_legged_state());
        } else {
            std::cerr << "Invalid controller type!" << std::endl;
            ros::shutdown();
            std::terminate();
        }
        while (ros::ok()) {
            // Record start time
            std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();

            // Solve MPC
            mtx.lock();
            mpc_ptr->update(intf->get_legged_state());
            mtx.unlock();

            // Record end time
            std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();

            // Sleep for a while to control the loop rate 
            std::chrono::duration<double, std::milli> time_span = end - start;
            ros::Duration sleep_duration(5.0 / 1000.0 - time_span.count() / 1000.0);
            if (sleep_duration.toSec() > 0) {
                sleep_duration.sleep();
            }
        }
    });

    // Set priority for mpc_thread
    pthread_t mpc_native_thread = mpc_thread.native_handle();
    pthread_getschedparam(mpc_native_thread, &policy, &sch_params);
    sch_params.sched_priority = 50;
    if(pthread_setschedparam(mpc_native_thread, SCHED_FIFO, &sch_params)) {
        std::cerr << "Failed to set thread priority for mpc_thread" << std::endl;
    }

    /// THREAD 2: UPDATE STATES, RUN WHOLE-BODY CONTROL, SEND COMMANDS ///
    std::thread low_level_thread([&]() {
        while (ros::ok()) {
            // Record start time
            std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();

            // Update states, run whole-body control, send commands
            // mtx.lock();
            bool main_update_running = intf->ctrl_update(LOW_LEVEL_CTRL_PERIOD / 1000.0);
            // mtx.unlock();
            if (!main_update_running) {
                std::cerr << "low_level_thread is terminated because of errors." << std::endl;
                ros::shutdown();
                std::terminate();
                break;
            }
            std::chrono::high_resolution_clock::time_point end_ = std::chrono::high_resolution_clock::now();

            // Record end time
            std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();

            // Sleep for a while to control the loop rate
            std::chrono::duration<double, std::milli> time_span = end - start;
            ros::Duration sleep_duration(LOW_LEVEL_CTRL_PERIOD / 1000.0 - time_span.count() / 1000.0);
            if (sleep_duration.toSec() > 0) {
                sleep_duration.sleep();
            }
        }
    });

    // Set priority for low_level_thread
    pthread_t low_level_native_thread = low_level_thread.native_handle();
    pthread_getschedparam(low_level_native_thread, &policy, &sch_params);
    sch_params.sched_priority = 25;
    if(pthread_setschedparam(low_level_native_thread, SCHED_FIFO, &sch_params)) {
        std::cerr << "Failed to set thread priority for low_level_thread" << std::endl;
    }

    /// THREAD 3: GET FEEDBACK ///
    std::thread feedback_thread([&]() {
        std::chrono::high_resolution_clock::time_point thread_start = std::chrono::high_resolution_clock::now();
        while (ros::ok()) {
            // Record start time
            std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();

            // Get feedback & publish to ROS
            mtx.lock();
            std::chrono::high_resolution_clock::time_point now = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::milli> elapsed = now - thread_start;
            bool fbk_running = intf->fbk_update(elapsed.count() / 1000.0, FEEDBACK_PERIOD / 1000.0);
            logger->publish_state(intf->get_legged_state());
            mtx.unlock();
            if (!fbk_running) {
                std::cerr << "feedback_thread is terminated because of errors." << std::endl;
                ros::shutdown();
                std::terminate();
                break;
            }

            // Record end time
            std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();

            // Sleep for a while to control the loop rate 
            std::chrono::duration<double, std::milli> time_span = end - start;
            ros::Duration sleep_duration(FEEDBACK_PERIOD / 1000.0 - time_span.count() / 1000.0);
            if (sleep_duration.toSec() > 0) {
                sleep_duration.sleep();
            }
        }
    });

    // Set priority for feedback_thread
    pthread_t feedback_native_thread = feedback_thread.native_handle();
    pthread_getschedparam(feedback_native_thread, &policy, &sch_params);
    sch_params.sched_priority = 10;
    if(pthread_setschedparam(feedback_native_thread, SCHED_FIFO, &sch_params)) {
        std::cerr << "Failed to set thread priority for feedback_thread" << std::endl;
    }

    ros::AsyncSpinner spinner(12);
    spinner.start();

    mpc_thread.join();
    low_level_thread.join();
    feedback_thread.join();

    return 0;
}