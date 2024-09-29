#include <iostream>
#include <Eigen/Dense>
#include <chrono>
#include <thread>
#include <pthread.h>
#include <cerrno>
#include <cstring>

#include <ros/ros.h>

int main(int argc, char **argv) {
    // Get the current thread
    pthread_t thread = pthread_self();

    // Set the current thread to the highest priority
    sched_param param;
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);

    int result = pthread_setschedparam(thread, SCHED_FIFO, &param);
    if (result != 0) {
        std::cerr << "Failed to set thread priority: " << std::strerror(result) << std::endl;
        return 1;
    }

    // Initialize ROS and random seed
    ros::Time::init();
    std::srand((unsigned int) time(0));

    // Create random matrices
    Eigen::MatrixXd A = Eigen::MatrixXd::Random(100, 100);
    Eigen::MatrixXd B = Eigen::MatrixXd::Random(100, 100);
    Eigen::MatrixXd C = Eigen::MatrixXd::Random(100, 100);

    // Start timer
    std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();

    // Perform matrix calculations
    Eigen::MatrixXd D = A * B;
    Eigen::MatrixXd E = D.inverse();
    double det = E.determinant();
    Eigen::MatrixXd F = C * E;

    // Calculate duration
    std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();    
    std::chrono::duration<double, std::milli> time_span = t2 - t1;

    // Sleep for 5 milliseconds minus time_span
    ros::Duration sleep_duration(0.0049 - time_span.count() / 1000.0);
    if (sleep_duration.toSec() > 0) {
        sleep_duration.sleep();
    }

    std::chrono::high_resolution_clock::time_point t3 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> total_time = t3 - t1;
    std::cout << "Matrix calculation took: " << total_time.count() << " milliseconds" << std::endl;

    return 0;
}
