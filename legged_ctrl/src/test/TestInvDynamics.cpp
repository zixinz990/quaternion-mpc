#include <Eigen/Dense>
#include <chrono>
#include <iostream>

#include "utils/Go1Dynamics.h"

int main(int, char **) {
    legged::Go1Dynamics test_dyn;

    Eigen::Vector3d tau;
    Eigen::Vector3d q;
    Eigen::Vector3d v;
    Eigen::Vector3d a;

    q << 0.0, 0.2, -0.4;
    v << 0.0, 0.0, 0.0;
    a << 0.0, 0.0, 0.0;
    std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
    test_dyn.go1_FL_inverse_dynamics_task_space(q, v, a, tau);
    std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> time_span = t2 - t1;
    std::cout << "time: " << time_span.count() << "ms" << std::endl;
    std::cout << tau << std::endl;
    return 0;
}
