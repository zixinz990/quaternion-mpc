#include <Eigen/Dense>
#include <chrono>
#include <iostream>

#include "utils/A1Kinematics.h"

int main(int, char **) {
    legged::A1Kinematics test_kin;

    // generate some theta value
    srand((unsigned)time(NULL));
    Eigen::Vector3d q;

    double ox = 0.1881;
    double oy = -0.04675;
    double d = -0.0812;
    double lt = 0.213;
    double lc = 0.213;

    Eigen::VectorXd rho_fix(5);
    rho_fix << ox, oy, d, lt, lc;

    Eigen::VectorXd rho_opt(3);
    rho_opt << 0.0, 0.0, 0.0;
    
    Eigen::Vector3d cur_q;
    cur_q << -0.224375, 0.466764, -1.39401;

    Eigen::Vector3d foot_pos;
    foot_pos << 0.23391, -0.364016, -0.294254;

    Eigen::Vector3d inv_kin_sol;
    inv_kin_sol = test_kin.inv_kin(foot_pos, cur_q, rho_opt, rho_fix);
    std::cout << "inv_kin_sol: " << std::endl << inv_kin_sol << std::endl;

    // for (int i = 0; i < 2000; i++) {
    //     // q1: [-46, 46] deg = [-0.8, 0.8] rad
    //     // q2: [-60, 240] deg = [-1.05, 4.19] rad
    //     // q3: [-154.5, -52.5] = [-2.69, -0.92] rad
    //     q << (double)rand() / RAND_MAX * M_PI / 180 * 46 * 2 - M_PI / 180 * 46,
    //          (double)rand() / RAND_MAX * M_PI / 180 * 300 - M_PI / 180 * 60,
    //          -(double)rand() / RAND_MAX * M_PI / 180 * 102 - M_PI / 180 * 52.5;

    //     Eigen::Vector3d cur_q;
    //     cur_q << q[0], 0, 0;

    //     // calculate fk
    //     Eigen::Vector3d foot_pos;
    //     foot_pos = test_kin.fk(q, rho_opt, rho_fix);

    //     // calculate ik
    //     Eigen::Vector3d inv_kin_sol;
    //     std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
    //     inv_kin_sol = test_kin.inv_kin(foot_pos, cur_q, rho_opt, rho_fix);
    //     std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
    //     std::chrono::duration<double, std::milli> time_span = end - start;
    //     std::cout << "IK time: " << time_span.count() << " ms " << std::endl;
        
    //     if (abs(inv_kin_sol[0] - q[0]) > 0.001 || abs(inv_kin_sol[1] - q[1]) > 0.001 || abs(inv_kin_sol[2] - q[2]) > 0.001) {
    //         std::cout << "Something wrong!";
    //         std::cout << "q:" << std::endl << q << std::endl;
    //         std::cout << "inv_kin_sol: " << std::endl << inv_kin_sol << std::endl;
    //         std::cout << "foot pos: " << std::endl << foot_pos << std::endl;
    //     }
    // }
    return 0;
}
