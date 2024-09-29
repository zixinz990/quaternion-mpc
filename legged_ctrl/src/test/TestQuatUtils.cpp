//
// Created by Zixin Zhang on 03/13/23.
// Copyright (c) 2022 Robotic Exploration Lab. All rights reserved.
//

#include <iostream>

#include "utils/QuaternionUtils.h"

int main() {
    Eigen::Vector4d q1;
    q1 << 0.7071068, 0.4082483, 0.4082483, 0.4082483; // rotate 90 deg around [1, 1, 1]
    std::cout << "q1: " << q1.transpose() << std::endl;

    Eigen::Vector4d q2;
    q2 << 0.7071068, 0, 0, 0.7071068; // rotate 90 deg around x axis
    std::cout << "q2: " << q2.transpose() << std::endl;

    // test quaternion multiplication
    Eigen::Vector4d q3 = QuaternionUtils::quat_mult(q1, q2);
    std::cout << "q3 = q1 * q2: " << q3.transpose() << std::endl;

    // test quaternion conjugate
    Eigen::Vector4d q4 = QuaternionUtils::quat_conj(q1);
    std::cout << "q4 = q1^*: " << q4.transpose() << std::endl;

    // test G function
    Eigen::MatrixXd G = QuaternionUtils::G(q1);
    std::cout << "G = L(q1) * H: " << std::endl << G << std::endl;
}