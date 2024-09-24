//
// Created by Zixin Zhang on 03/11/23.
// Copyright (c) 2022 Robotic Exploration Lab. All rights reserved.
//

#pragma once

#include <iostream>
#include <vector>
#include <Eigen/Dense>

#include "Utils.h"

class QuaternionUtils {
public:
    // Cayley Map: phi to quaternion
    static Eigen::Vector4d cayley_map(Eigen::Vector3d phi);

    // Inverse Cayley Map: quaternion to phi
    static Eigen::Vector3d inv_cayley_map(Eigen::Vector4d q);

    // Quaternion multiplication
    static Eigen::Vector4d quat_mult(Eigen::Vector4d q1, Eigen::Vector4d q2);

    // Quternion conjugate
    static Eigen::Vector4d quat_conj(Eigen::Vector4d q);

    // L function
    static Eigen::Matrix4d L(Eigen::Vector4d q);

    // R function
    static Eigen::Matrix4d R(Eigen::Vector4d q);

    // G function
    static Eigen::MatrixXd G(Eigen::Vector4d q);

};
