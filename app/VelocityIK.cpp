/**
 * @file VelocityIK.cpp
 * @author Adarsh Malapaka (amalapak@terpmail.umd.edu)
 * @author Kumara Ritvik Oruganti (okritvik@umd.edu)
 * @brief Implementation of Acme's Robot Manipulator's Velocity Kinematics class.
 * @version 0.1
 * @date 2022-10-18
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <matplot/matplot.h>
#include <cstddef>
#include <vector>
#include <NumCpp/Functions/ones.hpp>
#include <NumCpp/NdArray/NdArrayCore.hpp>
#include <NumCpp.hpp>
#include "../include/VelocityIK.hpp"

#define PI nc::constants::pi
// using namespace nc;

VelocityIK::VelocityIK() {
}

nc::NdArray<double> VelocityIK::cartesian_velocity(double *theta) {
    // Compute the x velocity and z velocity of the manipulator
    // end-effector frame.
    nc::NdArray<double> X_dot;
    double x_dot = -4.0 * PI * sin(*theta);
    double z_dot = 4.0 * PI * cos(*theta);
    X_dot = {{x_dot, 0.0, z_dot, 0, 0, 0}};
    return X_dot;
}

std::vector<double> VelocityIK::update_joint_angles(double *dt,
                std::vector<double> *present_joint_angle,
                std::vector<double> *joint_angle_dot) {
    // Using next_q = prev_q + q_dot * dt
    std::vector<double> next_joint_angle;
    std::vector<double> p_joint_angle = *present_joint_angle;
    std::vector<double> j_angle_dot =  *joint_angle_dot;
    // Update the joint angles
    for (size_t i = 0; i < p_joint_angle.size(); i++) {
       next_joint_angle.push_back(p_joint_angle.at(i) +
        j_angle_dot.at(i) * (*dt));
    }

    return next_joint_angle;
}

bool VelocityIK::compute_jacobian(std::vector<double> *joint_angle) {
    std::vector<double> thetas = *joint_angle;
    m_jacobian = nc::ones<double>(6, 6);
    // get the joint angles for use of parametrical computation of J matrix
    auto q1 = thetas[0];
    auto q2 = thetas[1];
    auto q4 = thetas[2];
    auto q5 = thetas[3];
    auto q6 = thetas[4];
    // auto q7 = thetas[5];

    // 1st row
    m_jacobian.at(0, 0) = -42.0 * sin(q1) * sin(q2)
    - 20.55 * sin(q1) * sin(q6) * cos(q5) * cos(q2 - q4) - 20.55
    * sin(q1) * sin(q2 - q4) * cos(q6) - 39.95 * sin(q1) * sin(q2 - q4)
    - 20.55 * sin(q5) * sin(q6) * cos(q1);

    m_jacobian.at(0, 1) = (-20.55 * sin(q6) * sin(q2 - q4)
    * cos(q5) + 42.0 * cos(q2) + 20.55 * cos(q6) * cos(q2 - q4) + 39.95
    * cos(q1 - q4)) * cos(q1);

    m_jacobian.at(0, 2) = (20.55 * sin(q6) * sin(q2 - q4)
    * cos(q5) - 20.55 * cos(q6) * cos(q2 - q4) - 39.95 * cos(q2 - q4))
    * cos(q1);

    m_jacobian.at(0, 3) = -20.55 * (sin(q1) * cos(q5)
    + sin(q5) * cos(q1) * cos(q2 - q4)) * sin(q6);

    m_jacobian.at(0, 4) = 20.55 * (-sin(q1) * sin(q5)
    + cos(q1) * cos(q5) * cos(q2 - q4)) * cos(q6) - 20.55 * sin(q6)
    * sin(q2 - q4) * cos(q1);

    m_jacobian.at(0, 5) = 0;

    // 2nd row
    m_jacobian.at(1, 0) = -20.55 * sin(q1) * sin(q5)
    * sin(q6) + 42.0 * sin(q2) * cos(q1) + 20.55
    * sin(q6) * cos(q1) * cos(q5) * cos(q2
    - q4) + 20.55 * sin(q2 - q4) * cos(q1)
    * cos(q6) + 39.95 * sin(q2 - q4) * cos(q1);

    m_jacobian.at(1, 1) = (-20.55 * sin(q6) * sin(q2
    - q4) * cos(q5) + 42.0 * cos(q2) + 20.55
    * cos(q6) * cos(q2 - q4) + 39.95
    * cos(q2 - q4)) * sin(q1);

    m_jacobian.at(1, 2) = (20.55 * sin(q6) * sin(q2
    - q4) * cos(q5) - 20.55 * cos(q6) * cos(q2
    - q4) - 39.95 * cos(q2 - q4)) * sin(q1);

    m_jacobian.at(1, 3) = -20.55 * (sin(q1)
    * sin(q5) * cos(q2 - q4) - cos(q1)
    * cos(q5)) * sin(q6);

    m_jacobian.at(1, 4) = 20.55 * (sin(q1) * cos(q5)
    * cos(q2 - q4) + sin(q5) * cos(q1))
    * cos(q6) - 20.55 * sin(q1) * sin(q6) * sin(q2
    - q4);

    m_jacobian.at(1, 5) = 0;

    // 3rd row
    m_jacobian.at(2, 0) = 0;

    m_jacobian.at(2, 1) = -42.0 * sin(q2) - 20.55 * sin(q6) * cos(q5)
    * cos(q2 - q4) - 20.55 * sin(q2 - q4) * cos(q6) - 39.95 * sin(q2 - q4);

    m_jacobian.at(2, 2) = 20.55 * sin(q6) * cos(q5) * cos(q2 - q4)
    + 20.55 * sin(q2 - q4) * cos(q6) + 39.95 * sin(q2 - q4);

    m_jacobian.at(2, 3) = 20.55 * sin(q5) * sin(q6) * sin(q2 - q4);

    m_jacobian.at(2, 4) =  -20.55 * sin(q6) * cos(q2 - q4) - 20.55
    * sin(q2 - q4) * cos(q5) * cos(q6);

    m_jacobian.at(2, 5) = 0;

    // 4th row
    m_jacobian.at(3, 0) = 0;

    m_jacobian.at(3, 1) = -sin(q1);

    m_jacobian.at(3, 2) = sin(q2) * cos(q1);

    m_jacobian.at(3, 3) = sin(q2 - q4) * cos(q1);

    m_jacobian.at(3, 4) = -sin(q1) * cos(q5) - sin(q5) * cos(q1)
    * cos(q2 - q4);

    m_jacobian.at(3, 5) = (-sin(q1) * sin(q5) + cos(q1) * cos(q5)
    * cos(q2 - q4)) * sin(q6) + sin(q2 - q4) * cos(q1) * cos(q6);

    // 5th row
    m_jacobian.at(4, 0) = 0;

    m_jacobian.at(4, 1) = cos(q1);

    m_jacobian.at(4, 2) = sin(q1) * sin(q2);

    m_jacobian.at(4, 3) = sin(q1) * sin(q2 - q4);

    m_jacobian.at(4, 4) = -sin(q1) * sin(q5) * cos(q2 - q4) + cos(q1) * cos(q5);

    m_jacobian.at(4, 5) = (sin(q1) * cos(q5) * cos(q2 - q4) + sin(q5) * cos(q1))
    * sin(q6) + sin(q1) * sin(q2 - q4) * cos(q6);

    // 6th row
    m_jacobian.at(5, 0) = 1;

    m_jacobian.at(5, 1) = 0;

    m_jacobian.at(5, 2) = cos(q2);

    m_jacobian.at(5, 3) = cos(q2 - q4);

    m_jacobian.at(5, 4) = sin(q5) * sin(q2 - q4);

    m_jacobian.at(5, 5) = -sin(q6) * sin(q2 - q4) * cos(q5)
    + cos(q6) * cos(q2 - q4);

    return true;
}


nc::NdArray<double> VelocityIK::get_jacobian() {
    // Return the jacobian matrix
    return m_jacobian;
}
