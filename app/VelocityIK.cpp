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

#include "../include/VelocityIK.hpp"
#include <NumCpp.hpp>


std::vector<double> VelocityIK::cartesian_velocity() {
    std::vector<double> vel;
    return vel;
}

std::vector<double> VelocityIK::update_joint_angles(double *dt,
                std::vector<double> *present_joint_angle) {
    std::vector<double> ang;
    return ang;
}

void VelocityIK::compute_jacobian() {
}

nc::NdArray<double> VelocityIK::get_jacobian() {
    nc::NdArray<double> jac;
    return jac;
}
