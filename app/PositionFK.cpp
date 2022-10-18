/**
 * @file PositionFK.cpp
 * @author Adarsh Malapaka (amalapak@terpmail.umd.edu)
 * @author Kumara Ritvik Oruganti (okritvik@umd.edu)
 * @brief Implementation of Acme's Robot Manipulator's Position Kinematics class.
 * @version 0.1
 * @date 2022-10-18
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <vector>

#include "../include/PositionFK.hpp"
#include "NumCpp.hpp"

PositionFK::PositionFK() {
    m_dh_table = 0;
}

nc::NdArray<double> PositionFK::get_dh() {
    nc::NdArray<double> dh;
    return dh;
}

nc::NdArray<double> PositionFK::link_transformation() {
    nc::NdArray<double> tr;
    return tr;
}

void PositionFK::set_joint_angles(std::vector<double> joint_angles) {
}
