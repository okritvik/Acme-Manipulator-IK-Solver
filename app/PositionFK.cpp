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
#define PI nc::constants::pi
#include "../include/PositionFK.hpp"
#include <NumCpp.hpp>

PositionFK::PositionFK() {
}

nc::NdArray<double> PositionFK::get_dh() {
    // Initialize the D-H table parametrically from the joint angles
    PositionFK::m_dh_table = {
                            {0, PositionFK::m_joint_angles[0], -PI/2, 36},
                            {0, PositionFK::m_joint_angles[1], PI/2, 0},
                            {0, 0, PI/2, 42},
                            {0, PositionFK::m_joint_angles[2], -PI/2, 0},
                            {0, PositionFK::m_joint_angles[3], -PI/2, 39.95},
                            {0, PositionFK::m_joint_angles[4], PI/2, 0},
                            {0, PositionFK::m_joint_angles[5], 0, 20.55}
                            };
    return PositionFK::m_dh_table;
}

std::vector<nc::NdArray<double>> PositionFK::link_transformation() {
    std::vector<nc::NdArray<double>> final_transformation;
    // Link base to 1 transformation matrix
    nc::NdArray<double> tr01 = {
                {nc::cos(m_joint_angles[0]), 0, -nc::sin(m_joint_angles[0]), 0},
                {nc::sin(m_joint_angles[0]), 0, nc::cos(m_joint_angles[0]), 0},
                {0, -1, 0, 36},
                {0, 0, 0, 1},
                };
    // Link 1 to 2 transformation matrix
    nc::NdArray<double> tr12 = {
                {nc::cos(m_joint_angles[1]), 0, nc::sin(m_joint_angles[1]), 0},
                {nc::sin(m_joint_angles[1]), 0, -nc::cos(m_joint_angles[1]), 0},
                {0, 1, 0, 0},
                {0, 0, 0, 1},
                };
    // Link 2 to 4 transformation matrix
    nc::NdArray<double> tr24 = {
                {nc::cos(m_joint_angles[2]), 0, -nc::sin(m_joint_angles[2]), 0},
                {0, 1, 0, 0},
                {nc::sin(m_joint_angles[2]), 0, nc::cos(m_joint_angles[2]), 42},
                {0, 0, 0, 1},
                };
    // Link 4 to 5 transformation matrix
    nc::NdArray<double> tr45 = {
                {nc::cos(m_joint_angles[3]), 0, -nc::sin(m_joint_angles[3]), 0},
                {nc::sin(m_joint_angles[3]), 0, nc::cos(m_joint_angles[3]), 0},
                {0, -1, 0, 39.95},
                {0, 0, 0, 1},
                };
    // Link 5 to 6 transformation matrix
    nc::NdArray<double> tr56 = {
                {nc::cos(m_joint_angles[4]), 0, nc::sin(m_joint_angles[4]), 0},
                {nc::sin(m_joint_angles[4]), 0, -nc::cos(m_joint_angles[4]), 0},
                {0, 1, 0, 0},
                {0, 0, 0, 1},
                };
    // Link 6 to end-effector transformation matrix
    nc::NdArray<double> tr6n = {
                {nc::cos(m_joint_angles[5]), -nc::sin(m_joint_angles[5]), 0, 0},
                {nc::sin(m_joint_angles[5]), nc::cos(m_joint_angles[5]), 0, 0},
                {0, 0, 1, 20.55},
                {0, 0, 0, 1},
                };
    // Push the individual successive link transformations to a vector
    final_transformation.push_back(tr01);

    final_transformation.push_back(tr12);

    final_transformation.push_back(tr24);

    final_transformation.push_back(tr45);

    final_transformation.push_back(tr56);

    final_transformation.push_back(tr6n);

    return final_transformation;
}

bool PositionFK::set_joint_angles(std::vector<double> *joint_angles) {
    // Set the robot joint angles
    PositionFK::m_joint_angles = *joint_angles;
    return true;
}
