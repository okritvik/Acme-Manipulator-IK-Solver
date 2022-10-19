/**
 * @file Robot.cpp
 * @author Adarsh Malapaka (amalapak@terpmail.umd.edu)
 * @author Kumara Ritvik Oruganti (okritvik@umd.edu)
 * @brief Implementation of Acme's Robot Manipulator's Robot class.
 * @version 0.1
 * @date 2022-10-18
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include <vector>

#include "../include/Controller.hpp"
#include "../include/Kinematics.hpp"
#include "../include/Simulator.hpp"
#include "../include/Robot.hpp"

bool Robot::execute_path() {
    return true;
}

bool Robot::set_initial_pose(std::vector<double> *pose) {
    return true;
}

bool Robot::set_final_pose(std::vector<double> *pose) {
    return true;
}

std::vector<double> Robot::get_joint_angles() {
    std::vector<double> ang;
    return ang;
}

void set_joint_angles(std::vector<double> *joint_angles) {
}
