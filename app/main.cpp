/**
 * @file main.cpp
 * @author Adarsh Malapaka (amalapak@terpmail.umd.edu)
 * @author Kumara Ritvik Oruganti (okritvik@umd.edu)
 * @brief Main function for Acme's Robot Manipulator's API
 * @version 0.1
 * @date 2022-10-18
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <iostream>
#include <vector>

#include "../include/Robot.hpp"

int main() {
    Robot acme_kuka;
    std::vector<double> ja = {0, 0, 0, 0, 0, 0};
    acme_kuka.m_kinematics.fk_solver.set_joint_angles(&ja);
    std::vector<nc::NdArray<double>> temp =
                    acme_kuka.m_kinematics.fk_solver.link_transformation();
    acme_kuka.m_kinematics.fk_solver.get_dh();
    std::vector<double> start_pose;
    bool test = acme_kuka.set_initial_pose(&start_pose);
    acme_kuka.execute_path();
    std::cout << test << " DONE";
}
