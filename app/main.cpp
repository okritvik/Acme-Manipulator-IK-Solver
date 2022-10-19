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

#include "../include/Robot.hpp"

int main() {
    Robot acme_kuka;
    std::vector<double> start_pose;
    bool test = acme_kuka.set_initial_pose(&start_pose);
    std::cout << test << " DONE";
}
