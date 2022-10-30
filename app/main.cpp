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
    Robot acme_kuka;  // Instantiate Robot object
    std::vector<double> start_pose = {0, 0, 78};

    // Set initial pose
    acme_kuka.set_initial_pose(&start_pose);

    // set final pose
    std::vector<double> end_pose = {0, 0, 58};
    acme_kuka.set_final_pose(&end_pose);

    // Execute the path from initial position to target position
    // by considering the parameters of the manipulator.
    acme_kuka.execute_path();
    return 0;
}
