#include <vector>

#include "../include/Controller.hpp"
#include "../include/Kinematics.hpp"
#include "../include/Simulator.hpp"
#include "../include/Robot.hpp"

bool Robot::execute_path() {
    return true;
}

bool Robot::set_initial_pose(std::vector<double> pose) {
    return true;
}

bool Robot::set_final_pose(std::vector<double> pose) {
    return true;
}

std::vector<double> Robot::get_joint_angles() {
    std::vector<double> ang;
    return ang;
}

void set_joint_angles(std::vector<double> joint_angles) {
}
